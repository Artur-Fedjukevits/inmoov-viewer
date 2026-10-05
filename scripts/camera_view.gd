## Robot camera video window (OAK-D Lite) — on the right, over the 3D view.
##
## The robot sends each frame as a single binary WebSocket message (ws://<ip>:9091), as is:
## JPEG (default), PNG or WebP — the format is detected by its signature.
## If frames arrive faster than we draw them, only the latest one is taken from the queue
## (the rest are dropped without decoding) — latency does not build up.
##
##   var cv := CameraView.new(); layer.add_child(cv); cv.connect_to("192.168.10.202")
class_name CameraView
extends PanelContainer

signal stream_state_changed(text: String)
signal frame_received(size: Vector2i)

const DEFAULT_PORT := 9091
const SIZES := [360.0, 560.0]       ## window width: compact / large (tap the picture)

@export var reconnect_interval := 2.0
## Window insets from the right/top screen edge (main.gd accounts for the cutout and rounded corners).
var inset_right := 8.0
var inset_top := 8.0

var url := ""
var frames := 0                     ## frames decoded since connecting
var dropped := 0                    ## stale frames dropped (not decoded)
var bad := 0                        ## failed to decode
var last_frame_size := Vector2i.ZERO
var last_frame_msec := -1

var _ws := WebSocketPeer.new()
var _want_open := false
var _was_open := false
var _retry_in := 0.0
var _tex: ImageTexture
var _img := Image.new()
var _view: TextureRect
var _status: Label
var _hint: Label
var _size_idx := 0
var _stat_acc := 0.0
var _stat_frames := 0
var _stat_bytes := 0
var _fps := 0.0
var _kbps := 0.0


func _init() -> void:
	# A 640×480 JPEG q80 frame is ≈ 30–80 KB, while the default receive buffer is 64 KB — frames would not get through.
	_ws.inbound_buffer_size = 4 * 1024 * 1024
	_ws.max_queued_packets = 64


func _ready() -> void:
	anchor_left = 1.0
	anchor_right = 1.0
	mouse_filter = Control.MOUSE_FILTER_STOP
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.08, 0.08, 0.09, 0.92)
	sb.set_corner_radius_all(6)
	sb.set_content_margin_all(6)
	add_theme_stylebox_override("panel", sb)

	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 4)
	add_child(box)
	var head := HBoxContainer.new()
	box.add_child(head)
	var title := Label.new()
	title.text = "Camera"
	head.add_child(title)
	_status = Label.new()
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_status.add_theme_font_size_override("font_size", 13)
	_status.clip_text = true
	head.add_child(_status)

	var frame := Control.new()          # fixed 4:3 area, image inside
	frame.size_flags_vertical = Control.SIZE_EXPAND_FILL
	box.add_child(frame)
	_view = TextureRect.new()
	_view.set_anchors_preset(Control.PRESET_FULL_RECT)
	_view.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_view.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	_view.mouse_filter = Control.MOUSE_FILTER_STOP
	_view.gui_input.connect(_on_view_input)
	frame.add_child(_view)
	_hint = Label.new()
	_hint.set_anchors_preset(Control.PRESET_FULL_RECT)
	_hint.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_hint.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_hint.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_hint.modulate = Color(1, 1, 1, 0.6)
	frame.add_child(_hint)
	_apply_size()
	_set_state("no link", "No video.\nConnect to the robot on the “Robot” tab.")


## host: "192.168.10.202", "ws://192.168.10.202:9090" (port is replaced with 9091) or a full ws://host:port.
static func make_url(host: String, port := DEFAULT_PORT) -> String:
	var h := host.strip_edges()
	for p in ["ws://", "wss://", "http://"]:
		if h.begins_with(p):
			h = h.substr(p.length())
	h = h.get_slice("/", 0)
	if h.contains(":"):
		h = h.get_slice(":", 0)
	return "ws://%s:%d" % [h, port]


func connect_to(host_or_url: String, port := DEFAULT_PORT) -> void:
	url = make_url(host_or_url, port)
	_want_open = true
	_open()


func disconnect_stream() -> void:
	_want_open = false
	_ws.close(1000, "bye")
	_set_state("disconnected", "Video disconnected.")


func is_streaming() -> bool:
	return _ws.get_ready_state() == WebSocketPeer.STATE_OPEN and last_frame_msec >= 0 \
		and Time.get_ticks_msec() - last_frame_msec < 1500


func _open() -> void:
	frames = 0; dropped = 0; bad = 0; last_frame_msec = -1
	_ws.close()
	var err := _ws.connect_to_url(url)
	if err != OK:
		_set_state("error %d" % err, "Could not open %s" % url)
		_retry_in = reconnect_interval
	else:
		_set_state("connecting…", "Connecting to %s…" % url)


func _process(dt: float) -> void:
	if url == "":
		return
	_ws.poll()
	var st := _ws.get_ready_state()
	if st == WebSocketPeer.STATE_OPEN:
		if not _was_open:
			_was_open = true
			_set_state("waiting for frames…", "Connected to %s, waiting for frames…" % url)
		var latest := PackedByteArray()
		var n := _ws.get_available_packet_count()
		for i in n:                       # take only the newest frame
			var pkt := _ws.get_packet()
			if _ws.was_string_packet():
				continue                  # ignore text messages
			if not latest.is_empty():
				dropped += 1
			latest = pkt
		if not latest.is_empty():
			_show(latest)
	elif st == WebSocketPeer.STATE_CLOSED:
		if _was_open:
			_was_open = false
			_set_state("link lost", "Camera link lost (%d %s)." % [_ws.get_close_code(), _ws.get_close_reason()])
			_retry_in = reconnect_interval
		if _want_open:
			_retry_in -= dt
			if _retry_in <= 0.0:
				_retry_in = reconnect_interval
				_open()
	_update_stats(dt)


func _show(data: PackedByteArray) -> void:
	var err := ERR_FILE_UNRECOGNIZED
	if data.size() > 3 and data[0] == 0xFF and data[1] == 0xD8:
		err = _img.load_jpg_from_buffer(data)
	elif data.size() > 8 and data[0] == 0x89 and data[1] == 0x50:
		err = _img.load_png_from_buffer(data)
	elif data.size() > 12 and data[0] == 0x52 and data[8] == 0x57:     # RIFF....WEBP
		err = _img.load_webp_from_buffer(data)
	if err != OK:
		bad += 1
		if frames == 0:
			_set_state("unknown format", "Frame not recognized (%d bytes, starts with %s)." % [data.size(), data.slice(0, 4).hex_encode()])
		return
	var sz := _img.get_size()
	if _tex == null or Vector2i(_tex.get_size()) != sz or _tex.get_format() != _img.get_format():
		_tex = ImageTexture.create_from_image(_img)
		_view.texture = _tex
	else:
		_tex.update(_img)
	frames += 1
	_stat_frames += 1
	_stat_bytes += data.size()
	last_frame_size = sz
	last_frame_msec = Time.get_ticks_msec()
	if frames == 1:
		_hint.text = ""
	frame_received.emit(sz)


func _update_stats(dt: float) -> void:
	_stat_acc += dt
	if _stat_acc < 1.0:
		return
	_fps = _stat_frames / _stat_acc
	_kbps = _stat_bytes / 1024.0 / _stat_acc
	_stat_acc = 0.0; _stat_frames = 0; _stat_bytes = 0
	if _ws.get_ready_state() != WebSocketPeer.STATE_OPEN:
		return
	if last_frame_msec >= 0 and Time.get_ticks_msec() - last_frame_msec > 1500:
		_set_state("no frames", "Connected, but no frames for more than 1.5 s.")
		_hint.text = "No frames > 1.5 s"
	elif frames > 0:
		_status.text = "%d×%d  %.1f fps  %d KB/s%s" % [last_frame_size.x, last_frame_size.y, _fps, int(_kbps),
			("  dropped %d" % dropped) if dropped > 0 else ""]
		_status.modulate = Color(0.55, 0.9, 0.55)
		stream_state_changed.emit(_status.text)


func _set_state(short: String, long: String) -> void:
	_status.text = short
	_status.modulate = Color(0.95, 0.65, 0.3)
	if frames == 0 or long.begins_with("Camera link"):
		_hint.text = long
	stream_state_changed.emit(short)


## Touches on the window must not rotate the 3D camera.
func _gui_input(e: InputEvent) -> void:
	if e is InputEventScreenTouch or e is InputEventScreenDrag:
		accept_event()


## Tap/click on the picture toggles the window size. On a phone a tap also arrives as an
## InputEventMouseButton (emulate_mouse_from_touch), so ScreenTouch is ignored — otherwise it would toggle twice.
func _on_view_input(e: InputEvent) -> void:
	if e is InputEventScreenTouch or e is InputEventScreenDrag:
		accept_event()
		return
	if e is InputEventMouseButton and e.pressed and e.button_index == MOUSE_BUTTON_LEFT:
		_size_idx = (_size_idx + 1) % SIZES.size()
		_apply_size()
		accept_event()


func _apply_size() -> void:
	apply_layout()


func apply_layout() -> void:
	var w: float = SIZES[_size_idx]
	var h := w * 3.0 / 4.0 + 40.0       # 4:3 + title
	offset_left = -w - inset_right
	offset_right = -inset_right
	offset_top = inset_top
	offset_bottom = inset_top + h
