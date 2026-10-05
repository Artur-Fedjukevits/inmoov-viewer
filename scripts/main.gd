# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## Main scene: joint sliders, click-to-pick body parts, gestures, self-collision checking, robot link, FPS.
extends Node3D

@onready var robot: InmoovRobot = $Robot
@onready var cam: Camera3D = $Camera

const SETTINGS := "user://robot_link.cfg"
const PANEL_W := 440.0          ## left panel width, px
const MARGIN := 16.0            ## margin of the panel and camera window from the screen edges
const TAP_SLOP := 12.0          ## movement (px) after which a touch counts as a rotation, not a click
const TAP_MAX_MSEC := 500
const PICK_TOL := 0.02          ## m: a movable part within 2 cm behind the shell wins over the shell
const GESTURE_DIR := "user://gestures"

## Face joints are a separate set when saving gestures (FACE_JOINTS on the robot); everything else is body (BODY_JOINTS).
const FACE_PATTERNS := ["jaw", "upper_lip", "cheek", "eyebrow", "forehead", "eye_horizontal", "eye_vertical", "eyelid"]

## Body parts selectable by click: [name, [joint-name substrings]].
## A part may have several DOF (shoulder 3, neck 3, eye 2): all of the part's sliders are highlighted.
const PARTS := [
	["Torso", ["torso_"]],
	["Neck / head", ["i01_head_neck_001", "neck_joint", "i01_head_rothead"]],
	["L. shoulder", ["left_shoulder"]], ["R. shoulder", ["right_shoulder"]],
	["L. elbow", ["left_elbow"]], ["R. elbow", ["right_elbow"]],
	["L. wrist", ["left_wrist"]], ["R. wrist", ["right_wrist"]],
	["L. thumb", ["i02_leftHand_Thumb"]], ["L. index", ["i02_leftHand_index"]],
	["L. middle", ["i02_leftHand_majeure"]], ["L. ring", ["i02_leftHand_ringFinger"]],
	["L. pinky", ["i02_leftHand_pinky"]],
	["R. thumb", ["i02_rightHand_Thumb"]], ["R. index", ["i02_rightHand_index"]],
	["R. middle", ["i02_rightHand_majeure"]], ["R. ring", ["i02_rightHand_ringFinger"]],
	["R. pinky", ["i02_rightHand_pinky"]],
	["Jaw", ["i02_head_jaw"]], ["Upper lip", ["upper_lip"]],
	["L. cheek", ["left_cheek"]], ["R. cheek", ["right_cheek"]],
	["L. eyebrow", ["left_eyebrow"]], ["R. eyebrow", ["right_eyebrow"]],
	["L. forehead", ["left_forehead"]], ["R. forehead", ["right_forehead"]],
	["L. eye", ["left_eye_"]], ["R. eye", ["right_eye_"]],
	["L. upper eyelid", ["left_top_eyelid"]], ["L. lower eyelid", ["left_bottom_eyelid"]],
	["R. upper eyelid", ["right_top_eyelid"]], ["R. lower eyelid", ["right_bottom_eyelid"]],
]

## Joint tabs. “Robot” (link) is first and open at startup, “Gestures” is last.
const PAGES := [
	["Torso/neck", ["torso_z", "torso_y", "i01_head_neck_001", "neck_joint", "i01_head_rothead"]],
	["L. arm", ["left_shoulder_y", "left_shoulder_x", "left_shoulder_z", "left_elbow_x", "left_wrist_z"]],
	["R. arm", ["right_shoulder_y", "right_shoulder_x", "right_shoulder_z", "right_elbow_x", "right_wrist_z"]],
	["L. hand", ["i02_leftHand_"]],
	["R. hand", ["i02_rightHand_"]],
	["Face", ["jaw", "upper_lip", "cheek", "eyebrow", "forehead"]],
	["Eyes", ["eye_horizontal", "eye_vertical", "eyelid"]],
]

var link: RobotLink
var camera_view: CameraView
var _cam_toggle: CheckButton

var _sliders := {}
var _rows := {}              ## joint -> row (PanelContainer), for highlighting
var _joint_page := {}        ## joint -> tab index
var _panel: PanelContainer
var _scroll: ScrollContainer
var _tab_buttons: Array[Button] = []
var _status: Label
var _fps: Label
var _conn_lab: Label
var _sel_label: Label
var _block: CheckButton
var _random_btn: Button
var _lock := false
var _last_ok := {}   ## last pose without collisions (for block mode)
var _selected_joints: Array = []

# “Robot” tab
var _url: LineEdit
var _conn_btn: Button
var _link_status: Label
var _fresh: Label
var _mode_btns: Array[Button] = []
var _calib_box: VBoxContainer
var _calib_joint: OptionButton
var _calib_info: Label
var _calib_servo: HSlider
var _calib_servo_lab: Label
var _calib_model: HSlider
var _calib_model_lab: Label
var _calib_points: Array = []    ## [[servo_deg, urdf_rad], ...]
var _calib_lock := false
var _release_btn: Button
var _help: Label

# “Gestures” tab
var _gest_name: LineEdit
var _gest_status: Label
var _gest_touched: Label
var _touched := {}               ## joint -> true: what the user moved since the last “Reset”/gesture save
var _gest_lists := {}            ## "body"/"face" -> VBoxContainer

# click on the robot
var _tap_start := Vector2.ZERO
var _tap_msec := 0
var _tap_ok := false
var _touch_ids := {}


func _ready() -> void:
	await get_tree().process_frame   # robot._ready has already run
	link = RobotLink.new()
	link.name = "RobotLink"
	link.robot = robot
	add_child(link)
	_build_ui()
	_layout()
	get_viewport().size_changed.connect(_layout)
	robot.self_collision_changed.connect(_on_collision)
	link.link_status_changed.connect(func(t): _link_status.text = t)
	link.real_pose_received.connect(_on_real_pose)
	link.map_received.connect(_on_map)
	link.ros.connected.connect(_update_controls)
	link.ros.disconnected.connect(_update_controls)
	_last_ok = robot.get_pose()
	_update_controls()
	_refresh_gesture_lists()
	_update_touched_label()


## Safe-area insets (camera cutout, rounded corners) in viewport px: left, top, right, bottom.
func _safe_insets() -> Vector4:
	if not (OS.get_name() in ["Android", "iOS"]):
		return Vector4.ZERO
	var win := Vector2(DisplayServer.window_get_size())
	if win.x <= 0.0 or win.y <= 0.0:
		return Vector4.ZERO
	var vp := get_viewport().get_visible_rect().size
	var s := Vector2(vp.x / win.x, vp.y / win.y)
	var sa := DisplayServer.get_display_safe_area()
	return Vector4(maxf(sa.position.x, 0.0) * s.x, maxf(sa.position.y, 0.0) * s.y,
		maxf(win.x - sa.end.x, 0.0) * s.x, maxf(win.y - sa.end.y, 0.0) * s.y)


## Position of the panel/camera window + centering the robot in the free part of the screen.
func _layout() -> void:
	if _panel == null:
		return
	var ins := _safe_insets()
	var vp := get_viewport().get_visible_rect().size
	_panel.anchor_left = 0.0
	_panel.anchor_right = 0.0
	_panel.anchor_top = 0.0
	_panel.anchor_bottom = 1.0
	_panel.offset_left = ins.x + MARGIN
	_panel.offset_top = ins.y + MARGIN
	_panel.offset_right = _panel.offset_left + minf(PANEL_W, vp.x * 0.62)
	_panel.offset_bottom = -(ins.w + MARGIN)
	camera_view.inset_right = ins.z + MARGIN
	camera_view.inset_top = ins.y + MARGIN
	camera_view.apply_layout()
	(cam as Node).call("set_left_inset", _panel.offset_right + MARGIN)


## Tabs instead of a long list — easier on a phone; elements enlarged for fingers.
func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	_panel = PanelContainer.new()
	var ps := StyleBoxFlat.new()
	ps.bg_color = Color(0.13, 0.13, 0.15, 0.92)
	ps.set_corner_radius_all(14)
	ps.content_margin_left = 14
	ps.content_margin_right = 14
	ps.content_margin_top = 12
	ps.content_margin_bottom = 12
	_panel.add_theme_stylebox_override("panel", ps)
	layer.add_child(_panel)
	var root := VBoxContainer.new()
	root.add_theme_constant_override("separation", 6)
	_panel.add_child(root)

	var top := HBoxContainer.new()
	top.add_theme_constant_override("separation", 10)
	root.add_child(top)
	_status = Label.new()
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.text = "No self-collisions"
	top.add_child(_status)
	_conn_lab = Label.new()                 # link status — next to FPS
	_conn_lab.text = "○ No link"
	top.add_child(_conn_lab)
	_fps = Label.new()
	_fps.custom_minimum_size = Vector2(72, 0)
	_fps.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	top.add_child(_fps)

	var row := HBoxContainer.new()
	root.add_child(row)
	var reset_btn := _button("Reset", _reset_pose)
	row.add_child(reset_btn)
	_random_btn = _button("Random", _random_pose)
	row.add_child(_random_btn)
	var cb_shapes := CheckButton.new(); cb_shapes.text = "Shapes"; row.add_child(cb_shapes)
	cb_shapes.toggled.connect(robot.set_collision_shapes_visible)
	_cam_toggle = CheckButton.new(); _cam_toggle.text = "Camera"; _cam_toggle.button_pressed = true
	row.add_child(_cam_toggle)
	_cam_toggle.toggled.connect(_on_cam_toggled)

	# video window on the right (OAK-D Lite, ws://<ip>:9091)
	camera_view = CameraView.new()
	camera_view.name = "CameraView"
	layer.add_child(camera_view)
	_block = CheckButton.new(); _block.text = "Block collisions"
	_block.button_pressed = true
	root.add_child(_block)

	var tabs := HFlowContainer.new()
	root.add_child(tabs)
	var group := ButtonGroup.new()
	var slider_theme := _make_slider_theme()

	_sel_label = Label.new()
	_sel_label.add_theme_font_size_override("font_size", 13)
	_sel_label.modulate = Color(0.5, 0.75, 1.0)
	_sel_label.visible = false
	root.add_child(_sel_label)

	# pages live in a scrollable area: the panel never extends past the screen
	_scroll = ScrollContainer.new()
	_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	root.add_child(_scroll)
	var pages_box := VBoxContainer.new()
	pages_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_scroll.add_child(pages_box)

	# 0 — “Robot” (main, open at startup)
	_add_page(tabs, group, pages_box, "Robot", _build_robot_page(slider_theme))

	# joints by tab; anything unmatched goes to the last joint tab
	var per_page: Array = []
	for pi in PAGES.size():
		per_page.append([])
	var used := {}
	for pi in PAGES.size():
		for jn: String in robot.controllable:
			if used.has(jn):
				continue
			for pat: String in PAGES[pi][1]:
				if jn.contains(pat):
					used[jn] = true
					per_page[pi].append(jn)
					break
	for jn: String in robot.controllable:
		if not used.has(jn):
			per_page[-1].append(jn)
	for pi in PAGES.size():
		var page := VBoxContainer.new()
		page.add_theme_constant_override("separation", 2)
		# names are right-aligned and pressed against the sliders: column width is set by the longest name on the tab
		var w := 0.0
		for jn: String in per_page[pi]:
			w = maxf(w, ThemeDB.fallback_font.get_string_size(_short(jn), HORIZONTAL_ALIGNMENT_LEFT, -1, 14).x)
		_add_page(tabs, group, pages_box, PAGES[pi][0], page)
		for jn: String in per_page[pi]:
			page.add_child(_make_row(jn, slider_theme, ceilf(w) + 2.0))
			_joint_page[jn] = _pages.size() - 1

	_add_page(tabs, group, pages_box, "Gestures", _build_gestures_page())
	(tabs.get_child(0) as Button).button_pressed = true
	_show_page(0)


var _pages: Array[Control] = []


func _add_page(tabs: Container, group: ButtonGroup, parent: Container, title: String, page: Control) -> void:
	var b := Button.new()
	b.text = title
	b.toggle_mode = true
	b.button_group = group
	b.custom_minimum_size = Vector2(0, 44)
	b.pressed.connect(_show_page.bind(_pages.size()))
	tabs.add_child(b)
	_tab_buttons.append(b)
	page.visible = false
	page.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	parent.add_child(page)
	_pages.append(page)


func _button(text: String, cb: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.custom_minimum_size = Vector2(0, 48)
	b.pressed.connect(cb)
	return b


func _row_style(selected: bool) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.set_content_margin_all(3)
	sb.set_corner_radius_all(8)
	if selected:
		sb.bg_color = Color(0.2, 0.5, 1.0, 0.30)
		sb.set_border_width_all(2)
		sb.border_color = Color(0.45, 0.72, 1.0)
	else:
		sb.bg_color = Color(0, 0, 0, 0)
	return sb


func _make_row(jn: String, th: Theme, label_w: float) -> Control:
	var row := PanelContainer.new()
	row.add_theme_stylebox_override("panel", _row_style(false))
	var r := HBoxContainer.new()
	r.add_theme_constant_override("separation", 6)
	row.add_child(r)
	var lab := Label.new()
	lab.text = _short(jn)
	lab.custom_minimum_size = Vector2(label_w, 0)
	lab.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	lab.add_theme_font_size_override("font_size", 14)
	r.add_child(lab)
	var lim := robot.get_limits(jn)
	var s := _make_slider(th, rad_to_deg(lim.x), rad_to_deg(lim.y), 0.5)
	s.value_changed.connect(_on_slider.bind(jn))
	r.add_child(s)
	_sliders[jn] = s
	_rows[jn] = row
	return row


func _make_slider(th: Theme, mn: float, mx: float, step: float) -> HSlider:
	var s := HSlider.new()
	s.theme = th
	s.min_value = mn
	s.max_value = mx
	s.step = step
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	s.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	s.custom_minimum_size = Vector2(0, 52)
	s.focus_mode = Control.FOCUS_NONE
	return s


static func _short(jn: String) -> String:
	return jn.replace("_joint", "").replace("i02_leftHand_", "L ").replace("i02_rightHand_", "R ") \
		.replace("i02_head_", "").replace("i01_head_", "")


## Large slider knob (40 px circle) and thick track — easy to use with a finger.
func _make_slider_theme() -> Theme:
	var img := Image.create(40, 40, false, Image.FORMAT_RGBA8)
	for y in 40:
		for x in 40:
			var d := Vector2(x - 19.5, y - 19.5).length()
			img.set_pixel(x, y, Color(0.95, 0.95, 0.95, clampf(19.5 - d, 0.0, 1.0)))
	var tex := ImageTexture.create_from_image(img)
	var track := StyleBoxFlat.new()
	track.bg_color = Color(0.25, 0.25, 0.27)
	track.set_corner_radius_all(5)
	track.content_margin_top = 5
	track.content_margin_bottom = 5
	var fill := track.duplicate() as StyleBoxFlat
	fill.bg_color = Color(0.35, 0.6, 0.95)
	var th := Theme.new()
	th.set_icon("grabber", "HSlider", tex)
	th.set_icon("grabber_highlight", "HSlider", tex)
	th.set_stylebox("slider", "HSlider", track)
	th.set_stylebox("grabber_area", "HSlider", fill)
	th.set_stylebox("grabber_area_highlight", "HSlider", fill)
	return th


func _show_page(i: int) -> void:
	for k in _pages.size():
		_pages[k].visible = k == i
	_scroll.scroll_vertical = 0


## Open a tab programmatically (click on a part): the tab button toggles too.
func _open_page(i: int) -> void:
	_tab_buttons[i].button_pressed = true
	_show_page(i)


# ------------------------------------------------------------------ click-to-pick a part

func _unhandled_input(e: InputEvent) -> void:
	# A click = a short touch/press with no movement and no second finger; otherwise it is a camera rotation/pinch.
	if e is InputEventScreenTouch:
		if e.pressed:
			var now := Time.get_ticks_msec()
			for k in _touch_ids.keys():            # forget “stuck” touches (released over the panel)
				if now - _touch_ids[k] > 2000:
					_touch_ids.erase(k)
			_touch_ids[e.index] = now
			if _touch_ids.size() == 1:
				_tap_begin(e.position)
			else:
				_tap_ok = false
		else:
			_touch_ids.erase(e.index)
			if _touch_ids.is_empty() and _tap_ok:
				_tap_end(e.position)
	elif e is InputEventScreenDrag:
		_tap_move(e.position)
	elif e is InputEventMouseButton and e.button_index == MOUSE_BUTTON_LEFT \
			and e.device != InputEvent.DEVICE_ID_EMULATION:   # mouse emulation from touch — don't duplicate
		if e.pressed:
			_tap_begin(e.position)
		elif _tap_ok:
			_tap_end(e.position)
	elif e is InputEventMouseMotion and e.device != InputEvent.DEVICE_ID_EMULATION:
		_tap_move(e.position)


func _tap_begin(p: Vector2) -> void:
	_tap_start = p
	_tap_msec = Time.get_ticks_msec()
	_tap_ok = true


func _tap_move(p: Vector2) -> void:
	if _tap_ok and p.distance_to(_tap_start) > TAP_SLOP:
		_tap_ok = false


func _tap_end(p: Vector2) -> void:
	_tap_ok = false
	if p.distance_to(_tap_start) > TAP_SLOP or Time.get_ticks_msec() - _tap_msec > TAP_MAX_MSEC:
		return
	var part := _pick_part(cam.project_ray_origin(p), cam.project_ray_normal(p))
	if part < 0:
		_clear_selection()
	else:
		_select_part(part)


## Body part under the ray. Small movable parts (jaw, eyebrows, fingers) sit right against the fixed
## shell and often “sink” into it by 1–2 cm — so among hits within PICK_TOL of the nearest
## hit, the link driven by its own joint is preferred over the parent shell.
func _pick_part(origin: Vector3, dir: Vector3) -> int:
	var hits := robot.pick_links(origin, dir)
	if hits.is_empty():
		return -1
	var nearest: float = hits[0][0]
	for h: Array in hits:
		if h[0] > nearest + PICK_TOL:
			break
		var jn := robot.leader_of(robot.joint_of_link(h[1]))
		if jn != "" and robot.controllable.has(jn):
			var p := _part_of_joint(jn)
			if p >= 0:
				return p
	return _part_for_link(hits[0][1])


func _part_of_joint(jn: String) -> int:
	for i in PARTS.size():
		for pat: String in PARTS[i][1]:
			if jn.contains(pat):
				return i
	return -1


## Link → body part. Fixed links (mounts, camera) return their parent's part.
func _part_for_link(link_name: String) -> int:
	var l := link_name
	while l != "":
		var jn := robot.joint_of_link(l)
		if jn == "":
			return -1
		var lead := robot.leader_of(jn)            # finger: Mid/Tip → leading Phal
		if robot.controllable.has(lead):
			var p := _part_of_joint(lead)
			if p >= 0:
				return p
		l = robot.parent_link_of_joint(jn)
	return -1


func _select_part(part: int) -> void:
	var jns: Array = []
	for jn: String in robot.controllable:
		if _part_of_joint(jn) == part:
			jns.append(jn)
	if jns.is_empty():
		_clear_selection()
		return
	_selected_joints = jns
	var links: Array = []
	for jn: String in jns:
		for f: String in robot.joint_family(jn):
			links.append(robot.joints[f].child)
	robot.set_selected_links(links)
	_set_rows_selected(jns)
	_sel_label.text = "Selected: %s — %d DOF" % [PARTS[part][0], jns.size()]
	_sel_label.visible = true
	_open_page(_joint_page[jns[0]])
	await get_tree().process_frame             # the tab layout will update — then scroll to the slider
	await get_tree().process_frame
	if _selected_joints == jns:
		_scroll.ensure_control_visible(_rows[jns[0]])


func _clear_selection() -> void:
	if _selected_joints.is_empty() and not _sel_label.visible:
		return
	_selected_joints = []
	_set_rows_selected([])
	robot.set_selected_links([])
	_sel_label.visible = false


func _set_rows_selected(jns: Array) -> void:
	for jn in _rows:
		(_rows[jn] as PanelContainer).add_theme_stylebox_override("panel", _row_style(jns.has(jn)))


# ------------------------------------------------------------------ “Gestures” tab

static func _is_face(jn: String) -> bool:
	for p: String in FACE_PATTERNS:
		if jn.contains(p):
			return true
	return false


func _build_gestures_page() -> Control:
	var page := VBoxContainer.new()
	page.add_theme_constant_override("separation", 6)
	var help := Label.new()
	help.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	help.add_theme_font_size_override("font_size", 13)
	help.text = "Only joints you moved since “Reset” or the previous save go into a gesture. " \
		+ "Body and face are separate files; the whole file is sent to the server: body — %s, face — %s." \
		% [RobotLink.T_GESTURE_BODY, RobotLink.T_GESTURE_FACE]
	page.add_child(help)
	_gest_name = LineEdit.new()
	_gest_name.placeholder_text = "Gesture name, e.g. wave"
	_gest_name.custom_minimum_size = Vector2(0, 48)
	page.add_child(_gest_name)
	var r := HBoxContainer.new()
	page.add_child(r)
	var b1 := _button("Save body", _save_gesture.bind("body"))
	b1.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	r.add_child(b1)
	var b2 := _button("Save face", _save_gesture.bind("face"))
	b2.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	r.add_child(b2)
	_gest_touched = Label.new()
	_gest_touched.add_theme_font_size_override("font_size", 13)
	_gest_touched.modulate = Color(0.5, 0.75, 1.0)
	page.add_child(_gest_touched)
	_gest_status = Label.new()
	_gest_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_gest_status.add_theme_font_size_override("font_size", 13)
	page.add_child(_gest_status)
	for kind in ["body", "face"]:
		var t := Label.new()
		t.text = "Body (BODY_JOINTS)" if kind == "body" else "Face (FACE_JOINTS)"
		t.modulate = Color(0.7, 0.8, 1.0)
		page.add_child(t)
		var box := VBoxContainer.new()
		box.add_theme_constant_override("separation", 6)
		page.add_child(box)
		_gest_lists[kind] = box
	return page


func _mark_touched(jns: Array) -> void:
	for jn in jns:
		if robot.controllable.has(jn):
			_touched[jn] = true
	_update_touched_label()


func _update_touched_label() -> void:
	if _gest_touched == null:
		return
	var nb := 0
	var nf := 0
	for jn in _touched:
		if _is_face(jn):
			nf += 1
		else:
			nb += 1
	_gest_touched.text = "Changed since the last reset/save: body — %d, face — %d" % [nb, nf]


func _kind_joints(kind: String) -> Array:
	var out: Array = []
	for jn: String in robot.controllable:
		if _is_face(jn) == (kind == "face"):
			out.append(jn)
	return out


static func _safe_name(n: String) -> String:
	return n.strip_edges().replace(" ", "_").validate_filename()


func _gesture_path(kind: String, safe: String) -> String:
	return "%s/%s/%s.json" % [GESTURE_DIR, kind, safe]


## Build a gesture from the model's current pose. positions — URDF angles (rad), as in /urdf_joint_cmd;
## servo_deg — the same poses in servo degrees per the robot's table (if already connected and the table was received).
func _make_gesture(kind: String, gname: String) -> Dictionary:
	var pos := {}
	var servo := {}
	for jn: String in _kind_joints(kind):
		if not _touched.has(jn):
			continue
		var a: float = robot.positions[jn]
		pos[jn] = snappedf(a, 0.0001)
		if link.joint_map.has(jn):
			var j: Dictionary = link.joint_map[jn]
			servo[j.servo] = snappedf(clampf(link.urdf_to_servo(jn, a), j.servo_range[0], j.servo_range[1]), 0.1)
	var d := {
		"format": "inmoov_gesture", "version": 1, "kind": kind, "name": gname,
		"robot": "inmoov_i2", "units": "urdf_rad",
		"created": Time.get_datetime_string_from_system(true) + "Z",
		"positions": pos,
	}
	if not servo.is_empty():
		d["servo_deg"] = servo
	return d


func _save_gesture(kind: String) -> void:
	var gname := _gest_name.text.strip_edges()
	var safe := _safe_name(gname)
	if safe == "":
		_gest_status.text = "Enter a gesture name"
		return
	var d := _make_gesture(kind, gname)
	if (d.positions as Dictionary).is_empty():
		_gest_status.text = "Nothing to save: you haven't moved any %s joints" % ("face" if kind == "face" else "body")
		return
	DirAccess.make_dir_recursive_absolute("%s/%s" % [GESTURE_DIR, kind])
	var path := _gesture_path(kind, safe)
	var existed := FileAccess.file_exists(path)
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		_gest_status.text = "Could not write the file (error %d)" % FileAccess.get_open_error()
		return
	f.store_string(JSON.stringify(d, "\t"))
	f.close()
	for k in d.positions:           # memory is cleared: saved joints are no longer “touched”
		_touched.erase(k)
	_update_touched_label()
	_gest_status.text = "%s: “%s” %s (%d joints)" % ["Body" if kind == "body" else "Face", gname,
		"overwritten" if existed else "saved", (d.positions as Dictionary).size()]
	_refresh_gesture_lists()


func _refresh_gesture_lists() -> void:
	for kind in _gest_lists:
		var box: VBoxContainer = _gest_lists[kind]
		for c in box.get_children():
			c.queue_free()
		var names: Array = []
		var da := DirAccess.open("%s/%s" % [GESTURE_DIR, kind])
		if da != null:
			for fn in da.get_files():
				if fn.ends_with(".json"):
					names.append(fn.get_basename())
		names.sort()
		if names.is_empty():
			var e := Label.new()
			e.text = "— none saved yet"
			e.modulate = Color(1, 1, 1, 0.5)
			box.add_child(e)
		for n: String in names:
			box.add_child(_gesture_item(kind, n))


func _gesture_item(kind: String, safe: String) -> Control:
	var item := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(1, 1, 1, 0.06)
	sb.set_corner_radius_all(8)
	sb.set_content_margin_all(6)
	item.add_theme_stylebox_override("panel", sb)
	var v := VBoxContainer.new()
	item.add_child(v)
	var l := Label.new()
	l.text = safe
	v.add_child(l)
	var h := HBoxContainer.new()
	v.add_child(h)
	for spec in [["Apply", _apply_gesture.bind(kind, safe)], ["To server", _send_gesture.bind(kind, safe)],
			["JSON", _copy_gesture.bind(kind, safe)]]:
		var b := _button(spec[0], spec[1])
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		h.add_child(b)
	var del := Button.new()
	del.text = "✕"
	del.custom_minimum_size = Vector2(64, 48)
	del.pressed.connect(_delete_gesture.bind(kind, safe, del))
	h.add_child(del)
	return item


func _read_gesture(kind: String, safe: String) -> String:
	return FileAccess.get_file_as_string(_gesture_path(kind, safe))


func _apply_gesture(kind: String, safe: String) -> void:
	var d = JSON.parse_string(_read_gesture(kind, safe))
	if typeof(d) != TYPE_DICTIONARY or typeof(d.get("positions")) != TYPE_DICTIONARY:
		_gest_status.text = "File “%s” is corrupted" % safe
		return
	if link.mode == RobotLink.Mode.MIRROR and link.is_connected_to_robot():
		_gest_status.text = "“Mirror” mode: the model follows the robot. Switch to “Control”."
		return
	var p := {}
	for k in d.positions:
		if robot.joints.has(k):
			p[k] = float(d.positions[k])
	var to_robot := link.mode == RobotLink.Mode.CONTROL and link.is_connected_to_robot()
	if _block.button_pressed or to_robot:
		var pairs = await robot.check_pose(p)
		if not pairs.is_empty():
			_gest_status.text = "Gesture rejected: %d collisions" % pairs.size()
			return
	robot.set_pose(p)
	_last_ok = robot.get_pose()
	_sync_sliders()
	_mark_touched(p.keys())
	if to_robot:
		link.send_pose(p)
	_gest_status.text = "Applied: %s" % safe


func _send_gesture(kind: String, safe: String) -> void:
	var txt := _read_gesture(kind, safe)
	if txt == "":
		_gest_status.text = "File “%s” not found" % safe
		return
	var topic := RobotLink.T_GESTURE_FACE if kind == "face" else RobotLink.T_GESTURE_BODY
	if link.send_gesture_file(kind, txt):
		_gest_status.text = "Sent to %s: %s (%d bytes)" % [topic, safe, txt.length()]
	else:
		_gest_status.text = "No link to the robot — connect on the “Robot” tab"


func _copy_gesture(kind: String, safe: String) -> void:
	DisplayServer.clipboard_set(_read_gesture(kind, safe))
	_gest_status.text = "JSON “%s” copied to clipboard" % safe


## Two-tap delete: the first tap changes the button to “Delete?”, the second (within 2 s) deletes.
func _delete_gesture(kind: String, safe: String, btn: Button) -> void:
	if btn.text == "✕":
		btn.text = "Delete?"
		await get_tree().create_timer(2.0).timeout
		if is_instance_valid(btn):
			btn.text = "✕"
		return
	DirAccess.remove_absolute(ProjectSettings.globalize_path(_gesture_path(kind, safe)))
	_gest_status.text = "Deleted: %s" % safe
	_refresh_gesture_lists()


# ------------------------------------------------------------------ “Robot” tab

func _build_robot_page(th: Theme) -> Control:
	var page := VBoxContainer.new()
	page.add_theme_constant_override("separation", 6)

	var r1 := HBoxContainer.new()
	page.add_child(r1)
	_url = LineEdit.new()
	_url.placeholder_text = "Robot IP, e.g. 192.168.1.50"
	_url.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_url.custom_minimum_size = Vector2(0, 48)
	var cfg := ConfigFile.new()
	if cfg.load(SETTINGS) == OK:
		_url.text = cfg.get_value("link", "url", "")
	r1.add_child(_url)
	_conn_btn = _button("Connect", _on_connect_pressed)
	r1.add_child(_conn_btn)

	_link_status = Label.new()
	_link_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_link_status.text = "Not connected"
	page.add_child(_link_status)
	_fresh = Label.new()
	page.add_child(_fresh)

	var modes := HBoxContainer.new()
	page.add_child(modes)
	var g := ButtonGroup.new()
	for m in [["Mirror", RobotLink.Mode.MIRROR], ["Control", RobotLink.Mode.CONTROL],
			["Calibration", RobotLink.Mode.CALIBRATE]]:
		var b := Button.new()
		b.text = m[0]
		b.toggle_mode = true
		b.button_group = g
		b.custom_minimum_size = Vector2(0, 48)
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		b.pressed.connect(_set_mode.bind(m[1]))
		modes.add_child(b)
		_mode_btns.append(b)
	_mode_btns[0].button_pressed = true
	_release_btn = _button("Release joints (give control back to the robot)", func(): link.release_all())
	page.add_child(_release_btn)

	var help := Label.new()
	_help = help
	help.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	help.add_theme_font_size_override("font_size", 13)
	help.text = "Mirror — the model follows the robot.  Control — sliders move the robot " \
		+ "(poses with self-collisions are not sent).  Calibration — tuning the servo↔URDF table."
	page.add_child(help)

	# --- calibration
	_calib_box = VBoxContainer.new()
	_calib_box.visible = false
	page.add_child(_calib_box)
	_calib_joint = OptionButton.new()
	_calib_joint.custom_minimum_size = Vector2(0, 44)
	_calib_joint.item_selected.connect(_on_calib_joint)
	_calib_box.add_child(_calib_joint)
	_calib_info = Label.new()
	_calib_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_calib_info.add_theme_font_size_override("font_size", 13)
	_calib_box.add_child(_calib_info)

	var rs := HBoxContainer.new()
	_calib_box.add_child(rs)
	_calib_servo_lab = Label.new(); _calib_servo_lab.custom_minimum_size = Vector2(120, 0)
	rs.add_child(_calib_servo_lab)
	_calib_servo = _make_slider(th, 0, 180, 1)
	_calib_servo.value_changed.connect(_on_calib_servo)
	rs.add_child(_calib_servo)
	var rm := HBoxContainer.new()
	_calib_box.add_child(rm)
	_calib_model_lab = Label.new(); _calib_model_lab.custom_minimum_size = Vector2(120, 0)
	rm.add_child(_calib_model_lab)
	_calib_model = _make_slider(th, -90, 90, 0.5)
	_calib_model.value_changed.connect(_on_calib_model)
	rm.add_child(_calib_model)

	var rb := HBoxContainer.new()
	_calib_box.add_child(rb)
	rb.add_child(_button("Point", _calib_add_point))
	rb.add_child(_button("Clear", func(): _calib_points.clear(); _calib_refresh_info()))
	rb.add_child(_button("Save", _calib_save))
	rb.add_child(_button("Factory", _calib_reset))
	return page


func _on_connect_pressed() -> void:
	if _conn_btn.text == "Disconnect":
		link.disconnect_from_robot()
		camera_view.disconnect_stream()
		_conn_btn.text = "Connect"
		_update_controls()
		return
	var cfg := ConfigFile.new()
	cfg.set_value("link", "url", _url.text.strip_edges())
	cfg.save(SETTINGS)
	link.connect_to(_url.text)
	if _cam_toggle.button_pressed:
		camera_view.connect_to(_url.text)
	_conn_btn.text = "Disconnect"


## A hidden camera window uses no traffic: the stream is switched off and on together with the window.
func _on_cam_toggled(on: bool) -> void:
	camera_view.visible = on
	if on and _conn_btn.text == "Disconnect":
		camera_view.connect_to(_url.text)
	elif not on:
		camera_view.disconnect_stream()


func _set_mode(m: RobotLink.Mode) -> void:
	link.set_mode(m)
	_calib_box.visible = m == RobotLink.Mode.CALIBRATE
	_help.visible = not _calib_box.visible          # save screen height
	_release_btn.visible = not _calib_box.visible
	if m == RobotLink.Mode.CONTROL:
		_block.button_pressed = true    # to the real robot — only collision-free poses
		_last_ok = robot.get_pose()
	if m == RobotLink.Mode.CALIBRATE:
		_on_calib_joint(_calib_joint.selected)
	_sync_sliders()
	_update_controls()


## In “Mirror” mode, once connected, the sliders only show the robot's pose.
func _update_controls() -> void:
	var mirror := link.mode == RobotLink.Mode.MIRROR and link.is_connected_to_robot()
	var ctrl := link.mode == RobotLink.Mode.CONTROL and link.is_connected_to_robot()
	for jn in _sliders:
		(_sliders[jn] as HSlider).editable = not mirror
		(_sliders[jn] as HSlider).modulate = Color(1, 1, 1, 0.55 if mirror else 1.0)
	_block.disabled = ctrl
	_random_btn.disabled = ctrl or mirror


func _on_real_pose(_p: Dictionary) -> void:
	if link.mode == RobotLink.Mode.MIRROR:
		_sync_sliders()


func _on_map(jm: Dictionary) -> void:
	var prev := _calib_joint.get_item_text(_calib_joint.selected) if _calib_joint.selected >= 0 else ""
	_calib_joint.clear()
	var names := jm.keys()
	names.sort()
	for k in names:
		_calib_joint.add_item(k)
	var idx := names.find(prev)
	_calib_joint.select(maxi(idx, 0))
	if link.mode == RobotLink.Mode.CALIBRATE:
		_on_calib_joint(_calib_joint.selected)


# ------------------------------------------------------------------ calibration

func _calib_urdf() -> String:
	return _calib_joint.get_item_text(_calib_joint.selected) if _calib_joint.selected >= 0 else ""


func _on_calib_joint(_i: int) -> void:
	var jn := _calib_urdf()
	if jn == "" or not link.joint_map.has(jn):
		_calib_info.text = "No table — connect to the robot"
		return
	link.release_servo()
	var j: Dictionary = link.joint_map[jn]
	_calib_points.clear()
	_calib_lock = true
	_calib_servo.min_value = j.servo_range[0]
	_calib_servo.max_value = j.servo_range[1]
	var lim := robot.get_limits(jn)
	_calib_model.min_value = rad_to_deg(lim.x)
	_calib_model.max_value = rad_to_deg(lim.y)
	var cur: float = link.current_servo_deg(jn, j.servo_rest)
	_calib_servo.value = cur
	_calib_model.value = rad_to_deg(link.servo_to_urdf(jn, cur))
	_calib_lock = false
	robot.set_joint(jn, deg_to_rad(_calib_model.value))
	_calib_labels()
	_calib_refresh_info()


func _on_calib_servo(v: float) -> void:
	_calib_labels()
	if _calib_lock:
		return
	var jn := _calib_urdf()
	if jn == "" or not link.joint_map.has(jn):
		return
	link.send_servo(link.joint_map[jn].servo, v)
	# model — per the current table; then the user adjusts it with the “Model” slider
	_calib_lock = true
	_calib_model.value = rad_to_deg(link.servo_to_urdf(jn, v))
	_calib_lock = false
	robot.set_joint(jn, deg_to_rad(_calib_model.value))


func _on_calib_model(v: float) -> void:
	_calib_labels()
	if _calib_lock:
		return
	var jn := _calib_urdf()
	if jn != "":
		robot.set_joint(jn, deg_to_rad(v))


func _calib_labels() -> void:
	_calib_servo_lab.text = "Servo %d°" % int(_calib_servo.value)
	_calib_model_lab.text = "Model %.1f°" % _calib_model.value


func _calib_add_point() -> void:
	var s := _calib_servo.value
	for p in _calib_points:
		if absf(p[0] - s) < 0.5:
			_calib_points.erase(p)
			break
	_calib_points.append([s, deg_to_rad(_calib_model.value)])
	_calib_points.sort_custom(func(a, b): return a[0] < b[0])
	_calib_refresh_info()


## Points must be monotonic: as the servo angle increases, the model angle only increases (or only decreases).
func _calib_points_ok() -> String:
	if _calib_points.size() < 2:
		return "at least 2 points needed"
	var sgn := 0
	for i in _calib_points.size() - 1:
		var d: float = _calib_points[i + 1][1] - _calib_points[i][1]
		if absf(d) < 1e-6:
			return "two points with the same model angle"
		var s := 1 if d > 0 else -1
		if sgn != 0 and s != sgn:
			return "model angles are not monotonic"
		sgn = s
	return ""


func _calib_save() -> void:
	var err := _calib_points_ok()
	if err != "":
		_calib_info.text = "Not saved: " + err
		return
	link.save_calibration(link.joint_map[_calib_urdf()].servo, _calib_points)
	_calib_info.text = "Sent to the robot…"


func _calib_reset() -> void:
	var jn := _calib_urdf()
	if jn != "":
		link.save_calibration(link.joint_map[jn].servo, [], true)


func _calib_refresh_info() -> void:
	var jn := _calib_urdf()
	if jn == "" or not link.joint_map.has(jn):
		return
	var j: Dictionary = link.joint_map[jn]
	var t := "Servo: %s (%d–%d°, rest %d°)%s\nRobot table: " % [j.servo, j.servo_range[0],
		j.servo_range[1], j.servo_rest, "  ✔ calibrated" if j.get("verified", false) else ""]
	for p in j.points:
		t += "%d°→%.1f°  " % [p[0], rad_to_deg(p[1])]
	t += "\nNew points: "
	for p in _calib_points:
		t += "%d°→%.1f°  " % [p[0], rad_to_deg(p[1])]
	if _calib_points.is_empty():
		t += "— (move “Servo”, fit “Model” to the real robot, then “Point”)"
	_calib_info.text = t


# ------------------------------------------------------------------ joint sliders

func _on_slider(v: float, jn: String) -> void:
	if _lock:
		return
	robot.set_joint(jn, deg_to_rad(v))
	var to_robot := link.mode == RobotLink.Mode.CONTROL and link.is_connected_to_robot()
	if _block.button_pressed or to_robot:
		await get_tree().physics_frame
		await get_tree().physics_frame
		if robot.compute_colliding_pairs().is_empty():
			_last_ok = robot.get_pose()
			_mark_touched([jn])
			if to_robot:
				link.send_pose({jn: robot.positions[jn]})
		else:
			robot.set_pose(_last_ok)   # roll back to the last valid pose
			_sync_sliders()
	else:
		_mark_touched([jn])


func _reset_pose() -> void:
	var p := {}
	for jn in robot.controllable:
		p[jn] = 0.0
	if link.mode == RobotLink.Mode.CONTROL and link.is_connected_to_robot():
		var pairs = await robot.check_pose(p)
		if not pairs.is_empty():
			_status.text = "Reset rejected: %d collisions" % pairs.size()
			return
		robot.set_pose(p)
		link.send_pose(p)
	else:
		robot.set_pose(p)
	_last_ok = robot.get_pose()
	_sync_sliders()
	_touched.clear()            # after “Reset” the gesture is collected anew
	_update_touched_label()


func _random_pose() -> void:
	var p := {}
	for jn in robot.controllable:
		var lim := robot.get_limits(jn)
		p[jn] = randf_range(lim.x, lim.y)
	if _block.button_pressed:
		var pairs = await robot.check_pose(p)
		if not pairs.is_empty():
			_status.text = "Random pose rejected: %d collisions" % pairs.size()
			return
	robot.set_pose(p)
	_last_ok = robot.get_pose()
	_sync_sliders()
	_mark_touched(p.keys())


func _sync_sliders() -> void:
	_lock = true
	for jn in _sliders:
		_sliders[jn].value = rad_to_deg(robot.positions[jn])
	_lock = false


func _on_collision(pairs: Array) -> void:
	if pairs.is_empty():
		_status.text = "No self-collisions"
	else:
		var t := "Collisions (%d):" % pairs.size()
		for p in pairs.slice(0, 6):
			t += "\n  %s ↔ %s" % [p[0], p[1]]
		_status.text = t


var _ui_acc := 0.0
var _conn_acc := 0.0


## Link status next to FPS: green — link up and poses arriving, yellow — connecting / no data, gray — no link.
func _update_conn_label() -> void:
	if link.is_connected_to_robot():
		if link.is_state_fresh():
			_conn_lab.text = "● Robot"
			_conn_lab.modulate = Color(0.35, 0.85, 0.4)
		else:
			_conn_lab.text = "● No poses"
			_conn_lab.modulate = Color(0.95, 0.7, 0.25)
	elif link.ros.is_trying():
		_conn_lab.text = "◌ Connecting…"
		_conn_lab.modulate = Color(0.95, 0.7, 0.25)
	else:
		_conn_lab.text = "○ No link"
		_conn_lab.modulate = Color(0.6, 0.6, 0.62)

func _process(dt: float) -> void:
	_fps.text = "FPS: %d" % Engine.get_frames_per_second()
	_ui_acc += dt
	_conn_acc += dt
	if _conn_acc > 0.25:
		_conn_acc = 0.0
		_update_conn_label()
	if _ui_acc > 0.5:
		_ui_acc = 0.0
		if not link.is_connected_to_robot():
			_fresh.text = ""
		elif link.is_state_fresh():
			_fresh.text = "● robot pose is updating"
			_fresh.modulate = Color(0.3, 0.8, 0.3)
		else:
			_fresh.text = "● no /urdf_joint_states (urdf_bridge not running?)"
			_fresh.modulate = Color(0.9, 0.4, 0.2)
