## Orbit camera: mouse (LMB — rotate, RMB/MMB — pan, wheel — zoom)
## and touch (1 finger — rotate, 2 fingers — pinch zoom + pan).
extends Camera3D

@export var target := Vector3(0, 1.45, 0)
@export var distance := 2.6
@export var yaw := 0.0
@export var pitch := -0.08
@export var min_distance := 0.3
@export var max_distance := 6.0
@export var rotate_speed := 0.006

## Robot width (with arms stretched out to the sides), m — used to pick the camera distance.
@export var span_m := 1.8
## Area occupied on the left (control panel), viewport px: the robot is centered in the free part on the right.
var left_inset_px := 0.0

var _touches := {}
var _auto_fit := true     ## until the user changes zoom — the distance is fitted to the window size


func _ready() -> void:
	get_viewport().size_changed.connect(_on_viewport_resized)
	_update()


func set_left_inset(px: float) -> void:
	left_inset_px = px
	_on_viewport_resized()


func _on_viewport_resized() -> void:
	if _auto_fit:
		var vp := get_viewport().get_visible_rect().size
		var free_w := maxf(vp.x - left_inset_px, 200.0)
		var k := 2.0 * tan(deg_to_rad(fov) * 0.5)          # frame height per unit of distance
		distance = clampf(maxf(span_m * vp.y / (free_w * k), 2.4), min_distance, max_distance)
	_update()


func _unhandled_input(e: InputEvent) -> void:
	if e is InputEventScreenTouch:
		if e.pressed:
			_touches[e.index] = e.position
		else:
			_touches.erase(e.index)
	elif e is InputEventScreenDrag:
		if _touches.size() >= 2:
			var old: Vector2 = _touches[e.index]
			var other_idx = _touches.keys().filter(func(k): return k != e.index)[0]
			var o: Vector2 = _touches[other_idx]
			var d0 := old.distance_to(o)
			var d1 := (e.position as Vector2).distance_to(o)
			if d1 > 1.0:
				_auto_fit = false
				distance = clampf(distance * d0 / d1, min_distance, max_distance)
			_pan(e.relative * 0.5)
		_touches[e.index] = e.position
		_update()
	elif e is InputEventMouseMotion and _touches.size() < 2:
		if e.button_mask & MOUSE_BUTTON_MASK_LEFT:
			yaw -= e.relative.x * rotate_speed
			pitch = clampf(pitch - e.relative.y * rotate_speed, -1.45, 1.45)
			_update()
		elif e.button_mask & (MOUSE_BUTTON_MASK_RIGHT | MOUSE_BUTTON_MASK_MIDDLE):
			_pan(e.relative)
			_update()
	elif e is InputEventMouseButton and e.pressed:
		if e.button_index == MOUSE_BUTTON_WHEEL_UP:
			_auto_fit = false
			distance = maxf(min_distance, distance * 0.9)
		elif e.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_auto_fit = false
			distance = minf(max_distance, distance * 1.1)
		_update()
	elif e is InputEventMagnifyGesture:
		_auto_fit = false
		distance = clampf(distance / e.factor, min_distance, max_distance)
		_update()


func _pan(rel: Vector2) -> void:
	var k := distance * 0.0015
	target += (-global_basis.x * rel.x + global_basis.y * rel.y) * k


func focus(point: Vector3, dist := -1.0) -> void:
	target = point
	if dist > 0:
		distance = dist
	_update()


func _update() -> void:
	var dir := Vector3(sin(yaw) * cos(pitch), -sin(pitch), cos(yaw) * cos(pitch))
	global_position = target + dir * distance
	look_at(target, Vector3.UP)
	# shift the image right by left_inset/2 px: the camera moves left by the same amount
	var vp_h := get_viewport().get_visible_rect().size.y
	if vp_h > 0.0 and left_inset_px > 0.0:
		var mpp := 2.0 * distance * tan(deg_to_rad(fov) * 0.5) / vp_h
		global_position -= global_basis.x * (left_inset_px * 0.5 * mpp)
