## Link between the 3D model and the real robot via rosbridge + urdf_bridge (inmoov_control).
##
## All angles use the URDF convention: the “servo degrees ↔ URDF radians” conversion is done by
## urdf_bridge on the robot using the servo_urdf_map.yaml table.
##
## Modes:
##   MIRROR   — the model follows the real pose (/urdf_joint_states, the pose after
##              command arbitration — the servos have no feedback).
##   CONTROL  — model changes are sent to the robot (/urdf_joint_cmd, source "remote",
##              priority 80, lease). Only joints touched by
##              the user are sent; the lease is renewed while the mode is on; on exit — release.
##   CALIBRATE— the model does not follow the robot; servo commands go straight to /joint_cmd
##              (source "calibration", priority 90) — see send_servo().
class_name RobotLink
extends Node

signal link_status_changed(text: String)
signal real_pose_received(pose: Dictionary)
signal map_received(joints: Dictionary)        ## urdf -> description (servo, points, ...)
signal bridge_status_received(status: Dictionary)

enum Mode { MIRROR, CONTROL, CALIBRATE }

const T_STATES := "/urdf_joint_states"
const T_URDF_CMD := "/urdf_joint_cmd"
const T_SERVO_CMD := "/joint_cmd"
const T_CALIB := "/urdf_bridge/set_calibration"
const T_STATUS := "/urdf_bridge/status"
const S_GET_MAP := "/urdf_bridge/get_map"
## Gestures: the whole JSON file (std_msgs/String.data). Body and face are separate topics.
## The server side (receive/save/play back) lives on the robot; for the format see GESTURES.md.
const T_GESTURE_BODY := "/gesture/body_joints"
const T_GESTURE_FACE := "/gesture/face_joints"
const JS_TYPE := "sensor_msgs/msg/JointState"
const JC_TYPE := "inmoov_msgs/msg/JointCommand"
const STR_TYPE := "std_msgs/msg/String"

const PRIORITY_REMOTE := 80
const PRIORITY_CALIBRATION := 90

@export var robot: InmoovRobot
@export var send_rate_hz := 20.0
@export var lease_sec := 3.0
@export var state_throttle_ms := 40

var ros: RosbridgeClient
var mode := Mode.MIRROR
var real_pose := {}          ## last real pose (URDF, rad)
var joint_map := {}          ## urdf -> {servo, points, servo_range, verified, ...}
var bridge_status := {}
var last_state_time := -1.0  ## Time.get_ticks_msec()/1000 of the last /urdf_joint_states

var _pending := {}           ## joint -> angle, not yet sent
var _owned := {}             ## joint -> last sent angle (we hold the lease)
var _send_acc := 0.0
var _lease_acc := 0.0
var _map_call := ""
var _calib_servo := ""       ## servo currently driven by calibration
var _calib_acc := 0.0
var _calib_deg := 0.0


func _ready() -> void:
	ros = RosbridgeClient.new()
	ros.name = "Rosbridge"
	add_child(ros)
	ros.connected.connect(_on_connected)
	ros.disconnected.connect(func(): _status("Link lost, reconnecting…"))
	ros.message_received.connect(_on_message)
	ros.service_response.connect(_on_service)
	ros.subscribe(T_STATES, JS_TYPE, state_throttle_ms)
	ros.subscribe(T_STATUS, STR_TYPE)
	ros.advertise(T_URDF_CMD, JC_TYPE)
	ros.advertise(T_SERVO_CMD, JC_TYPE)
	ros.advertise(T_CALIB, STR_TYPE)
	ros.advertise(T_GESTURE_BODY, STR_TYPE)
	ros.advertise(T_GESTURE_FACE, STR_TYPE)


func connect_to(url: String) -> void:
	_status("Connecting to %s…" % url)
	ros.connect_to_url(url)


func disconnect_from_robot() -> void:
	release_all()
	release_servo()
	ros.close()
	_status("Disconnected")


func is_connected_to_robot() -> bool:
	return ros.is_open()


## The real pose is older than 1 s — the robot (or joint_state_publisher) is not responding.
func is_state_fresh() -> bool:
	return last_state_time >= 0.0 and Time.get_ticks_msec() / 1000.0 - last_state_time < 1.0


func set_mode(m: Mode) -> void:
	if m == mode:
		return
	if mode == Mode.CONTROL:
		release_all()
	if mode == Mode.CALIBRATE:
		release_servo()
	mode = m
	if mode == Mode.MIRROR and not real_pose.is_empty():
		robot.set_pose(real_pose)


## CONTROL mode: queue joints for sending (URDF angles, rad).
## Call only for a pose already checked for self-collisions.
func send_pose(pose: Dictionary) -> void:
	if mode != Mode.CONTROL or not ros.is_open():
		return
	for k in pose:
		if joint_map.is_empty() or joint_map.has(k):
			_pending[k] = pose[k]


## Release all joints held by the app (other sources can move them again).
func release_all() -> void:
	if _owned.is_empty() and _pending.is_empty():
		return
	var names: Array = _owned.keys()
	for k in _pending:
		if not names.has(k):
			names.append(k)
	_pending.clear()
	_owned.clear()
	ros.publish(T_URDF_CMD, JC_TYPE, _jc("remote", PRIORITY_REMOTE, 0.0, true, names, []))


func request_map() -> void:
	_map_call = ros.call_service(S_GET_MAP, "std_srvs/srv/Trigger")


# ------------------------------------------------------------- gestures

## Send a saved gesture to the robot: the JSON file content goes as is into std_msgs/String.
## kind: "body" -> T_GESTURE_BODY, "face" -> T_GESTURE_FACE. Returns false if there is no link.
func send_gesture_file(kind: String, json_text: String) -> bool:
	if not ros.is_open():
		return false
	var topic := T_GESTURE_FACE if kind == "face" else T_GESTURE_BODY
	return ros.publish(topic, STR_TYPE, {"data": json_text})


# ------------------------------------------------------------- calibration

## Direct command to a single servo in servo degrees (0..180) — bypassing the table.
## Holds the servo (priority 90) while called; release_servo() releases it.
func send_servo(servo: String, deg: float) -> void:
	if _calib_servo != "" and _calib_servo != servo:
		release_servo()
	_calib_servo = servo
	_calib_deg = deg
	_calib_acc = 0.0
	ros.publish(T_SERVO_CMD, JC_TYPE, _jc("calibration", PRIORITY_CALIBRATION, 2.0, false,
		[servo], [deg_to_rad(deg - 90.0)]))


func release_servo() -> void:
	if _calib_servo == "":
		return
	ros.publish(T_SERVO_CMD, JC_TYPE, _jc("calibration", PRIORITY_CALIBRATION, 0.0, true, [_calib_servo], []))
	_calib_servo = ""


## points: [[servo_deg, urdf_rad], ...] (>= 2). reset=true — restore the factory table.
func save_calibration(servo: String, points: Array, reset := false) -> void:
	var d := {"servo": servo}
	if reset:
		d["reset"] = true
	else:
		d["points"] = points
		d["verified"] = true
	ros.publish(T_CALIB, STR_TYPE, {"data": JSON.stringify(d)})


## Convert servo° -> URDF rad using the robot's table (for the preview in calibration).
func servo_to_urdf(urdf_joint: String, servo_deg: float) -> float:
	var pts: Array = joint_map[urdf_joint].points
	var i := 0
	while i < pts.size() - 2 and servo_deg > pts[i + 1][0]:
		i += 1
	var a: Array = pts[i]
	var b: Array = pts[i + 1]
	return a[1] + (b[1] - a[1]) * (servo_deg - a[0]) / (b[0] - a[0])


## Reverse conversion URDF rad -> servo° using the robot's table.
func urdf_to_servo(urdf_joint: String, rad: float) -> float:
	var pts: Array = (joint_map[urdf_joint].points as Array).duplicate()
	pts.sort_custom(func(a, b): return a[1] < b[1])
	var i := 0
	while i < pts.size() - 2 and rad > pts[i + 1][1]:
		i += 1
	var a: Array = pts[i]
	var b: Array = pts[i + 1]
	return a[0] + (b[0] - a[0]) * (rad - a[1]) / (b[1] - a[1])


## Current servo angle (from the real pose), otherwise fallback.
func current_servo_deg(urdf_joint: String, fallback: float) -> float:
	if real_pose.has(urdf_joint) and joint_map.has(urdf_joint):
		var j: Dictionary = joint_map[urdf_joint]
		return clampf(roundf(urdf_to_servo(urdf_joint, real_pose[urdf_joint])), j.servo_range[0], j.servo_range[1])
	return fallback


# ------------------------------------------------------------------ internal

func _jc(source: String, prio: int, lease: float, release: bool, names: Array, pos: Array) -> Dictionary:
	return {
		"source": source, "priority": prio, "lease_sec": lease, "release": release,
		"cmd": {"header": RosbridgeClient.header(), "name": names, "position": pos,
			"velocity": [], "effort": []},
	}


func _process(dt: float) -> void:
	if not ros.is_open():
		return
	if mode == Mode.CONTROL:
		_send_acc += dt
		_lease_acc += dt
		if _send_acc >= 1.0 / send_rate_hz and not _pending.is_empty():
			_send_acc = 0.0
			_flush(_pending)
			for k in _pending:
				_owned[k] = _pending[k]
			_pending.clear()
		elif _lease_acc >= lease_sec * 0.4 and not _owned.is_empty():
			_flush(_owned)            # renew the lease by repeating the last values
	elif mode == Mode.CALIBRATE and _calib_servo != "":
		_calib_acc += dt
		if _calib_acc >= 0.8:
			send_servo(_calib_servo, _calib_deg)


func _flush(pose: Dictionary) -> void:
	_lease_acc = 0.0
	ros.publish(T_URDF_CMD, JC_TYPE, _jc("remote", PRIORITY_REMOTE, lease_sec, false,
		pose.keys(), pose.values()))


func _on_connected() -> void:
	_status("Connected: %s" % ros.url)
	request_map()


func _on_message(topic: String, msg: Dictionary) -> void:
	match topic:
		T_STATES:
			var names: Array = msg.get("name", [])
			var pos: Array = msg.get("position", [])
			var p := {}
			for i in mini(names.size(), pos.size()):
				p[names[i]] = float(pos[i])
			real_pose = p
			last_state_time = Time.get_ticks_msec() / 1000.0
			if mode == Mode.MIRROR:
				robot.set_pose(p)
			real_pose_received.emit(p)
		T_STATUS:
			var s = JSON.parse_string(str(msg.get("data", "")))
			if typeof(s) == TYPE_DICTIONARY:
				bridge_status = s
				bridge_status_received.emit(s)
				var lc = s.get("last_calibration")
				if typeof(lc) == TYPE_DICTIONARY and lc.get("ok", false):
					request_map()


func _on_service(id: String, ok: bool, values: Variant) -> void:
	if id != _map_call:
		return
	if not ok or typeof(values) != TYPE_DICTIONARY:
		_status("No service %s — is urdf_bridge running? (%s)" % [S_GET_MAP, str(values)])
		return
	var d = JSON.parse_string(str(values.get("message", "")))
	if typeof(d) != TYPE_DICTIONARY:
		return
	joint_map.clear()
	for j: Dictionary in d.get("joints", []):
		joint_map[j.urdf] = j
	map_received.emit(joint_map)
	var n_ok := 0
	for k in joint_map:
		if joint_map[k].get("verified", false):
			n_ok += 1
	_status("Connected: %s — %d joints, calibrated %d" % [ros.url, joint_map.size(), n_ok])


func _status(t: String) -> void:
	link_status_changed.emit(t)
