# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## End-to-end link test with the “robot”: requires a running mock
##   python3 mock_rosbridge_robot.py --port 9090 --demo
## then: godot --headless --path . -s res://tests/link_test.gd [-- ws://127.0.0.1:9090]
extends SceneTree

var fails := 0
var main: Node
var link: RobotLink


func check(cond: bool, msg: String) -> void:
	print(("  OK   " if cond else "  FAIL ") + msg)
	if not cond:
		fails += 1


func wait(sec: float) -> void:
	await create_timer(sec).timeout


func get_map() -> Dictionary:
	var id := link.ros.call_service("/urdf_bridge/get_map", "std_srvs/srv/Trigger")
	while true:
		var r = await link.ros.service_response
		if r[0] == id:
			return JSON.parse_string(r[2].message)
	return {}


func _initialize() -> void:
	var url := "ws://127.0.0.1:9090"
	var ua := OS.get_cmdline_user_args()
	if ua.size() > 0:
		url = ua[0]
	main = (load("res://main.tscn") as PackedScene).instantiate()
	root.add_child(main)
	await wait(0.5)
	link = main.link
	var robot: InmoovRobot = main.robot

	# 1. connection and table
	main._url.text = url
	main._on_connect_pressed()
	var t := 0.0
	while link.joint_map.is_empty() and t < 5.0:
		await wait(0.1); t += 0.1
	check(link.is_connected_to_robot(), "connected to %s" % url)
	check(link.joint_map.size() == 41, "servo↔URDF table received: %d joints" % link.joint_map.size())

	# 2. mirror: mock --demo turns the head
	await wait(1.0)
	check(link.is_state_fresh(), "/urdf_joint_states arrive")
	var a0: float = robot.positions["i01_head_rothead_joint"]
	await wait(1.0)
	var a1: float = robot.positions["i01_head_rothead_joint"]
	check(absf(a1 - a0) > 0.01, "mirror: the model's head follows the robot (%.3f → %.3f)" % [a0, a1])
	check(absf(a1 - link.real_pose["i01_head_rothead_joint"]) < 1e-4, "mirror: model angle == real angle")
	check(not main._sliders["left_elbow_x_joint"].editable, "mirror: sliders are display-only")

	# 3. control: elbow 30° -> bicep_L 30° (direct table: rest 0°)
	main._set_mode(RobotLink.Mode.CONTROL)
	check(main._sliders["left_elbow_x_joint"].editable, "control: sliders active")
	main._sliders["left_elbow_x_joint"].value = 30.0
	await wait(1.5)   # servo moves ~33°/s
	var m := await get_map()
	check(absf(m.servo_deg["bicep_L"] - 30.0) < 1.0, "command arrived: bicep_L = %.1f° (expected 30)" % m.servo_deg["bicep_L"])
	check(absf(rad_to_deg(link.real_pose["left_elbow_x_joint"]) - 30.0) < 1.0, "real pose came back: %.1f°" % rad_to_deg(link.real_pose["left_elbow_x_joint"]))

	# 4. a self-colliding pose is not sent (torso_y 15° -> elbow into torso)
	main._sliders["left_elbow_x_joint"].value = 0.0
	await wait(1.2)
	var before: float = (await get_map()).servo_deg["topstom"]
	main._sliders["torso_y_joint"].value = -15.0
	await wait(1.0)
	var after: float = (await get_map()).servo_deg["topstom"]
	check(absf(robot.positions["torso_y_joint"]) < 1e-6, "the colliding pose was rolled back in the model")
	check(absf(after - before) < 0.5, "and was not sent to the robot (topstom %.1f → %.1f)" % [before, after])

	# 5. lease: after 5 s (> lease 3 s) the joint is still ours — the lease is renewed
	main._sliders["right_elbow_x_joint"].value = 20.0
	await wait(5.0)
	m = await get_map()
	check(absf(m.servo_deg["bicep_R"] - 20.0) < 1.0, "bicep_R holds 20° (lease renewed)")

	# 6. rothead calibration: servo 60° and 120° -> model -25° and +25°
	main._set_mode(RobotLink.Mode.CALIBRATE)
	var idx := -1
	for i in main._calib_joint.item_count:
		if main._calib_joint.get_item_text(i) == "i01_head_rothead_joint":
			idx = i
	main._calib_joint.select(idx)
	main._on_calib_joint(idx)
	main._calib_servo.value = 60
	await wait(1.2)
	m = await get_map()
	check(absf(m.servo_deg["rothead"] - 60.0) < 1.0, "calibration: rothead servo by direct command = %.1f° (demo tracker overridden)" % m.servo_deg["rothead"])
	main._calib_model.value = -25.0
	main._calib_add_point()
	main._calib_servo.value = 120
	main._calib_model.value = 25.0
	main._calib_add_point()
	main._calib_save()
	await wait(1.0)
	var jm: Dictionary = link.joint_map["i01_head_rothead_joint"]
	check(jm.get("verified", false) and absf(jm.points[0][1] - deg_to_rad(-25.0)) < 1e-4,
		"calibration saved and table re-read: %s" % str(jm.points))
	check(absf(link.servo_to_urdf("i01_head_rothead_joint", 90.0)) < 1e-6, "with the new table 90° -> 0°")

	# 7. disconnect
	main._set_mode(RobotLink.Mode.MIRROR)
	main._on_connect_pressed()
	await wait(0.5)
	check(not link.is_connected_to_robot(), "disconnected")

	print("RESULT: %s (%d fails)" % ["PASS" if fails == 0 else "FAIL", fails])
	quit(fails)
