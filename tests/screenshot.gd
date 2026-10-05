# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## xvfb-run godot --path . --rendering-method gl_compatibility -s res://tests/screenshot.gd
extends SceneTree

func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 10: await process_frame
	var robot: InmoovRobot = main.get_node("Robot")
	robot.set_pose({"torso_y_joint": 0.26, "left_shoulder_x_joint": 1.2, "left_elbow_x_joint": 0.8, "right_shoulder_y_joint": 0.5})
	for jn in robot.controllable:
		if jn.begins_with("i02_rightHand"):
			robot.set_joint(jn, robot.get_limits(jn).y)
	for i in 20: await process_frame
	root.get_viewport().get_texture().get_image().save_png(OS.get_environment("SHOT_OUT"))
	quit()
