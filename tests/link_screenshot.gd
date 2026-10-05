# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## Screenshot of the “Robot” tab in calibration mode (requires the mock on 9090):
## xvfb-run godot --path . --rendering-method gl_compatibility -s res://tests/link_screenshot.gd
extends SceneTree

func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 10: await process_frame
	main._url.text = "127.0.0.1"
	main._on_connect_pressed()
	await create_timer(1.5).timeout
	main._show_page(main._pages.size() - 1)
	main._mode_btns[2].button_pressed = true
	main._set_mode(RobotLink.Mode.CALIBRATE)
	for i in main._calib_joint.item_count:
		if main._calib_joint.get_item_text(i) == "left_elbow_x_joint":
			main._calib_joint.select(i); main._on_calib_joint(i)
	main._calib_servo.value = 40
	main._calib_add_point()
	await create_timer(1.0).timeout
	root.get_viewport().get_texture().get_image().save_png(OS.get_environment("SHOT_OUT"))
	quit()
