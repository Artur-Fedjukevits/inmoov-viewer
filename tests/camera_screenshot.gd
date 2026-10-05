# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## UI screenshot with the camera window (requires mock_camera_ws.py):
## SHOT_OUT=/tmp/cam.png xvfb-run godot --path . --rendering-method gl_compatibility -s res://tests/camera_screenshot.gd
extends SceneTree

func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 10: await process_frame
	main.camera_view.connect_to("127.0.0.1")
	await create_timer(5.0).timeout
	root.get_viewport().get_texture().get_image().save_png(OS.get_environment("SHOT_OUT"))
	quit()
