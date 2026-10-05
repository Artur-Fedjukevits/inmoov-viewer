# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## UI test: “Robot” tab first, click on a part → tab + slider highlight, gestures (save/read).
## xvfb-run godot --path . --rendering-method gl_compatibility -s res://tests/ui_test.gd
extends SceneTree

var fails := 0

func check(c: bool, msg: String) -> void:
	print(("PASS  " if c else "FAIL  ") + msg)
	if not c:
		fails += 1


func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 12: await process_frame
	var robot: InmoovRobot = main.get_node("Robot")
	var cam: Camera3D = main.get_node("Camera")

	# --- tabs
	var tabs: Array = main._tab_buttons
	check(tabs[0].text == "Robot" and tabs[0].button_pressed, "first tab “Robot”, pressed at startup")
	check(main._pages[0].visible and not main._pages[1].visible, "“Robot” page is open")
	check(tabs[-1].text == "Gestures", "last tab “Gestures”")
	var pr: Rect2 = main._panel.get_global_rect()
	var vp := root.get_visible_rect().size
	check(pr.position.x >= 12.0 and pr.end.x <= vp.x * 0.7 and pr.end.y <= vp.y - 12.0, "panel with margins: %s, window %s" % [pr, vp])

	# --- click on body parts: link mesh center → screen → pick → tab and sliders
	var cases := {
		"left_elbow_x_link": "left_elbow_x_joint",
		"i02_leftHand_indexPhal": "i02_leftHand_indexPhal_joint",
		"i02_rightHand_ThumbMid": "i02_rightHand_ThumbMid_joint",
		"left_shoulder_z_link": "left_shoulder_x_joint",    # shoulder: 3 DOF
		"i02_head_jaw": "i02_head_jaw_joint",
	}
	for l: String in cases:
		var n: Node3D = robot.link_nodes[l]
		var mi: MeshInstance3D = null
		for ch in n.get_children():
			if ch is MeshInstance3D and String(ch.name).begins_with(l + "__"):
				if mi == null or ch.get_aabb().get_volume() > mi.get_aabb().get_volume():
					mi = ch
		if mi == null:
			print("SKIP  no mesh for ", l)
			continue
		# mesh surface point nearest the camera (the AABB center may lie in empty space)
		var wc := Vector3.ZERO
		var bd := INF
		for v: Vector3 in mi.mesh.get_faces():
			var w: Vector3 = mi.global_transform * v
			var dd := w.distance_to(cam.global_position)
			if dd < bd:
				bd = dd
				wc = w
		wc += (cam.global_position - wc).normalized() * 0.002      # 2 mm toward the camera: so the ray doesn't “slip through” a face
		var sp := cam.unproject_position(wc)
		var got := robot.pick_link(cam.project_ray_origin(sp), cam.project_ray_normal(sp))
		var part: int = main._pick_part(cam.project_ray_origin(sp), cam.project_ray_normal(sp))
		var exp: int = main._part_of_joint(cases[l])
		print("  ", l,  " nearest point on screen ", sp, " pick=", got, " part=", main.PARTS[part][0] if part >= 0 else "-")
		check(part == exp, "click on %s → part “%s”" % [l, main.PARTS[exp][0]])
		if part >= 0:
			main._select_part(part)
			for i in 4: await process_frame
			var on := 0
			for jn in main._rows:
				if (main._rows[jn].get_theme_stylebox("panel") as StyleBoxFlat).border_width_left > 0:
					on += 1
			check(on == main._selected_joints.size() and on >= 1, "  sliders highlighted: %d (part DOF = %d)" % [on, main._selected_joints.size()])
			var pg: int = main._joint_page[main._selected_joints[0]]
			check(main._pages[pg].visible and tabs[pg].button_pressed, "  tab “%s” opened" % tabs[pg].text)
	main._select_part(main._part_of_joint("left_shoulder_y_joint"))
	check(main._selected_joints.size() == 3, "shoulder: 3 DOF highlighted")
	main._select_part(main._part_of_joint("i01_head_neck_001_joint"))
	check(main._selected_joints.size() == 3, "neck/head: 3 DOF highlighted")
	main._clear_selection()
	check(main._selected_joints.is_empty(), "click on empty space clears the selection")
	var empty := robot.pick_link(Vector3(0, 5, 5), Vector3(0, 0, -1))
	check(empty == "", "ray missing the robot → nothing")

	# --- gestures: only touched joints go into the file; after save/reset the memory is cleared
	await main._reset_pose()
	check(main._touched.is_empty(), "after “Reset” the memory is empty")
	var g0: Dictionary = main._make_gesture("body", "x")
	check((g0.positions as Dictionary).is_empty(), "nothing moved → positions empty")
	main._gest_name.text = "unit test gesture"
	var safe: String = main._safe_name("unit test gesture")
	main._save_gesture("body")
	check(not FileAccess.file_exists(main._gesture_path("body", safe)), "an empty gesture is not saved")
	# move 2 body joints and 1 face joint with the sliders (like a user)
	main._sliders["left_elbow_x_joint"].value = 30.0
	main._sliders["i02_rightHand_indexPhal_joint"].value = 40.0
	main._sliders["i02_head_jaw_joint"].value = 5.0
	for i in 4: await process_frame
	var tb: Array = main._touched.keys()
	check(tb.size() == 3, "3 joints touched: %s" % str(tb))
	main._save_gesture("body")
	var txt := FileAccess.get_file_as_string(main._gesture_path("body", safe))
	var d = JSON.parse_string(txt)
	check(typeof(d) == TYPE_DICTIONARY and d.kind == "body", "body file written: " + safe)
	var keys: Array = d.positions.keys()
	keys.sort()
	check(keys == ["i02_rightHand_indexPhal_joint", "left_elbow_x_joint"], "only touched joints in the body: %s" % str(keys))
	check(absf(d.positions["left_elbow_x_joint"] - deg_to_rad(30.0)) < 1e-3, "elbow value is correct")
	check(main._touched.keys() == ["i02_head_jaw_joint"], "after saving the body the face memory remained, the body memory was cleared")
	main._save_gesture("face")
	var df = JSON.parse_string(FileAccess.get_file_as_string(main._gesture_path("face", safe)))
	check(df.positions.keys() == ["i02_head_jaw_joint"], "only the jaw in the face")
	check(main._touched.is_empty(), "after saving the face the memory is empty")
	main._save_gesture("body")
	check(main._gest_status.text.begins_with("Nothing"), "saving again without movement → “Nothing to save”")
	main._sliders["left_elbow_x_joint"].value = 10.0
	for i in 3: await process_frame
	check(main._touched.size() == 1, "after saving, new movements accumulate anew")
	await main._reset_pose()
	check(main._touched.is_empty(), "“Reset” clears the memory")
	check(main._gest_lists["body"].get_child_count() >= 1, "gesture list updated")
	robot.set_joint("left_elbow_x_joint", 0.0)
	await main._apply_gesture("body", safe)
	check(absf(robot.positions["left_elbow_x_joint"] - deg_to_rad(30.0)) < 1e-3, "“Apply” sets the pose")
	check(not main.link.send_gesture_file("body", txt), "without a link, sending → false")
	DirAccess.remove_absolute(ProjectSettings.globalize_path(main._gesture_path("body", safe)))
	DirAccess.remove_absolute(ProjectSettings.globalize_path(main._gesture_path("face", safe)))
	print("TOTAL: ", "OK" if fails == 0 else "%d FAIL" % fails)
	quit(1 if fails else 0)
