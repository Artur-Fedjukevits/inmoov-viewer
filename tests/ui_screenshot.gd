## UI screenshot: arms out to the sides, the given joint selected. SHOT_OUT=file SHOT_PART=part name (optional)
## xvfb-run godot --path . --rendering-method gl_compatibility --resolution 1280x720 -s res://tests/ui_screenshot.gd
extends SceneTree

func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 12: await process_frame
	var robot: InmoovRobot = main.get_node("Robot")
	for jn in ["left_shoulder_x_joint", "right_shoulder_x_joint"]:
		var lim := robot.get_limits(jn)
		print(jn, " limits ", rad_to_deg(lim.x), " .. ", rad_to_deg(lim.y))
	for jn in ["left_shoulder_x_joint", "right_shoulder_x_joint"]:
		var lim := robot.get_limits(jn)
		robot.set_joint(jn, deg_to_rad(90.0))
	var part := OS.get_environment("SHOT_PART")
	if part != "":
		main._select_part(main._part_of_joint(part))
	for i in 20: await process_frame
	root.get_viewport().get_texture().get_image().save_png(OS.get_environment("SHOT_OUT"))
	quit()
