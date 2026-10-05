## Simulated finger touch on the first slider (xvfb). Prints the value before/after.
extends SceneTree

func _initialize() -> void:
	var main: Node = load("res://main.tscn").instantiate()
	root.add_child(main)
	for i in 10: await process_frame
	var s: HSlider = main._sliders.values()[1]
	var r := s.get_global_rect()
	print("slider rect ", r, " value ", s.value)
	var y := r.get_center().y
	var x0 := r.position.x + r.size.x * 0.2
	var x1 := r.position.x + r.size.x * 0.8
	var t := InputEventScreenTouch.new(); t.index = 0; t.position = Vector2(x0, y); t.pressed = true
	Input.parse_input_event(t)
	for i in 3: await process_frame
	print("after tap value ", s.value)
	for k in 10:
		var d := InputEventScreenDrag.new(); d.index = 0
		d.position = Vector2(lerp(x0, x1, (k + 1) / 10.0), y); d.relative = Vector2((x1 - x0) / 10.0, 0)
		Input.parse_input_event(d)
		await process_frame
	t = InputEventScreenTouch.new(); t.index = 0; t.position = Vector2(x1, y); t.pressed = false
	Input.parse_input_event(t)
	for i in 3: await process_frame
	print("after drag value ", s.value, "  (min ", s.min_value, " max ", s.max_value, ")")
	quit()
