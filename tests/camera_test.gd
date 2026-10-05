## Camera window test: needs a stream, e.g.
##   python3 tests/mock_camera_ws.py
## then: godot --headless --path . -s res://tests/camera_test.gd [-- 127.0.0.1]
extends SceneTree

var fails := 0


func check(cond: bool, msg: String) -> void:
	print(("  OK   " if cond else "  FAIL ") + msg)
	if not cond:
		fails += 1


func _initialize() -> void:
	var host := "127.0.0.1"
	var ua := OS.get_cmdline_user_args()
	if ua.size() > 0:
		host = ua[0]
	check(CameraView.make_url("192.168.10.202") == "ws://192.168.10.202:9091", "URL from IP")
	check(CameraView.make_url("ws://192.168.10.202:9090") == "ws://192.168.10.202:9091", "URL from rosbridge address")
	var main: Node = (load("res://main.tscn") as PackedScene).instantiate()
	root.add_child(main)
	await create_timer(0.5).timeout
	var cv: CameraView = main.camera_view
	check(cv != null and cv.visible, "camera window created")
	cv.connect_to(host)
	await create_timer(4.0).timeout
	print("  frames %d, dropped %d, errors %d, size %s" % [cv.frames, cv.dropped, cv.bad, cv.last_frame_size])
	check(cv.frames >= 40, "frames arrive (~15 fps, ≥ 40 in 4 s)")
	check(cv.last_frame_size == Vector2i(640, 480), "frame 640×480")
	check(cv.bad == 0, "all frames decoded")
	check(cv.is_streaming(), "stream active")
	cv.disconnect_stream()
	await create_timer(0.5).timeout
	var n := cv.frames
	await create_timer(1.0).timeout
	check(cv.frames == n, "no frames after disconnect")
	print("RESULT: %s (%d fails)" % ["PASS" if fails == 0 else "FAIL", fails])
	quit(fails)
