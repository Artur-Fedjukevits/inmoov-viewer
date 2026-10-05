# Copyright (C) 2026 Artur-Fedjukevits
# Author: Artur-Fedjukevits
# SPDX-License-Identifier: GPL-3.0-or-later

## Headless tests: godot --headless --path . -s res://tests/run_tests.gd
extends SceneTree

var fails := 0

func check(cond: bool, msg: String) -> void:
	print(("  OK   " if cond else "  FAIL ") + msg)
	if not cond:
		fails += 1


func fk(robot: InmoovRobot, link: String, by_child: Dictionary) -> Transform3D:
	if not by_child.has(link):
		return Transform3D.IDENTITY
	var j: Dictionary = by_child[link]
	var o := Transform3D(InmoovRobot._urdf_basis(j.origin_rpy), Vector3(j.origin_xyz[0], j.origin_xyz[1], j.origin_xyz[2]))
	var r := Transform3D(Basis(Quaternion(j.axis_v, robot.positions[j.name])), Vector3.ZERO)
	return fk(robot, j.parent, by_child) * o * r


func _initialize() -> void:
	var robot := InmoovRobot.new()
	root.add_child(robot)
	await physics_frame
	await physics_frame
	print("joints=%d controllable=%d links=%d areas=%d" % [robot.joints.size(), robot.controllable.size(), robot.link_nodes.size(), robot.link_areas.size()])
	check(robot.joints.size() == 64, "64 joints found in glb (62 + 2 fixed OAK)")
	check(robot.link_nodes.size() == 65, "65 links found in glb (63 + oak_mount_link, optical frame)")

	# FK: glb node transforms == URDF chain from JSON (for a random pose)
	var by_child := {}
	for jn in robot.joints:
		by_child[robot.joints[jn].child] = robot.joints[jn]
	seed(7)
	var p := {}
	for jn in robot.controllable:
		var lim := robot.get_limits(jn)
		p[jn] = randf_range(lim.x, lim.y)
	robot.set_pose(p)
	var base: Node3D = robot.link_nodes[robot.root_link]
	var maxerr := 0.0
	for link: String in robot.link_nodes:
		var n: Node3D = robot.link_nodes[link]
		var got := base.global_transform.affine_inverse() * n.global_transform
		var exp := fk(robot, link, by_child)
		maxerr = maxf(maxerr, (got.origin - exp.origin).length())
		for i in 3:
			maxerr = maxf(maxerr, (got.basis[i] - exp.basis[i]).length())
	check(maxerr < 1e-4, "FK glb == URDF, max err %.8f" % maxerr)

	# mimic
	var m := ""
	for jn in robot.joints:
		if robot.joints[jn].has("mimic"):
			m = jn; break
	var mj: Dictionary = robot.joints[m].mimic
	check(absf(robot.positions[m] - (robot.positions[mj.joint] * mj.multiplier + mj.offset)) < 1e-6, "mimic %s follows %s" % [m, mj.joint])

	# limits
	robot.set_joint("left_elbow_x_joint", 5.0)
	check(is_equal_approx(robot.positions["left_elbow_x_joint"], robot.get_limits("left_elbow_x_joint").y), "elbow limit is applied")

	# initial pose — no self-collisions
	robot.reset_pose()
	for i in 3: await physics_frame
	var pairs := robot.compute_colliding_pairs()
	print("  rest pairs: ", pairs)
	check(pairs.is_empty(), "initial pose has no self-collisions")

	# known real contact: tilting the torso sideways -> forearm ↔ thigh/pelvis
	var found := {}
	for a in [0.26, -0.26]:
		pairs = await robot.check_pose({"torso_y_joint": a})
		print("  torso_y=%.2f pairs: %s" % [a, pairs])
		found[a] = not pairs.is_empty()
	check(found[0.26] or found[-0.26], "torso_y ±15° tilt produces a collision (as expected from MoveIt)")

	# arm “into” the torso
	pairs = await robot.check_pose({"right_shoulder_z_joint": robot.get_limits("right_shoulder_z_joint").x, "right_elbow_x_joint": 1.2, "right_shoulder_x_joint": 0.6})
	print("  arm-in pairs: ", pairs)

	# random poses: how many are valid
	var ok := 0
	for k in 40:
		var q := {}
		for jn in robot.controllable:
			var lim := robot.get_limits(jn)
			q[jn] = randf_range(lim.x, lim.y)
		if (await robot.check_pose(q)).is_empty():
			ok += 1
	print("  random poses without collision: %d/40" % ok)

	print("RESULT: %s (%d fails)" % ["PASS" if fails == 0 else "FAIL", fails])
	quit(fails)
