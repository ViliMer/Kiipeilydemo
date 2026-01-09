class_name ClimbingController
extends Node

var character: CharacterBody3D
var skeleton: Skeleton3D
var route: Route
var joints: Array[Generic6DOFJoint3D]

var saved_joint_angles: Dictionary
var previous_joint_angles := {}
var previous_joint_torques := {}
var run_joint_motors := false

var left_hand_target: Node3D
var right_hand_target: Node3D
var left_foot_target: Node3D
var right_foot_target: Node3D

var right_hand_force = 50.0
var left_hand_force = 50.0
var right_foot_force = 50.0
var left_foot_force = 50.0

var reaching_left_hand = false
var reaching_right_hand = false
var reaching_left_foot = false
var reaching_right_foot = false

var attached_holds = {
	"lh": null,
	"rh": null,
	"lf": null,
	"rf": null
}

var target_holds = {
	"lh": null,
	"rh": null,
	"lf": null,
	"rf": null
}

func init(character_ref: CharacterBody3D) -> void:
	character = character_ref
	skeleton = character.skeleton
	joints = character.get_joints()

	left_hand_target = character.get_node("Left Hand Target")
	right_hand_target = character.get_node("Right Hand Target")
	left_foot_target = character.get_node("Left Foot Target")
	right_foot_target = character.get_node("Right Foot Target")
	
	connect_signals()

func enter_climb(new_route: Route):
	route = new_route
	character.collision_shape.disabled = true
	
	character.velocity = Vector3.ZERO
	
	character.anim.play("hand_pose")
	character.anim.seek(0.0, true)
	character.anim.stop()
	
	await move_character_to_start(route.get_starting_holds(), route.climb_start_position, 1.0)
	
	attached_holds = route.get_starting_holds().duplicate(true)
	target_holds = route.get_starting_holds().duplicate(true)
	
	if character.left_hand_ik:
		character.left_hand_ik.start(true)
		_add_limb_lock("lh")
		character.left_hand_ik.stop()
	if character.right_hand_ik:
		character.right_hand_ik.start(true)
		_add_limb_lock("rh")
		character.right_hand_ik.stop()
	if character.left_foot_ik:
		character.left_foot_ik.start(true)
		_add_limb_lock("lf")
		character.left_foot_ik.stop()
	if character.right_foot_ik:
		character.right_foot_ik.start(true)
		_add_limb_lock("rf")
		character.right_foot_ik.stop()
	
	character.bone_sim.active = true
	character.run_bone_sim(true)

func update_climbing(delta: float) -> void:
	# Get physical bones
	var hip_bone = character.bone_sim.find_child("Physical Bone Hips")
	var upper_chest_bone = character.bone_sim.find_child("Physical Bone UpperChest")

	# HANDS
	if attached_holds["lh"]:
		var target = attached_holds["lh"].global_position
		var body_pos = get_bone_world_pos("UpperChest")
		var direction = (target - body_pos).normalized()
		apply_force(upper_chest_bone, direction, left_hand_force)
	
	if attached_holds["rh"]:
		var target = attached_holds["rh"].global_position
		var body_pos = get_bone_world_pos("UpperChest")
		var direction = (target - body_pos).normalized()
		apply_force(upper_chest_bone, direction, right_hand_force)
	
	# FEET
	if attached_holds["lf"]:
		var target = attached_holds["lf"].global_position
		var body_pos = get_bone_world_pos("Hips")
		var direction = (body_pos - target).normalized()
		apply_force(hip_bone, direction, left_foot_force)

	if attached_holds["rf"]:
		var target = attached_holds["rf"].global_position
		var body_pos = get_bone_world_pos("Hips")
		var direction = (body_pos - target).normalized()
		apply_force(hip_bone, direction, right_foot_force)
	
	reach()
	try_grab()
	
	if run_joint_motors:
		update_joint_motors(saved_joint_angles, delta)

func exit_climb():
	attached_holds = {"lh":null, "rh":null, "lf":null, "rf":null}
	_remove_limb_locks()
	route = null

func move_character_to_start(start_holds: Dictionary, start_pos: Node3D, duration := 1.0) -> void:
	var t0 := character.global_transform
	var t1 := start_pos.global_transform

	# Reset IK influences to 0
	_set_all_ik_strength(0.0)

	# Start IK solvers but at zero influence
	_start_enabled_iks(start_holds)

	# -------------------------
	# Create the tween
	# -------------------------
	var tween := create_tween()
	tween.set_parallel(true)
	tween.set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)

	# ---- POSITION LERP ----
	tween.tween_property(
		character,
		"global_position",
		t1.origin,
		duration
	)

	# ---- ROTATION SLERP ----
	tween.tween_method(
		func(alpha):
			var new_basis := t0.basis.slerp(t1.basis, alpha)
			character.global_transform = Transform3D(
				new_basis,
				character.global_position
			)
	,
	0.0, 1.0, duration)

	# ---- IK INFLUENCE LERP (0 → 1 over first 0.5s) ----
	var ik_duration = min(0.5, duration)
	tween.tween_method(
		func(a):
			_set_all_ik_strength(a)
	,
	0.0, 1.0, ik_duration)

	# -------------------------
	# UPDATE IK TARGETS EVERY FRAME
	# -------------------------
	var update_tween := create_tween().set_trans(Tween.TRANS_LINEAR)
	update_tween.tween_method(
		func(_unused):
			_update_ik_targets(start_holds)
	,
	0.0, 1.0, duration)
	# The method is called once per frame over the tween

	# -------------------------
	# Wait for all tweens to finish
	# -------------------------
	await tween.finished
	await update_tween.finished

func _start_enabled_iks(start_holds: Dictionary) -> void:
	if character.left_hand_ik and start_holds["lh"]:
		character.left_hand_ik.start()
	if character.right_hand_ik and start_holds["rh"]:
		character.right_hand_ik.start()
	if character.left_foot_ik and start_holds["lf"]:
		character.left_foot_ik.start()
	if character.right_foot_ik and start_holds["rf"]:
		character.right_foot_ik.start()

func _set_all_ik_strength(value: float) -> void:
	if character.left_hand_ik:
		character.left_hand_ik.influence = value
	if character.right_hand_ik:
		character.right_hand_ik.influence = value
	if character.left_foot_ik:
		character.left_foot_ik.influence = value
	if character.right_foot_ik:
		character.right_foot_ik.influence = value

func _update_ik_targets(holds: Dictionary) -> void:
	if holds["lh"]:
		left_hand_target.global_transform = holds["lh"].get_hand_grab_transform()
	if holds["rh"]:
		right_hand_target.global_transform = holds["rh"].get_hand_grab_transform()
	if holds["lf"]:
		left_foot_target.global_transform = holds["lf"].get_foot_grab_transform()
	if holds["rf"]:
		right_foot_target.global_transform = holds["rf"].get_foot_grab_transform()

func _add_limb_lock(limb: String) -> void:

	var bone_name: String
	match limb:
		"lh": bone_name = "Physical Bone LeftHand"
		"rh": bone_name = "Physical Bone RightHand"
		"lf": bone_name = "Physical Bone LeftFoot"
		"rf": bone_name = "Physical Bone RightFoot"
		_: return
	
	var physical_bone: PhysicalBone3D = character.bone_sim.find_child(bone_name)
	
	if physical_bone == null:
		push_error("No physical bone found for %s" % bone_name)
		return
	
	physical_bone.axis_lock_linear_x = true
	physical_bone.axis_lock_linear_y = true
	physical_bone.axis_lock_linear_z = true
	physical_bone.axis_lock_angular_x = true
	if limb == "lh" or limb == "rh":
		physical_bone.axis_lock_angular_y = true

func _remove_limb_locks() -> void:
	var limbs = ["lh", "rh", "lf", "rf"]
	for limb in limbs:
		release_limb(limb)

func reach() -> void:
	var hand_reach_force = 100.0
	var foot_reach_force = 100.0
	
	if reaching_left_hand and not attached_holds["lh"]:
		var left_hand = character.bone_sim.find_child("Physical Bone LeftHand")
		var dir = (left_hand_target.global_position - get_bone_world_pos("LeftHand"))
		apply_force(left_hand, dir, hand_reach_force)
	
	if reaching_right_hand and not attached_holds["rh"]:
		var right_hand = character.bone_sim.find_child("Physical Bone RightHand")
		var dir = (right_hand_target.global_position - get_bone_world_pos("RightHand"))
		apply_force(right_hand, dir, hand_reach_force)
	
	if reaching_left_foot and not attached_holds["lf"]:
		var left_foot = character.bone_sim.find_child("Physical Bone LeftFoot")
		var dir = (left_foot_target.global_position - get_bone_world_pos("LeftFoot"))
		apply_force(left_foot, dir, foot_reach_force)
	
	if reaching_right_foot and not attached_holds["rf"]:
		var right_foot = character.bone_sim.find_child("Physical Bone RightFoot")
		var dir = (right_foot_target.global_position - get_bone_world_pos("RightFoot"))
		apply_force(right_foot, dir, foot_reach_force)

func try_grab() -> void:
	var threshold_distance = 0.1
	
	# LEFT HAND
	if reaching_left_hand and not attached_holds["lh"]:
		var dist = get_bone_world_pos("LeftHand").distance_to(left_hand_target.global_position)
		if dist <= threshold_distance:
			grab_hold("lh")

	# RIGHT HAND
	if reaching_right_hand and not attached_holds["rh"]:
		var dist = get_bone_world_pos("RightHand").distance_to(right_hand_target.global_position)
		if dist <= threshold_distance:
			grab_hold("rh")

	# LEFT FOOT
	if reaching_left_foot and not attached_holds["lf"]:
		var dist = get_bone_world_pos("LeftFoot").distance_to(left_foot_target.global_position)
		if dist <= threshold_distance:
			grab_hold("lf")

	# RIGHT FOOT
	if reaching_right_foot and not attached_holds["rf"]:
		var dist = get_bone_world_pos("RightFoot").distance_to(right_foot_target.global_position)
		if dist <= threshold_distance:
			grab_hold("rf")


func grab_hold(code: String) -> void:
	print("Grab")
	character.run_bone_sim(false)
	copy_physical_to_skeleton()
	
	match code:
		"lh":	
			character.left_hand_ik.start(true)
		"rh":
			character.right_hand_ik.start(true)
		"lf":
			character.left_foot_ik.start(true)
		"rf":
			character.right_foot_ik.start(true)
		_:
			push_error("Error in limb_grab(): unknown code '%s'" % code)
			return
	
	_add_limb_lock(code)
	attached_holds[code] = target_holds[code]
	character.run_bone_sim(true)

func release_limb(limb: String) -> void:
	var bone_name: String
	match limb:
		"lh": bone_name = "Physical Bone LeftHand"
		"rh": bone_name = "Physical Bone RightHand"
		"lf": bone_name = "Physical Bone LeftFoot"
		"rf": bone_name = "Physical Bone RightFoot"
		_: return
	var physical_bone: PhysicalBone3D = character.bone_sim.find_child(bone_name)
	physical_bone.axis_lock_linear_x = false
	physical_bone.axis_lock_linear_y = false
	physical_bone.axis_lock_linear_z = false
	physical_bone.axis_lock_angular_x = false
	physical_bone.axis_lock_angular_y = false
	physical_bone.axis_lock_angular_z = false
	
	attached_holds[limb] = null

func apply_force(body_bone: PhysicalBone3D, direction: Vector3, magnitude: float) -> void:
	var force = direction * magnitude
	body_bone.external_force += force

func copy_physical_to_skeleton():
	for bone_name in character.bone_names:
		var physical_bone_name = character.bone_names[bone_name]
		var phys_bone: PhysicalBone3D = character.bone_sim.find_child(physical_bone_name)
		var bone_idx = skeleton.find_bone(bone_name)
		if bone_idx == -1:
			continue

		# 1. PhysicalBone is in WORLD space
		var phys_world: Transform3D = phys_bone.global_transform

		# 2. Convert world → skeleton-space
		var skeleton_world: Transform3D = skeleton.global_transform
		var phys_in_skeleton_space: Transform3D = skeleton_world.affine_inverse() * phys_world

		# 3. Compute local bone transform relative to parent bone
		var parent_idx = skeleton.get_bone_parent(bone_idx)
		if parent_idx == -1:
			# Root bone → set in skeleton-space directly
			skeleton.set_bone_global_pose(bone_idx, phys_in_skeleton_space)
		else:
			var offset_inverse = character.bone_rotation_offsets[bone_name].inverse()
			var parent_pose: Transform3D = skeleton.get_bone_global_pose(parent_idx)
			var local_pose: Transform3D = parent_pose.affine_inverse() * phys_in_skeleton_space * offset_inverse
			skeleton.set_bone_pose(bone_idx, local_pose)

func get_bone_world_pos(bone_name: String) -> Vector3:
	var physical_bone_name = character.bone_names[bone_name]
	var phys_bone: PhysicalBone3D = character.bone_sim.find_child(physical_bone_name)
	if phys_bone == null:
		return Vector3.ZERO
	
	# 1. PhysicalBone global transform (world)
	var phys_world: Transform3D = phys_bone.global_transform
	
	# 2. Offset that maps bone → physical, so inverse maps physical → bone
	var offset: Transform3D = character.bone_rotation_offsets[bone_name]
	var offset_inv: Transform3D = offset.inverse()
	
	# 3. Compute **true bone world transform**
	var bone_world: Transform3D = phys_world * offset_inv
	
	return bone_world.origin

func save_joint_angles() -> Dictionary:
	var result := {}
	if joints == null:
		return result

	for joint in joints:
		var node_a := joint.get_node_or_null(joint.node_a)
		var node_b := joint.get_node_or_null(joint.node_b)

		if node_a == null or node_b == null:
			continue
		
		var Ba = node_a.global_transform.basis
		var Bb = node_b.global_transform.basis
		var Bj = joint.global_transform.basis
		
		var A_in_joint = Bj.inverse() * Ba
		var B_in_joint = Bj.inverse() * Bb
		
		var rel_basis = A_in_joint.inverse() * B_in_joint
		
		#var a_basis = node_a.global_transform.basis
		#var b_basis = node_b.global_transform.basis
		#var j_basis = joint.global_transform.basis

		#var rel_basis = (j_basis.inverse() * a_basis).inverse() * (j_basis.inverse() * b_basis)
		var q : Quaternion = rel_basis.get_rotation_quaternion().normalized()

		result[joint.name] = {
			"qx": q.x,
			"qy": q.y,
			"qz": q.z,
			"qw": q.w
		}

	saved_joint_angles = result
	return result

func enable_joint_motors(strength := 1.0) -> void:
	run_joint_motors = true
	return
	if joints == null:
		return

	var max_force := 10.0 * strength

	for joint in joints:

		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		
		#joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		#joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		#joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)

		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)

func disable_joint_motors() -> void:
	run_joint_motors = false
	return
	if joints == null:
		return

	for joint in joints:

		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)

var JOINT_LIMITS = {
	"Spine Joint":			{ "max_torque": 120.0 },
	"LowerChest Joint":		{ "max_torque": 120.0 },
	"Chest Joint":			{ "max_torque": 120.0 },
	"LeftUpperChest Joint":	{ "max_torque": 120.0 },
	"LeftShoulder Joint":	{ "max_torque": 80.0 },
	"LeftElbow Joint":		{ "max_torque": 40.0 },
	"LeftWrist Joint":		{ "max_torque": 15.0 },
	"RightUpperChest Joint":{ "max_torque": 120.0 },
	"RightShoulder Joint":	{ "max_torque": 80.0 },
	"RightElbow Joint":		{ "max_torque": 40.0 },
	"RightWrist Joint":		{ "max_torque": 15.0 },
	"LeftHip Joint":		{ "max_torque": 150.0 },
	"LeftKnee Joint":		{ "max_torque": 120.0 },
	"LeftAnkle Joint":		{ "max_torque": 50.0 },
	"RightHip Joint":		{ "max_torque": 150.0 },
	"RightKnee Joint":		{ "max_torque": 120.0 },
	"RightAnkle Joint":		{ "max_torque": 50.0 },
}

func update_joint_motors(saved: Dictionary, delta: float, strength := 1.0) -> void:
	if joints == null or saved == null:
		return

	var Kp := 5.0 * strength	# proportional gain
	var Kd := 0.0 * strength	# derivative gain

	for joint in joints:

		var joint_name = joint.name
		if not saved.has(joint_name):
			continue

		# ---- RESOLVE THE TWO BODIES ----
		var node_a := joint.get_node_or_null(joint.node_a)
		var node_b := joint.get_node_or_null(joint.node_b)
		if node_a == null or node_b == null:
			continue
		
		var Ba = node_a.global_transform.basis
		var Bb = node_b.global_transform.basis
		var Bj = joint.global_transform.basis
		
		var A_in_joint = Bj.inverse() * Ba
		var B_in_joint = Bj.inverse() * Bb
		
		var rel_basis = A_in_joint.inverse() * B_in_joint
		
		var q_current : Quaternion = rel_basis.get_rotation_quaternion().normalized()
		
		# ---- TARGET QUATERNION ----
		var s = saved[joint_name]
		var q_target = Quaternion(s["qx"], s["qy"], s["qz"], s["qw"]).normalized()

		# ---- ROTATION DELTA (shortest path) ----
		var q_delta : Quaternion = (q_target * q_current.inverse()).normalized()
		if q_delta.w < 0.0:
			q_delta = Quaternion(-q_delta.x, -q_delta.y, -q_delta.z, -q_delta.w)
		
		var axis = q_delta.get_axis()
		var angle = q_delta.get_angle()
		
		# joint-local angular error vector
		var error_vec : Vector3 = axis * angle
		
		debug_draw_joint_error(joint, error_vec)
		if joint_name == "RightElbow Joint":
			print(joint_name, " error angle (deg): ", rad_to_deg(angle))
		
		# ---- ANGULAR VELOCITY (derivative term) ----
		var prev_q : Quaternion = previous_joint_angles.get(joint_name, q_current)
		var q_vel = (prev_q.inverse() * q_current).normalized()
		
		# Ensure shortest-path rotation
		if q_vel.w < 0.0:
			q_vel = Quaternion(-q_vel.x, -q_vel.y, -q_vel.z, -q_vel.w)

		var angular_velocity = Vector3.ZERO
		var vel_angle := q_vel.get_angle()
		if vel_angle > 0.00001:
			angular_velocity = q_vel.get_axis() * (vel_angle / delta)

		previous_joint_angles[joint_name] = q_current
		
		# ---- PD CONTROL ----
		var target_torque = (error_vec * Kp) - (angular_velocity * Kd)
		''' Ignore for nows
		var max_torque = JOINT_LIMITS[joint_name].max_torque
		if target_torque.length() > max_torque:
			target_torque = target_torque.normalized() * max_torque
		
		var prev_torque = previous_joint_torques.get(joint_name, Vector3.ZERO)
		var max_delta = max_torque * 1.0 * delta  # tune per joint

		var delta_torque = target_torque - prev_torque
		if delta_torque.length() > max_delta:
			delta_torque = delta_torque.normalized() * max_delta

		target_torque = prev_torque + delta_torque
		previous_joint_torques[joint_name] = target_torque
		'''
		var torque_world = joint.global_transform.basis * target_torque
		
		DebugDraw3D.draw_arrow(
			joint.global_transform.origin,
			joint.global_transform.origin + torque_world,
			Color.BLUE,
			0.01
		)
		

		node_b.external_torque -= torque_world
		#node_a.external_torque += torque_world

func debug_draw_joint_error(joint: Generic6DOFJoint3D, error_joint_local: Vector3) -> void:
	var origin = joint.global_transform.origin
	
	var world_dir = joint.global_transform.basis * error_joint_local
	
	DebugDraw3D.draw_arrow(origin, origin + world_dir*2, Color.RED, 0.01)

#### Signal stuff ####
func connect_signals() -> void:
	
	# Left hand UI signals
	SignalBus.left_hand_strength_changed.connect(_on_left_hand_strength_changed)
	SignalBus.release_left_hand.connect(_on_release_left_hand)
	SignalBus.left_hand_target_changed.connect(_on_left_hand_target_changed)
	SignalBus.reach_left_hand.connect(_on_reach_left_hand)
	
	# Right hand UI signals
	SignalBus.right_hand_strength_changed.connect(_on_right_hand_strength_changed)
	SignalBus.release_right_hand.connect(_on_release_right_hand)
	SignalBus.right_hand_target_changed.connect(_on_right_hand_target_changed)
	SignalBus.reach_right_hand.connect(_on_reach_right_hand)
	
	# Left leg UI signals
	SignalBus.left_leg_strength_changed.connect(_on_left_leg_strength_changed)
	SignalBus.release_left_foot.connect(_on_release_left_foot)
	SignalBus.left_foot_target_changed.connect(_on_left_foot_target_changed)
	SignalBus.reach_left_foot.connect(_on_reach_left_foot)
	
	# Right leg UI signals
	SignalBus.right_leg_strength_changed.connect(_on_right_leg_strength_changed)
	SignalBus.release_right_foot.connect(_on_release_right_foot)
	SignalBus.right_foot_target_changed.connect(_on_right_foot_target_changed)
	SignalBus.reach_right_foot.connect(_on_reach_right_foot)
	
	# Other
	SignalBus.save_pose.connect(_on_save_pose)
	SignalBus.reach_pose.connect(_on_reach_pose)

func _on_left_hand_strength_changed(value: float) -> void:
	left_hand_force = value

func _on_right_hand_strength_changed(value: float) -> void:
	right_hand_force = value

func _on_left_leg_strength_changed(value: float) -> void:
	left_foot_force = value

func _on_right_leg_strength_changed(value: float) -> void:
	right_foot_force = value

func _on_release_left_hand() -> void:
	release_limb("lh")
	print("Release left hand pressed")

func _on_release_right_hand() -> void:
	release_limb("rh")
	print("Release right hand pressed")

func _on_release_left_foot() -> void:
	release_limb("lf")
	print("Release left foot pressed")

func _on_release_right_foot() -> void:
	release_limb("rf")
	print("Release right foot pressed")

func _on_reach_left_hand() -> void:
	reaching_left_hand = not reaching_left_hand

func _on_reach_right_hand() -> void:
	reaching_right_hand = not reaching_right_hand

func _on_reach_left_foot() -> void:
	reaching_left_foot = not reaching_left_foot

func _on_reach_right_foot() -> void:
	reaching_right_foot = not reaching_right_foot

func _on_left_hand_target_changed(value: int) -> void:
	# To be specific, value is the index of the selected item in the dropdown menu. Should be the same as the index of the hold in route.get_holds()
	var hold = route.get_holds()[value]
	target_holds["lh"] = hold
	left_hand_target.global_transform = hold.get_hand_grab_transform()

func _on_right_hand_target_changed(value: int) -> void:
	var hold = route.get_holds()[value]
	target_holds["rh"] = hold
	right_hand_target.global_transform = hold.get_hand_grab_transform()

func _on_left_foot_target_changed(value: int) -> void:
	var hold = route.get_holds()[value]
	target_holds["lf"] = hold
	left_foot_target.global_transform = hold.get_foot_grab_transform()

func _on_right_foot_target_changed(value: int) -> void:
	var hold = route.get_holds()[value]
	target_holds["rf"] = hold
	right_foot_target.global_transform = hold.get_foot_grab_transform()

func _on_save_pose() -> void:
	var res = save_joint_angles()
	print("Button pressed")
	print(res)

func _on_reach_pose() -> void:
	if run_joint_motors:
		print("Disabling motors")
		disable_joint_motors()
	else:
		print("Driving motors to reach target pose")
		enable_joint_motors()
