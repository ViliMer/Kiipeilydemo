class_name ClimbingController
extends Node

var character: CharacterBody3D
var skeleton: Skeleton3D
var route: Route
var joints: Array[Generic6DOFJoint3D]

var saved_joint_angles: Dictionary
var initial_joint_params: Dictionary
var previous_joint_angles := {}
var previous_joint_torques := {}
var reach_pose := false

var joint_motor_velocity_deadzone = 0.05

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
	save_joint_params()

	left_hand_target = character.get_node("Left Hand Target")
	right_hand_target = character.get_node("Right Hand Target")
	left_foot_target = character.get_node("Left Foot Target")
	right_foot_target = character.get_node("Right Foot Target")
	
	connect_signals()

func _input(_event: InputEvent) -> void:
	if not character.state == character.PlayerState.CLIMB:
		return
	if Input.is_action_just_pressed("Save Pose"):
		print("Saving free joint angles...")
		_on_save_pose()

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
	if limb == "lh" or limb == "rh":
		physical_bone.axis_lock_angular_x = true
		physical_bone.axis_lock_angular_y = true
	if limb == "lf" or limb == "rf":
		physical_bone.axis_lock_angular_z = true
		physical_bone.axis_lock_angular_x = true

func _remove_limb_locks() -> void:
	var limbs = ["lh", "rh", "lf", "rf"]
	for limb in limbs:
		release_limb(limb)

# This function saves current joint angular spring stiffness and damping for all joints
func save_joint_params() -> void:
	for joint in joints:
		var joint_name = joint.name
		var angular_spring_stiffness: Vector3 = Vector3(joint.get_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS), joint.get_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS), joint.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS))
		var angular_spring_damping: Vector3 = Vector3(joint.get_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING), joint.get_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING), joint.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING))
		initial_joint_params[joint_name] = {
			"stiffness": angular_spring_stiffness,
			"damping": angular_spring_damping
		}

# This function applies joint angular spring stiffness and damping for all joints
func apply_joint_params(joint_params: Dictionary) -> void:
	for joint in joints:
		var stiffness = joint_params[joint.name].stiffness
		var damping = joint_params[joint.name].damping
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, stiffness.x)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, stiffness.y)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, stiffness.z)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, damping.x)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, damping.y)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, damping.z)


func update_climbing(_delta: float) -> void:
	var omega_IK = get_omega_IK()
	try_reach_limbs(omega_IK)
	try_reach_pose(omega_IK)
	try_grab()

func try_reach_limbs(omega_IK: Dictionary) -> void:
	for joint in joints:
		if joint.name in omega_IK:
			var omega = omega_IK[joint.name]
			set_joint_velocity(joint, omega)

func try_reach_pose(omega_IK: Dictionary) -> void:
	if not reach_pose:
		return
	
	for joint in joints:
		if (joint.name not in omega_IK) and (saved_joint_angles.has(joint.name)):
			var omega = compute_pose_driven_joint_velocity(joint, saved_joint_angles)
			set_joint_velocity(joint, omega)
			# Visualize velocity
			var vel_world = joint.global_transform.basis * omega
			DebugDraw3D.draw_arrow(joint.global_transform.origin,joint.global_transform.origin + vel_world,Color.BLUE,0.01)

func try_grab() -> void:
	var threshold_distance = 0.03
	
	# LEFT HAND
	if reaching_left_hand and not attached_holds["lh"]:
		var dist = get_bone_world_transform("LeftHand").origin.distance_to(left_hand_target.global_position)
		if dist <= threshold_distance:
			grab_hold("lh")

	# RIGHT HAND
	if reaching_right_hand and not attached_holds["rh"]:
		var dist = get_bone_world_transform("RightHand").origin.distance_to(right_hand_target.global_position)
		if dist <= threshold_distance:
			grab_hold("rh")

	# LEFT FOOT
	if reaching_left_foot and not attached_holds["lf"]:
		var dist = get_bone_world_transform("LeftFoot").origin.distance_to(left_foot_target.global_position)
		if dist <= threshold_distance:
			grab_hold("lf")

	# RIGHT FOOT
	if reaching_right_foot and not attached_holds["rf"]:
		var dist = get_bone_world_transform("RightFoot").origin.distance_to(right_foot_target.global_position)
		if dist <= threshold_distance:
			grab_hold("rf")

func get_omega_IK() -> Dictionary:
	
	var result = {}
	
	# This implementation assumes there is 0 or 1 reaching limbs, fix later
	var end_effector_pos = Vector3.ZERO
	var target_pos = Vector3.ZERO
	
	var end_effector_rot = Quaternion.IDENTITY
	var target_rot = Quaternion.IDENTITY
	
	if reaching_left_hand and not attached_holds["lh"]:
		end_effector_pos = get_bone_world_transform("LeftHand").origin
		target_pos = left_hand_target.global_position
		
		end_effector_rot = get_bone_world_transform("LeftHand").basis.get_rotation_quaternion()
		target_rot = left_hand_target.global_basis.get_rotation_quaternion()
	
	if reaching_right_hand and not attached_holds["rh"]:
		end_effector_pos = get_bone_world_transform("RightHand").origin
		target_pos = right_hand_target.global_position
		
		end_effector_rot = get_bone_world_transform("RightHand").basis.get_rotation_quaternion()
		target_rot = right_hand_target.global_basis.get_rotation_quaternion()
	
	if reaching_left_foot and not attached_holds["lf"]:
		end_effector_pos = get_bone_world_transform("LeftFoot").origin
		target_pos = left_foot_target.global_position
		
		end_effector_rot = get_bone_world_transform("LeftFoot").basis.get_rotation_quaternion()
		target_rot = left_foot_target.global_basis.get_rotation_quaternion()
	
	if reaching_right_foot and not attached_holds["rf"]:
		end_effector_pos = get_bone_world_transform("RightFoot").origin
		target_pos = right_foot_target.global_position
		
		end_effector_rot = get_bone_world_transform("RightFoot").basis.get_rotation_quaternion()
		target_rot = right_foot_target.global_basis.get_rotation_quaternion()
	
	var reaching_joints = get_reaching_joints()
	
	var target_vel = compute_desired_end_effector_velocity(end_effector_pos, target_pos)
	var position_jacobian = compute_position_jacobian(reaching_joints, end_effector_pos)
	var pos_IK_velocities = compute_position_joint_velocities(position_jacobian, target_vel)
	
	var target_angular_vel = compute_desired_end_effector_angular_velocity(end_effector_rot, target_rot)
	var rotation_jacobian = compute_rotation_jacobian(reaching_joints)
	var rot_IK_velocities = compute_rotation_joint_velocities(rotation_jacobian, target_angular_vel)
	
	for joint in reaching_joints:
		var omega_target_joint = -pos_IK_velocities[joint.name] - rot_IK_velocities[joint.name] * 0.2
		result[joint.name] = omega_target_joint
	
	return result

func get_reaching_joints() -> Array[Generic6DOFJoint3D]:
	var reaching_joint_names = []
	var reaching_joints: Array[Generic6DOFJoint3D] = []
	
	if reaching_left_hand and not attached_holds["lh"]:
		reaching_joint_names.append_array(["LeftUpperChest Joint", "LeftShoulder Joint", "LeftElbow Joint", "LeftWrist Joint"])
	if reaching_right_hand and not attached_holds["rh"]:
		reaching_joint_names.append_array(["RightUpperChest Joint", "RightShoulder Joint", "RightElbow Joint", "RightWrist Joint"])
	if reaching_left_foot and not attached_holds["lf"]:
		reaching_joint_names.append_array(["LeftHip Joint", "LeftKnee Joint", "LeftAnkle Joint"])
	if reaching_right_foot and not attached_holds["rf"]:
		reaching_joint_names.append_array(["RightHip Joint", "RightKnee Joint", "RightAnkle Joint"])
	
	for joint in joints:
		if joint.name in reaching_joint_names:
			reaching_joints.append(joint)

	return reaching_joints

func set_joint_velocity(joint: Generic6DOFJoint3D, omega: Vector3):
	joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega.x)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega.y)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega.z)

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

func get_bone_world_transform(bone_name: String) -> Transform3D:
	var physical_bone_name = character.bone_names[bone_name]
	var phys_bone: PhysicalBone3D = character.bone_sim.find_child(physical_bone_name)
	if phys_bone == null:
		return Transform3D.IDENTITY
	
	# 1. PhysicalBone global transform (world)
	var phys_world: Transform3D = phys_bone.global_transform
	
	# 2. Offset that maps bone → physical, so inverse maps physical → bone
	var offset: Transform3D = character.bone_rotation_offsets[bone_name]
	var offset_inv: Transform3D = offset.inverse()
	
	# 3. Compute **true bone world transform**
	var bone_world: Transform3D = phys_world * offset_inv
	
	return bone_world

func get_joint_rot_quaternion(joint: Generic6DOFJoint3D) -> Quaternion:
	var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
	var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)

	if node_a == null or node_b == null:
		push_error("No node_a or node_b in get_joint_rot_quaternion()")
		return Quaternion.IDENTITY
	
	var B_joint: Basis = joint.basis
	var Ba: Basis = node_a.global_basis
	var Bb: Basis = node_b.global_basis
	
	# In joint space:
	var rel_basis = B_joint.inverse() * (Ba.inverse() * Bb) * B_joint
	var q : Quaternion = rel_basis.get_rotation_quaternion().normalized()
	
	return q

func save_joint_angles() -> Dictionary:
	var result := {}
	if joints == null:
		return result

	for joint in joints:
		var q : Quaternion = get_joint_rot_quaternion(joint)

		result[joint.name] = {
			"qx": q.x,
			"qy": q.y,
			"qz": q.z,
			"qw": q.w
		}

	saved_joint_angles = result
	return result

# This function returns P and D in world space
func get_PD_data(joint: Generic6DOFJoint3D, target: Dictionary) -> Dictionary:

	var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
	var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)
	if node_a == null or node_b == null:
		push_warning("Joint '%s' has missing nodes." % joint.name)
		return {"error_vec": Vector3.ZERO,"angular_velocity": Vector3.ZERO, "error_integral": Vector3.ZERO}

	# ---- RELATIVE ROTATION IN JOINT SPACE ----
	var q_current: Quaternion = get_joint_rot_quaternion(joint)

	# ---- TARGET QUATERNION (joint-local) ----
	var s = target.get(joint.name, null)
	if s == null:
		return {"error_vec": Vector3.ZERO, "angular_velocity": Vector3.ZERO, "error_integral": Vector3.ZERO}

	var q_target := Quaternion(s["qx"], s["qy"], s["qz"], s["qw"]).normalized()

	# ---- ROTATION ERROR (current → target) ----
	var q_delta: Quaternion = (q_current.inverse() * q_target).normalized()
	if q_delta.w < 0.0:
		q_delta = -q_delta
	
	var error_vec := Vector3.ZERO
	
	var v := Vector3(q_delta.x, q_delta.y, q_delta.z)
	var v_len := v.length()

	if v_len > 1e-6:
		var error_angle := 2.0 * atan2(v_len, q_delta.w)
		error_vec = (v / v_len) * error_angle
		error_vec = joint.global_transform.basis * error_vec

	# ---- ANGULAR VELOCITY (DERIVATIVE TERM) ----
	var w_a = node_a.angular_velocity
	var w_b = node_b.angular_velocity
	var angular_velocity = w_b - w_a

	# Returns in world space
	return {
		"error_vec": error_vec,
		"angular_velocity": angular_velocity,
	}

'''
# Torque motor values
var JOINT_CONSTANTS = {
	"Spine Joint":			{ "max_torque": 200.0, "Kd": Vector3(5.0, 5.0, 5.0), "omega_max": Vector3(5.0, 5.0, 5.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
	"LowerChest Joint":		{ "max_torque": 120.0, "Kd": Vector3(4.0, 4.0, 4.0), "omega_max": Vector3(5.0, 5.0, 5.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
	"Chest Joint":			{ "max_torque": 120.0, "Kd": Vector3(3.0, 3.0, 3.0), "omega_max": Vector3(5.0, 5.0, 5.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
	"LeftUpperChest Joint":	{ "max_torque": 120.0, "Kd": Vector3(0.0, 2.5, 3.5), "omega_max": Vector3(0.0, 10.0, 10.0), "error_scale": Vector3(1.0, 0.25, 0.25) }, # x = twist, y = fwd/back, z = up/down
	"LeftShoulder Joint":	{ "max_torque": 80.0, "Kd": Vector3(0.5, 2.0, 2.0), "omega_max": Vector3(5.0, 12.0, 12.0), "error_scale": Vector3(0.5, 0.25, 0.25) }, # x = twist, y = forward / back, z = lateral raise
	"LeftElbow Joint":		{ "max_torque": 60.0, "Kd": Vector3(0.0, 1.0, 0.0), "omega_max": Vector3(0.0, 12.0, 0.0), "error_scale": Vector3(1.0, 0.25, 1.0) },
	"LeftWrist Joint":		{ "max_torque": 30.0, "Kd": Vector3(0.1, 0.1, 0.2), "omega_max": Vector3(5.0, 5.0, 12.0), "error_scale": Vector3(0.5, 0.5, 0.25) }, # x = supination / pronation, y = abduction / adduction, z = flexion/extension
	"RightUpperChest Joint":{ "max_torque": 120.0, "Kd": Vector3(0.0, 2.5, 3.5), "omega_max": Vector3(0.0, 10.0, 10.0), "error_scale": Vector3(1.0, 0.25, 0.25) }, # x = twist, y = fwd/back, z = up/down
	"RightShoulder Joint":	{ "max_torque": 80.0, "Kd": Vector3(0.5, 2.0, 2.0), "omega_max": Vector3(5.0, 12.0, 12.0), "error_scale": Vector3(0.5, 0.25, 0.25) }, # x = twist, y = forward / back, z = lateral raise
	"RightElbow Joint":		{ "max_torque": 60.0, "Kd": Vector3(0.0, 1.0, 0.0), "omega_max": Vector3(0.0, 12.0, 0.0), "error_scale": Vector3(1.0, 0.25, 1.0) },
	"RightWrist Joint":		{ "max_torque": 30.0, "Kd": Vector3(0.1, 0.1, 0.2), "omega_max": Vector3(5.0, 5.0, 12.0), "error_scale": Vector3(0.5, 0.5, 0.25) }, # x = supination / pronation, y = abduction / adduction, z = flexion/extension
	"LeftHip Joint":		{ "max_torque": 150.0, "Kd": Vector3(6.0, 0.5, 4.0), "omega_max": Vector3(20.0, 10.0, 20.0), "error_scale": Vector3(0.3, 0.5, 0.3) }, # Here y is the twist axis
	"LeftKnee Joint":		{ "max_torque": 120.0, "Kd": Vector3(2.0, 0.0, 0.0), "omega_max": Vector3(20.0, 0.0, 0.0), "error_scale": Vector3(0.3, 1.0, 1.0) },
	"LeftAnkle Joint":		{ "max_torque": 50.0, "Kd": Vector3(0.5, 0.5, 0.0), "omega_max": Vector3(10.0, 10.0, 0.0), "error_scale": Vector3(0.2, 0.2, 1.0) },
	"RightHip Joint":		{ "max_torque": 150.0, "Kd": Vector3(6.0, 0.5, 4.0), "omega_max": Vector3(20.0, 10.0, 20.0), "error_scale": Vector3(0.3, 0.5, 0.3) },
	"RightKnee Joint":		{ "max_torque": 120.0, "Kd": Vector3(2.0, 0.0, 0.0), "omega_max": Vector3(20.0, 0.0, 0.0), "error_scale": Vector3(0.3, 1.0, 1.0) },
	"RightAnkle Joint":		{ "max_torque": 50.0, "Kd": Vector3(0.5, 0.5, 0.0), "omega_max": Vector3(10.0, 10.0, 0.0), "error_scale": Vector3(0.2, 0.2, 1.0) },
}'''

# Velocity motor values
var JOINT_CONSTANTS = {
	"Spine Joint":			{ "max_torque": 200.0, "omega_max": Vector3(0.5, 0.5, 0.5), "error_scale": Vector3(0.5, 0.5, 0.5), "stiffness": Vector3(20.0, 20.0, 20.0), "damping": Vector3(100.0, 100.0, 100.0) }, # x = fwd/back, y = twist, z = side to side
	"LowerChest Joint":		{ "max_torque": 120.0, "omega_max": Vector3(0.5, 0.5, 0.5), "error_scale": Vector3(0.5, 0.5, 0.5), "stiffness": Vector3(10.0, 10.0, 10.0), "damping": Vector3(75.0, 75.0, 75.0) }, # x = fwd/back, y = twist, z = side to side
	"Chest Joint":			{ "max_torque": 120.0, "omega_max": Vector3(0.5, 0.5, 0.5), "error_scale": Vector3(0.5, 0.5, 0.5), "stiffness": Vector3(10.0, 10.0, 10.0), "damping": Vector3(50.0, 50.0, 50.0) }, # x = fwd/back, y = twist, z = side to side
	"LeftUpperChest Joint":	{ "max_torque": 120.0, "omega_max": Vector3(0.0, 3.0, 3.0), "error_scale": Vector3(1.0, 0.5, 0.5), "stiffness": Vector3(0.0, 20.0, 100.0), "damping": Vector3(0.0, 30.0, 30.0) }, # x = twist, y = fwd/back, z = up/down
	"LeftShoulder Joint":	{ "max_torque": 80.0, "omega_max": Vector3(1.0, 3.0, 3.0), "error_scale": Vector3(1.0, 0.5, 0.5), "stiffness": Vector3(0.001, 0.001, 0.001), "damping": Vector3(20.0, 20.0, 20.0) }, # x = twist, y = forward / back, z = lateral raise
	"LeftElbow Joint":		{ "max_torque": 60.0, "omega_max": Vector3(0.0, 3.0, 0.0), "error_scale": Vector3(1.0, 0.5, 1.0), "stiffness": Vector3(0.0, 0.01, 0.0), "damping": Vector3(0.0, 10.0, 0.0) },
	"LeftWrist Joint":		{ "max_torque": 30.0, "omega_max": Vector3(0.5, 0.5, 1.2), "error_scale": Vector3(1.0, 1.0, 0.5), "stiffness": Vector3(0.01, 0.01, 0.01), "damping": Vector3(5.0, 5.0, 5.0) }, # x = supination / pronation, y = abduction / adduction, z = flexion/extension
	"RightUpperChest Joint":{ "max_torque": 120.0, "omega_max": Vector3(0.0, 3.0, 3.0), "error_scale": Vector3(1.0, 0.5, 0.5), "stiffness": Vector3(0.0, 20.0, 100.0), "damping": Vector3(0.0, 30.0, 30.0) }, # x = twist, y = fwd/back, z = up/down
	"RightShoulder Joint":	{ "max_torque": 80.0, "omega_max": Vector3(1.0, 3.0, 3.0), "error_scale": Vector3(1.0, 0.5, 0.5), "stiffness": Vector3(0.01, 0.01, 0.01), "damping": Vector3(20.0, 20.0, 20.0) }, # x = twist, y = forward / back, z = lateral raise
	"RightElbow Joint":		{ "max_torque": 60.0, "omega_max": Vector3(0.0, 3.0, 0.0), "error_scale": Vector3(1.0, 0.5, 1.0), "stiffness": Vector3(0.0, 0.01, 0.0), "damping": Vector3(0.0, 10.0, 0.0) },
	"RightWrist Joint":		{ "max_torque": 30.0, "omega_max": Vector3(0.5, 0.5, 1.2), "error_scale": Vector3(1.0, 1.0, 0.5), "stiffness": Vector3(0.01, 0.01, 0.01), "damping": Vector3(5.0, 5.0, 5.0) }, # x = supination / pronation, y = abduction / adduction, z = flexion/extension
	"LeftHip Joint":		{ "max_torque": 150.0, "omega_max": Vector3(4.0, 1.0, 4.0), "error_scale": Vector3(0.75, 1.0, 0.75), "stiffness": Vector3(20.0, 20.0, 20.0), "damping": Vector3(50.0, 50.0, 50.0) }, # Here y is the twist axis
	"LeftKnee Joint":		{ "max_torque": 120.0, "omega_max": Vector3(4.0, 0.0, 0.0), "error_scale": Vector3(0.5, 1.0, 1.0), "stiffness": Vector3(0.01, 0.0, 0.0), "damping": Vector3(30.0, 0.0, 0.0) },
	"LeftAnkle Joint":		{ "max_torque": 50.0, "omega_max": Vector3(1.0, 1.0, 0.0), "error_scale": Vector3(0.5, 0.5, 1.0), "stiffness": Vector3(0.001, 0.001, 0.0), "damping": Vector3(10.0, 10.0, 0.0) },
	"RightHip Joint":		{ "max_torque": 150.0, "omega_max": Vector3(4.0, 1.0, 4.0), "error_scale": Vector3(0.75, 1.0, 0.75), "stiffness": Vector3(20.0, 20.0, 20.0), "damping": Vector3(50.0, 50.0, 50.0) },
	"RightKnee Joint":		{ "max_torque": 120.0, "omega_max": Vector3(4.0, 0.0, 0.0), "error_scale": Vector3(0.5, 1.0, 1.0), "stiffness": Vector3(0.01, 0.0, 0.0), "damping": Vector3(30.0, 0.0, 0.0) },
	"RightAnkle Joint":		{ "max_torque": 50.0, "omega_max": Vector3(1.0, 1.0, 0.0), "error_scale": Vector3(0.5, 0.5, 1.0), "stiffness": Vector3(0.001, 0.001, 0.0), "damping": Vector3(10.0, 10.0, 0.0) },
}

func update_joint_torque_motors(target: Dictionary) -> void:
	if joints == null or target == null:
		return
	
	# World frame gravity compensations for root supported joints
	#var gravity_compensations = character.compute_gravity_compensation_for_joints(free_joint_names) #If I bring this back later for limbs that are not connected, I should refactor the function t accept joint nodes instead of the names
	
	# If I want to use this function I will have to:
	# 1. Compute gravity compensating torques (seems complicated)
	# OR
	# 2. Use effective inertia in velocity control (this is what the velocity motors do). Might be simpler than 1. (at least the concept is simpler) but still need to estimate effective inertia which might not be easy
	
	for joint in joints:
		var joint_name = joint.name
		if not target.has(joint_name):
			continue
		
		var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
		var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)
		
		var B_joint: Basis = joint.global_basis
		
		var Kd: Vector3 = JOINT_CONSTANTS[joint_name].Kd

		var res = get_PD_data(joint, target)
		var error_vec = res["error_vec"]
		var angular_velocity_world = res["angular_velocity"]
		
		var error_joint = B_joint.inverse() * error_vec
		
		var omega_max = JOINT_CONSTANTS[joint_name].omega_max
		var error_scale = JOINT_CONSTANTS[joint_name].error_scale
		var omega_target_joint := Vector3(
			omega_max.x * tanh(error_joint.x / error_scale.x),
			omega_max.y * tanh(error_joint.y / error_scale.y),
			omega_max.z * tanh(error_joint.z / error_scale.z)
		)
		
		# Deadzone might prevent micro-jitter:
		if abs(error_joint.x) < 0.01:
			omega_target_joint.x = 0.0
		if abs(error_joint.y) < 0.01:
			omega_target_joint.y = 0.0
		if abs(error_joint.z) < 0.01:
			omega_target_joint.z = 0.0
		
		# Control velocity
		var angular_velocity_joint = B_joint.inverse() * angular_velocity_world
		var omega_error = omega_target_joint - angular_velocity_joint
		var joint_velocity_correction_torque = Kd * omega_error
		var joint_velocity_correction_torque_world = B_joint * joint_velocity_correction_torque
		
		var target_torque_world = joint_velocity_correction_torque_world
		
		# Cap torque
		var max_torque = JOINT_CONSTANTS[joint_name].max_torque
		if target_torque_world.length() > max_torque:
			print("Joint: " + joint_name + " hit its torque limit")
			target_torque_world = target_torque_world.normalized() * max_torque
		
		node_a.external_torque -= target_torque_world
		node_b.external_torque += target_torque_world

func update_joint_velocity_motors(target: Dictionary) -> void:
	if joints == null or target == null:
		return
	
	var reaching_joints = get_reaching_joints()
	
	for joint in joints:
		if (not target.has(joint.name)) or (joint in reaching_joints):
			continue
		
		var omega_target_joint = compute_pose_driven_joint_velocity(joint, target)
		set_joint_velocity(joint, omega_target_joint)
		
		var vel_world = joint.global_transform.basis * omega_target_joint
		DebugDraw3D.draw_arrow(joint.global_transform.origin,joint.global_transform.origin + vel_world,Color.BLUE,0.01)

func compute_pose_driven_joint_velocity(joint: Generic6DOFJoint3D, target: Dictionary) -> Vector3:
	var B_joint: Basis = joint.global_basis
		
	var res = get_PD_data(joint, target)
	var error_vec = res["error_vec"]
	
	var error_joint = B_joint.inverse() * error_vec
	
	var omega_max = JOINT_CONSTANTS[joint.name].omega_max
	var error_scale = Vector3.ONE #JOINT_CONSTANTS[joint_name].error_scale
	var omega_target_joint := Vector3(
		-omega_max.x * tanh(error_joint.x / error_scale.x),
		-omega_max.y * tanh(error_joint.y / error_scale.y),
		-omega_max.z * tanh(error_joint.z / error_scale.z)
	)
	
	# Deadzone might prevent micro-jitter:
	if abs(error_joint.x) < joint_motor_velocity_deadzone:
		omega_target_joint.x = 0.0
	if abs(error_joint.y) < joint_motor_velocity_deadzone:
		omega_target_joint.y = 0.0
	if abs(error_joint.z) < joint_motor_velocity_deadzone:
		omega_target_joint.z = 0.0
	
	return omega_target_joint


func compute_desired_end_effector_velocity(current_pos: Vector3, target_pos: Vector3) -> Vector3:
	var pos_error = target_pos - current_pos
	var max_vel = 10.0
	var error_scale = 0.25
	var v_desired := Vector3(
			max_vel * tanh(pos_error.x / error_scale),
			max_vel * tanh(pos_error.y / error_scale),
			max_vel * tanh(pos_error.z / error_scale)
		)
	return v_desired

func compute_desired_end_effector_angular_velocity(current_rot: Quaternion, target_rot: Quaternion, gain := 1.0, max_ang_vel := 4.0, angle_scale := 1.0) -> Vector3:
	var q_err = target_rot * current_rot.inverse()

	if q_err.w < 0.0:
		q_err = -q_err

	var axis = q_err.get_axis()
	var angle = q_err.get_angle()

	var scaled_angle = angle * gain
	var ang_speed = max_ang_vel * tanh(scaled_angle / angle_scale)

	return axis * ang_speed

func compute_position_jacobian(reaching_joints: Array, end_effector_pos: Vector3) -> Dictionary:
	var J := {}

	for joint in reaching_joints:
		var basis = joint.global_transform.basis
		var joint_pos = joint.global_transform.origin
		var r = end_effector_pos - joint_pos

		J[joint.name] = {
			"x": basis.x.cross(r),
			"y": basis.y.cross(r),
			"z": basis.z.cross(r)
		}

	return J

func compute_rotation_jacobian(reaching_joints: Array) -> Dictionary:
	var J := {}

	for joint in reaching_joints:
		var basis = joint.global_transform.basis

		J[joint.name] = {
			"x": basis.x,
			"y": basis.y,
			"z": basis.z
		}

	return J

func compute_position_joint_velocities(J: Dictionary, v_desired: Vector3, gain := 1.0) -> Dictionary:
	var velocities := {}

	for joint_name in J.keys():
		var Ji = J[joint_name]

		velocities[joint_name] = Vector3(
			gain * Ji["x"].dot(v_desired),
			gain * Ji["y"].dot(v_desired),
			gain * Ji["z"].dot(v_desired)
		)

	return velocities

func compute_rotation_joint_velocities(J: Dictionary, w_desired: Vector3, gain := 1.0) -> Dictionary:
	var velocities := {}

	for joint_name in J.keys():
		var Ji = J[joint_name]

		velocities[joint_name] = Vector3(
			gain * Ji["x"].dot(w_desired),
			gain * Ji["y"].dot(w_desired),
			gain * Ji["z"].dot(w_desired)
		)

	return velocities

func enable_velocity_motors(joints_to_enable: Array[Generic6DOFJoint3D]) -> void:
	print("Enabling:")
	for joint in joints_to_enable:
		print(joint.name)
		var max_torque = JOINT_CONSTANTS[joint.name].max_torque
		
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)

		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)

func disable_velocity_motors(joints_to_disable: Array[Generic6DOFJoint3D]) -> void:
	for joint in joints_to_disable:
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)


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
	
	SignalBus.generic_value_changed.connect(_on_generic_value_changed)

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
	var reaching_joints = get_reaching_joints()
	
	if reaching_left_hand:
		reaching_left_hand = false
		if not reach_pose:
			disable_velocity_motors(reaching_joints)
	else:
		reaching_left_hand = true
		reaching_joints = get_reaching_joints()
		enable_velocity_motors(reaching_joints)

func _on_reach_right_hand() -> void:
	var reaching_joints = get_reaching_joints()
	
	if reaching_right_hand:
		reaching_right_hand = false
		if not reach_pose:
			disable_velocity_motors(reaching_joints)
	else:
		reaching_right_hand = true
		reaching_joints = get_reaching_joints()
		enable_velocity_motors(reaching_joints)

func _on_reach_left_foot() -> void:
	var reaching_joints = get_reaching_joints()
	
	if reaching_left_foot:
		reaching_left_foot = false
		if not reach_pose:
			disable_velocity_motors(reaching_joints)
	else:
		reaching_left_foot = true
		reaching_joints = get_reaching_joints()
		enable_velocity_motors(reaching_joints)

func _on_reach_right_foot() -> void:
	var reaching_joints = get_reaching_joints()
	
	if reaching_right_foot:
		reaching_right_foot = false
		if not reach_pose:
			disable_velocity_motors(reaching_joints)
	else:
		reaching_right_foot = true
		reaching_joints = get_reaching_joints()
		enable_velocity_motors(reaching_joints)

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
	print(res)

func _on_reach_pose() -> void:
	if reach_pose:
		print("Disabling motors")
		reach_pose = false
		
		#These lines are currently for joint velocity motors specifically
		disable_velocity_motors(joints)
		#apply_joint_params(initial_joint_params)
		
		var reaching_joints = get_reaching_joints()
		enable_velocity_motors(reaching_joints)
	else:
		print("Driving motors to reach target pose")
		reach_pose = true
		
		#These lines are currently for joint velocity motors specifically
		enable_velocity_motors(joints)
		#apply_joint_params(JOINT_CONSTANTS)

func _on_generic_value_changed(value: float) -> void:
	joint_motor_velocity_deadzone = value
