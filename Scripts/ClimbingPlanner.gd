class_name ClimbingPlanner
extends Node

var character: CharacterBody3D

# This dictionary defines the bone indices of the bones that each joint connects
var IK_skeleton_joint_definitions = {
	"Spine Joint": {"bone_a": 1, "bone_b": 2},
	"LowerChest Joint": {"bone_a": 2, "bone_b": 3},
	"Chest Joint": {"bone_a": 3, "bone_b": 4},
	"LeftUpperChest Joint": {"bone_a": 4, "bone_b": 7},
	"LeftShoulder Joint": {"bone_a": 7, "bone_b": 8},
	"LeftElbow Joint": {"bone_a": 8, "bone_b": 9},
	"LeftWrist Joint": {"bone_a": 9, "bone_b": 10},
	"RightUpperChest Joint": {"bone_a": 4, "bone_b": 26},
	"RightShoulder Joint": {"bone_a": 26, "bone_b": 27},
	"RightElbow Joint": {"bone_a": 27, "bone_b": 28},
	"RightWrist Joint": {"bone_a": 28, "bone_b": 29},
	"LeftHip Joint": {"bone_a": 1, "bone_b": 45},
	"LeftKnee Joint": {"bone_a": 45, "bone_b": 46},
	"LeftAnkle Joint": {"bone_a": 46, "bone_b": 47},
	"RightHip Joint": {"bone_a": 1, "bone_b": 49},
	"RightKnee Joint": {"bone_a": 49, "bone_b": 50},
	"RightAnkle Joint": {"bone_a": 50, "bone_b": 51},
}

func init(character_ref: CharacterBody3D) -> void:
	character = character_ref


# This function will return a dictionary of target holds for the next move. It should:
#Take the current attachment state (which limbs are grabbing holds)
#Precomputes reachability (limb lengths, max distance envelopes, feasible angles)
#Evaluates candidate moves for both hands and feet
#Scores moves based on reachability, balance, and stability
#Chooses a target “next configuration”—the next combination of holds for the limbs
func plan_climb(holds: Array[Hold], attached_holds: Dictionary, character_reaches: Dictionary, target_holds = null) -> Dictionary:
	if target_holds:
		return target_holds
	return {}

#Given a desired configuration (hand/foot assignments), this function should compute:
#Desired hip (COM) position
#Body/chest rotation
#Arm/leg targets
#Uses SkeletonIK3D nodes only to solve limb joint angles and validate poses
#Converts solved IK angles into target joint angles for the physics joint motors
#And return this target pose
func generate_target_pose(target_holds: Dictionary) -> void:
	pass

func plan_start_pose(target_holds: Dictionary) -> Dictionary:
	
	# Initial guess for the hip starting position:
	var target_pos = compute_hold_centroid(target_holds)
	var forward = _compute_forward_from_wall(target_holds)
	var up = _compute_up_direction(target_holds, forward)
	target_pos = target_pos - 0.5 * forward - 0.2 * up

	var hip_target_transform = compute_hip_transform(target_holds, target_pos)
	
	var hip_idx = 1
	var hip_pose_skel = character.IK_skeleton.get_bone_global_pose(hip_idx)
	var root_target_transform = hip_target_transform * hip_pose_skel.affine_inverse()
	
	character.IK_character_node.global_transform = root_target_transform
	character.IK_character_node.visible = true
	
	# Next run IK to reach target holds with limbs:
	character.right_hand_ik.active = true
	character.left_hand_ik.active = true
	character.right_foot_ik.active = true
	character.left_foot_ik.active = true
	
	# Move start pos downward (first implementation for testing)
	var last_good_transform = root_target_transform
	var new_transform = root_target_transform
	var errors := await compute_ik_errors(target_holds)
	
	# Instead of just moving down, we should use some heuristic to find a "good" position
	while errors["total"] < 0.03:
		last_good_transform = new_transform
		new_transform = Transform3D(last_good_transform.basis, last_good_transform.origin - Vector3(0, 0.01, 0))
		character.IK_character_node.global_transform = new_transform
		errors = await compute_ik_errors(target_holds)
		#print("Total IK error: %f" % errors["total"])
	
	character.IK_character_node.global_transform = last_good_transform
	
	var result := {}
	result["bone_transforms"] = character.IK_modified_bone_transforms
	result["global_transform"] = last_good_transform
	
	return result

# Need to figure this out. Might need to use the physical bone <-> skeleton bone transformations. Also need to check out how the angles are computed in save_joint_angles
func get_IK_skeleton_joint_angles() -> Dictionary:
	var result = {}
	var skel: Skeleton3D = character.IK_skeleton
	
	for joint_name in IK_skeleton_joint_definitions:
		var def = IK_skeleton_joint_definitions[joint_name]
		var bone_b_idx: int = def["bone_b"]
		var bone_a_idx: int = def["bone_a"]
		
		if not character.IK_modified_bone_transforms.has(bone_b_idx):
			continue
		if not character.IK_modified_bone_transforms.has(bone_a_idx):
			continue
		
		# --- IK skeleton bone poses in skeleton space ---
		var Tb_ik: Transform3D = character.IK_modified_bone_transforms[bone_b_idx]
		var Ta_ik: Transform3D = character.IK_modified_bone_transforms[bone_a_idx]
		
		# --- Offsets: skeleton → physical bone frames (skeleton space) ---
		var bone_b_name := skel.get_bone_name(bone_b_idx)
		var bone_a_name := skel.get_bone_name(bone_a_idx)
		
		var Ob: Transform3D = character.bone_rotation_offsets[bone_b_name]
		var Oa: Transform3D = character.bone_rotation_offsets[bone_a_name]
		
		# Convert IK bones into "physical-bone-equivalent" space
		var Tb_phys := Tb_ik * Ob
		var Ta_phys := Ta_ik * Oa
		
		# --- Relative rotation of child vs parent (in world space) ---
		var rel_basis := Ta_phys.basis.inverse() * Tb_phys.basis
		
		# --- Express in joint local space ---
		var joint: Generic6DOFJoint3D = character.joints[joint_name]
		var B_joint: Basis = joint.basis
		
		rel_basis = B_joint.inverse() * rel_basis * B_joint
		var q_joint = rel_basis.get_rotation_quaternion().normalized()
		
		# Store rotation quaternion in joint space
		result[joint_name] = q_joint
		
	return result

func _collect_active_shapes(attached_holds: Dictionary) -> Array[Node3D]:
	var shapes: Array[Node3D] = []

	if attached_holds.lh != null:
		shapes.append(character.left_hand_target.get_node("ValidHipPosShape"))

	if attached_holds.rh != null:
		shapes.append(character.right_hand_target.get_node("ValidHipPosShape"))

	if attached_holds.lf != null:
		shapes.append(character.left_foot_target.get_node("ValidHipPosShape"))

	if attached_holds.rf != null:
		shapes.append(character.right_foot_target.get_node("ValidHipPosShape"))

	return shapes

func _sdf_sphere(local_p: Vector3, radius: float) -> float:
	return local_p.length() - radius

func _sdf_shape(p_world: Vector3, shape_root: Node3D) -> float:
	var d_union := INF
	var d_sub := -INF

	for child in shape_root.get_children():
		if child is CSGSphere3D:
			var local_p = child.global_transform.affine_inverse() * p_world
			var d := _sdf_sphere(local_p, child.radius)

			if child.operation == CSGShape3D.OPERATION_SUBTRACTION:
				d_sub = max(d_sub, -d)
			else:
				d_union = min(d_union, d)

	return max(d_union, d_sub)

func _sdf_intersection(p: Vector3, shapes: Array[Node3D]) -> float:
	var d := -INF

	for shape in shapes:
		d = max(d, _sdf_shape(p, shape))

	return d

func _find_closest_inside(
	start: Vector3,
	shapes: Array[Node3D],
	max_steps := 64,
	epsilon := 0.01
) -> Variant:
	var p := start

	for i in max_steps:
		var d := _sdf_intersection(p, shapes)

		# We are inside (or close enough)
		if d <= epsilon:
			return p

		# Numerical gradient (central differences)
		var grad := Vector3(
			_sdf_intersection(p + Vector3(epsilon, 0, 0), shapes)
				- _sdf_intersection(p - Vector3(epsilon, 0, 0), shapes),
			_sdf_intersection(p + Vector3(0, epsilon, 0), shapes)
				- _sdf_intersection(p - Vector3(0, epsilon, 0), shapes),
			_sdf_intersection(p + Vector3(0, 0, epsilon), shapes)
				- _sdf_intersection(p - Vector3(0, 0, epsilon), shapes)
		)

		var grad_len := grad.length()
		if grad_len < 0.00001:
			# Flat region or numerical issue → treat as failure
			print("Entered flat region in _find_closest_inside()")
			return null

		# Move toward the intersection
		p -= (grad / grad_len) * d

	print("Did not reach intersection in _find_closest_inside()")
	return null


func compute_hip_position(
	attached_holds: Dictionary,
	target_pos: Vector3
) -> Variant:
	var shapes := _collect_active_shapes(attached_holds)
	if shapes.is_empty():
		return target_pos
	
	var hip_pos = _find_closest_inside(target_pos, shapes)
	if hip_pos == null:
		return null

	return hip_pos

func compute_hold_centroid(holds: Dictionary) -> Vector3:
	var sum := Vector3.ZERO
	var count := 0
	for hold in holds.values():
		var pos = hold.get_grab_transform().origin
		sum += pos
		count += 1

	if count == 0:
		push_error("No holds in compute_hold_centroid")

	return sum / count


func compute_hip_transform(
	attached_holds: Dictionary,
	target_pos: Vector3
) -> Variant:
	var hip_pos = compute_hip_position(attached_holds, target_pos)
	
	var forward = _compute_forward_from_wall(attached_holds)
	if forward == null:
		return null

	var up := _compute_up_direction(
		attached_holds,
		forward
	)

	var right := up.cross(forward).normalized()
	up = forward.cross(right).normalized()

	var basis := Basis(right, up, forward)
	return Transform3D(basis, hip_pos)

func _compute_forward_from_wall(
	attached_holds: Dictionary
) -> Variant:
	var n := Vector3.ZERO
	var count := 0

	for key in attached_holds.keys():
		var hold = attached_holds[key]
		if hold == null:
			continue

		# Hold local +Z is wall normal
		var hold_normal = hold.global_transform.basis.z
		n += hold_normal
		count += 1

	if count == 0 or n.length() < 0.0001:
		return null

	# Face INTO the wall
	return -n.normalized()

func _compute_up_direction(
	attached_holds: Dictionary,
	forward: Vector3
) -> Vector3:
	var raw_up := _compute_raw_up_direction(attached_holds)

	# Project to be orthogonal to forward
	var f := forward.normalized()
	var up := raw_up - f * raw_up.dot(f)

	if up.length() < 0.001:
		return Vector3.UP

	return up.normalized()


func _compute_raw_up_direction(
	attached_holds: Dictionary
) -> Vector3:
	var up := Vector3.ZERO
	var total_weight := 0.0
	
	var centroid: Vector3 = compute_hold_centroid(attached_holds)

	for key in attached_holds.keys():
		var hold = attached_holds[key]
		if hold == null:
			continue

		var attach_pos: Vector3
		var weight: float

		match key:
			"lh", "rh":
				attach_pos = hold.get_grab_transform().origin
				weight = 1.0
			"lf", "rf":
				attach_pos = hold.get_grab_transform().origin
				weight = -0.8
			_:
				continue

		var dir := attach_pos - centroid
		if dir.length() < 0.001:
			continue

		up += dir.normalized() * weight
		total_weight += abs(weight)

	if total_weight < 0.001 or up.length() < 0.001:
		return Vector3.UP

	return up.normalized()


func compute_ik_errors(target_holds: Dictionary) -> Dictionary:

	var res := {}
	var total := 0.0

	for i in 2:
		await get_tree().process_frame

	var IK_pose = character.IK_modified_bone_transforms

	for code in target_holds.keys():

		var hold = target_holds[code]
		var target_pos = hold.get_grab_transform().origin

		var bone_idx: int = _get_limb_bone_index(code)

		var end_effector_transform: Transform3D = IK_pose[bone_idx]
		var end_effector_global = character.IK_skeleton.global_transform * end_effector_transform
		
		if code == "rh" or code == "lh":
			end_effector_global = get_hand_palm_transform(bone_idx)
		
		var end_effector_pos = end_effector_global.origin
		var error = end_effector_pos.distance_to(target_pos)
		
		res[code] = error
		total += error
	
	res["total"] = total
	return res

func _get_limb_bone_index(code) -> int:
	var bone_idx: int
	match code:
			"rh":
				bone_idx = 29
			"lh":
				bone_idx = 10
			"rf":
				bone_idx = 52
			"lf":
				bone_idx = 48
			_:
				push_error("Invalid code in _get_limb_bone_index")
	return bone_idx

func get_hand_palm_transform(bone_idx: int) -> Transform3D:
	var IK_pose = character.IK_modified_bone_transforms
	var hand_transform: Transform3D = IK_pose[bone_idx]
	
	var wrist_pos = hand_transform.origin
	var wrist_basis = hand_transform.basis
	
	# Expects y to be local up direction
	var extension = wrist_basis.y * 0.12
	
	var end_effector_global = character.IK_skeleton.global_transform * Transform3D(hand_transform.basis, wrist_pos + extension)
	
	return end_effector_global
