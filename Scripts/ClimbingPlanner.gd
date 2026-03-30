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

func plan_next_pose(attachments: Dictionary, target_holds: Dictionary) -> Dictionary:
	
	var current_holds: Dictionary = {}
	var holds_to_consider: Dictionary = {}
	var free_limbs: Array[String] = []
	
	for code in attachments.keys():
		var attachment = attachments[code]
		
		if attachment:
			current_holds[code] = attachment.hold
			holds_to_consider[code] = attachment.hold
		elif target_holds[code]:
			holds_to_consider[code] = target_holds[code]
		else:
			free_limbs.append(code)
	
	var target_pos = compute_hold_centroid(holds_to_consider)
	
	var hip_target_transform = compute_hip_transform(holds_to_consider, target_pos)
	
	var hip_idx = 1
	var hip_pose_skel = character.IK_skeleton.get_bone_global_pose(hip_idx)
	var initial_root_target_transform = hip_target_transform * hip_pose_skel.affine_inverse()
	
	var new_transform = await iterate_target_pose(initial_root_target_transform, current_holds, target_holds, holds_to_consider, free_limbs)
	
	character.IK_character_node.global_transform = new_transform
	
	var result = {
		"bone_transforms": character.IK_modified_bone_transforms,
		"global_transform": new_transform
	}
	
	return result

func iterate_target_pose(initial_transform: Transform3D, current_holds: Dictionary, target_holds: Dictionary, holds_to_consider: Dictionary, free_limbs: Array[String]) -> Transform3D:
	var best_transform := initial_transform
	var best_score: float = INF
	
	#var yaw_samples = [-90, -60, -30, 0, 30, 60, 90]
	var yaw_samples = [0]
	
	#var pitch_samples = [-20, -10, 0, 10, 20]
	var pitch_samples = [0]
	
	var max_radius := 0.3
	var pos_samples := 100
	
	# Evaluate initial pos first
	character.IK_character_node.global_transform = initial_transform
	
	for limb_code in free_limbs:
		balance_with_free_limb(limb_code)
	
	var errors := await compute_ik_errors(holds_to_consider)
	
	var cur_holds_error = 0.0
	var number_of_current_holds = current_holds.size()
	for code in current_holds.keys():
		cur_holds_error += errors[code]
	
	var reaching_limbs_error: float = errors["total"] - cur_holds_error
	
	if errors["total"] < 0.0125 * number_of_current_holds:
		best_score = 0.0
		best_score += reaching_limbs_error * 2
		best_score += heuristic_hips_close_to_wall(initial_transform, holds_to_consider)
		best_score += heuristic_COM_supported_wall_plane(current_holds)
	
	for i in pos_samples:
		
		var offset := Vector3(
			randf_range(-max_radius, max_radius),
			randf_range(-max_radius, max_radius),
			randf_range(-max_radius, max_radius)
		)
		
		var pos := initial_transform.origin + offset
		
		for yaw in yaw_samples:
			for pitch in pitch_samples:
				
				var basis := sample_oriented_basis(initial_transform.basis, yaw, pitch)
				var candidate := Transform3D(basis, pos)
				
				character.IK_character_node.global_transform = candidate
				
				for limb_code in free_limbs:
					balance_with_free_limb(limb_code)
				
				errors = await compute_ik_errors(holds_to_consider)
				
				cur_holds_error = 0.0
				for code in current_holds.keys():
					cur_holds_error += errors[code]
				
				reaching_limbs_error = errors["total"] - cur_holds_error
				
				if errors["total"] > 0.0125 * number_of_current_holds:
					continue
				
				var score: float = 0.0
				score += reaching_limbs_error * 2
				score += heuristic_hips_close_to_wall(candidate, holds_to_consider)
				score += heuristic_COM_supported_wall_plane(current_holds)
				
				if score < best_score:
					best_score = score
					best_transform = candidate
	
	return best_transform

func plan_start_pose(target_holds: Dictionary) -> Dictionary:
	
	# Initial guess for the hip starting position:
	var target_pos = compute_hold_centroid(target_holds)
	var forward = _compute_forward_from_wall(target_holds)
	var up = _compute_up_direction(target_holds, forward)
	target_pos = target_pos - 0.5 * forward - 0.2 * up

	var hip_target_transform = compute_hip_transform(target_holds, target_pos)
	
	var hip_idx = 1
	var hip_pose_skel = character.IK_skeleton.get_bone_global_pose(hip_idx)
	var initial_root_target_transform = hip_target_transform * hip_pose_skel.affine_inverse()
	
	character.IK_character_node.visible = true # Maybe move this so that it happens after iterating root transform (?)
	
	# Next run IK to reach target holds with limbs:
	character.right_hand_ik.active = true
	character.left_hand_ik.active = true
	character.right_foot_ik.active = true
	character.left_foot_ik.active = true
	
	var new_transform = await iterate_start_pose(initial_root_target_transform, target_holds)
	
	character.IK_character_node.global_transform = new_transform
	
	var result: Dictionary = {
		"bone_transforms": character.IK_modified_bone_transforms,
		"global_transform": new_transform
	}
	
	return result


# Maybe rotation should be decided another way, so we can only sample position. Only position sampling seems fast enough, rotation sampling takes significant time.
func iterate_start_pose(initial_transform: Transform3D, holds: Dictionary) -> Transform3D:
	var best_transform := initial_transform
	var best_score: float = INF
	
	#var yaw_samples = [-90, -60, -30, 0, 30, 60, 90]
	var yaw_samples = [0]
	
	#var pitch_samples = [-20, -10, 0, 10, 20]
	var pitch_samples = [0]
	
	var max_radius := 0.3
	var pos_samples := 100
	
	# Evaluate initial pos first
	character.IK_character_node.global_transform = initial_transform
	
	var errors := await compute_ik_errors(holds)
	
	if errors["total"] < 0.05:
		best_score = 0.0
		best_score += heuristic_hands_straight()
		best_score += heuristic_hips_close_to_wall(initial_transform, holds)
		best_score += heuristic_COM_supported_wall_plane(holds)
	
	for i in pos_samples:
		
		var offset := Vector3(
			randf_range(-max_radius, max_radius),
			randf_range(-max_radius, max_radius),
			randf_range(-max_radius, max_radius)
		)
		
		#if offset.length() > max_radius:
		#	continue
		
		var pos := initial_transform.origin + offset
		
		for yaw in yaw_samples:
			for pitch in pitch_samples:
				
				var basis := sample_oriented_basis(initial_transform.basis, yaw, pitch)
				var candidate := Transform3D(basis, pos)
				
				character.IK_character_node.global_transform = candidate
				
				errors = await compute_ik_errors(holds)
				if errors["total"] > 0.03:
					continue
				
				var score: float = 0.0
				score += heuristic_hands_straight()
				score += heuristic_hips_close_to_wall(candidate, holds)
				score += heuristic_COM_supported_wall_plane(holds)
				
				if score < best_score:
					best_score = score
					best_transform = candidate
	
	return best_transform

# Helper function to generate rotated hip/root basis based on yaw and pitch
func sample_oriented_basis(base_basis: Basis, yaw_deg: float, pitch_deg: float) -> Basis:
	var yaw = deg_to_rad(yaw_deg)
	var pitch = deg_to_rad(pitch_deg)
	
	var yaw_rot = Basis(Vector3.UP, yaw)
	var pitch_rot = Basis(Vector3.RIGHT, pitch)
	
	return base_basis * yaw_rot * pitch_rot

func heuristic_hands_straight() -> float:
	var res = 0.0
	
	var IK_joint_angles: Dictionary = get_IK_skeleton_joint_angles()
	var left_elbow_q: Quaternion = IK_joint_angles["LeftElbow Joint"]
	var right_elbow_q: Quaternion = IK_joint_angles["RightElbow Joint"]
	var left_wrist_q: Quaternion = IK_joint_angles["LeftWrist Joint"]
	var right_wrist_q: Quaternion = IK_joint_angles["RightWrist Joint"]
	
	var left_elbow_angle := left_elbow_q.get_euler().y # radians
	var right_elbow_angle := right_elbow_q.get_euler().y # radians
	var left_wrist_angle := left_wrist_q.get_euler().z # radians
	var right_wrist_angle := right_wrist_q.get_euler().z # radians
	
	res -= abs(left_elbow_angle)
	res -= abs(right_elbow_angle)
	res -= abs(left_wrist_angle)
	res -= abs(right_wrist_angle)
	
	return res

func heuristic_hips_close_to_wall(current_root_transform: Transform3D, holds: Dictionary) -> float:
	var hip_pos := current_root_transform.origin
	
	var wall_normal: Vector3 = -_compute_forward_from_wall(holds)
	var wall_point: Vector3 = compute_hold_centroid(holds)
	
	var signed_distance_from_wall: float = wall_normal.dot(hip_pos - wall_point)

	return signed_distance_from_wall

func heuristic_COM_supported_wall_plane(holds: Dictionary) -> float:
	var res := 0.0

	# Build wall coordinate frame
	var forward: Vector3 = _compute_forward_from_wall(holds)
	var wall_normal: Vector3 = -forward
	
	var up: Vector3 = _compute_up_direction(holds, forward)
	var right: Vector3 = up.cross(wall_normal).normalized()

	#up = wall_normal.cross(right).normalized()

	# Compute COM world position
	var skel: Skeleton3D = character.IK_skeleton
	
	var hip_bone_idx := 1
	var hip_pos_skel: Vector3 = skel.get_bone_global_pose(hip_bone_idx).origin
	var hip_pos_world: Vector3 = skel.global_transform * hip_pos_skel
	
	var COM_world: Vector3 = hip_pos_world + character.COM_relative_to_hip_bone

	# Collect support points
	var support_points := []

	for code in holds.keys():
		var hold = holds[code]
		support_points.append(hold.get_grab_point_transform().origin)

	if support_points.size() < 2:
		return res

	# Project points to wall plane
	var points2 := []

	for p in support_points:
		points2.append(Vector2(p.dot(right), p.dot(up)))

	var COM2 := Vector2(COM_world.dot(right), COM_world.dot(up))

	# Compute support polygon
	var hull := Geometry2D.convex_hull(points2)

	if hull.size() < 2:
		return res

	# Compute distance to polygon edges
	var inside := Geometry2D.is_point_in_polygon(COM2, hull)

	var min_dist := INF

	for i in hull.size():
		var a: Vector2 = hull[i]
		var b: Vector2 = hull[(i + 1) % hull.size()]

		var ab := b - a
		var t: float = clamp((COM2 - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
		var closest := a + ab * t

		var d := COM2.distance_to(closest)

		min_dist = min(min_dist, d)

	# Score
	if inside:
		res -= min_dist        # deeper inside = better
	else:
		res += min_dist * 2.0  # outside = bad

	return res


# WIP: This function should move IK target node corresponding to limb code, so that the limb helps with balance. It should e.g. do flagging or move COM
func balance_with_free_limb(limb_code: String) -> void:
	pass

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

func _collect_active_shapes(holds: Dictionary) -> Array[Node3D]:
	var shapes: Array[Node3D] = []

	if holds.has("lh"):
		shapes.append(character.left_hand_target.get_node("ValidHipPosShape"))

	if holds.has("rh"):
		shapes.append(character.right_hand_target.get_node("ValidHipPosShape"))

	if holds.has("lf"):
		shapes.append(character.left_foot_target.get_node("ValidHipPosShape"))

	if holds.has("rf"):
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
	holds: Dictionary,
	target_pos: Vector3
) -> Variant:
	var shapes := _collect_active_shapes(holds)
	if shapes.is_empty():
		return target_pos
	
	var hip_pos = _find_closest_inside(target_pos, shapes)
	if hip_pos == null:
		return target_pos

	return hip_pos

func compute_hold_centroid(holds: Dictionary) -> Vector3:
	var sum := Vector3.ZERO
	var count := 0
	for hold in holds.values():
		var pos = hold.get_grab_point_transform().origin
		sum += pos
		count += 1

	if count == 0:
		push_error("No holds in compute_hold_centroid")

	return sum / count


func compute_hip_transform(
	holds: Dictionary,
	target_pos: Vector3
) -> Variant:
	var hip_pos = compute_hip_position(holds, target_pos)
	
	var forward = _compute_forward_from_wall(holds)
	if forward == null:
		return null

	var up := _compute_up_direction(
		holds,
		forward
	)

	var right := up.cross(forward).normalized()
	up = forward.cross(right).normalized()

	var basis := Basis(right, up, forward)
	return Transform3D(basis, hip_pos)

func _compute_forward_from_wall(
	holds: Dictionary
) -> Variant:
	var n := Vector3.ZERO
	var count := 0

	for key in holds.keys():
		var hold = holds[key]
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
	holds: Dictionary,
	forward: Vector3
) -> Vector3:
	var raw_up := _compute_raw_up_direction(holds)

	# Project to be orthogonal to forward
	var f := forward.normalized()
	var up := raw_up - f * raw_up.dot(f)

	if up.length() < 0.001:
		return Vector3.UP

	return up.normalized()


func _compute_raw_up_direction(
	holds: Dictionary
) -> Vector3:
	var up := Vector3.ZERO
	var total_weight := 0.0
	
	var centroid: Vector3 = compute_hold_centroid(holds)

	for key in holds.keys():
		var hold = holds[key]
		if hold == null:
			continue

		var attach_pos: Vector3
		var weight: float

		match key:
			"lh", "rh":
				attach_pos = hold.get_grab_point_transform().origin
				weight = 1.0
			"lf", "rf":
				attach_pos = hold.get_grab_point_transform().origin
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

	#for i in 2:
	await get_tree().process_frame

	var IK_pose = character.IK_modified_bone_transforms

	for code in target_holds.keys():

		var hold = target_holds[code]
		var target_pos = hold.get_grab_point_transform().origin

		var bone_idx: int = get_limb_bone_index(code)

		var end_effector_transform: Transform3D = IK_pose[bone_idx]
		var end_effector_global: Transform3D
		
		if code == "rh" or code == "lh":
			end_effector_global = character.IK_skeleton.global_transform * get_hand_palm_transform(end_effector_transform)
		else:
			end_effector_global = character.IK_skeleton.global_transform * end_effector_transform
		
		var end_effector_pos = end_effector_global.origin
		var error = end_effector_pos.distance_to(target_pos)
		
		res[code] = error
		total += error
	
	res["total"] = total
	return res

func get_limb_bone_index(code) -> int:
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

func get_hand_palm_transform(hand_transform: Transform3D) -> Transform3D:
	var wrist_pos = hand_transform.origin
	var wrist_basis = hand_transform.basis
	
	# Expects y to be local up direction
	var extension = wrist_basis.y * 0.12
	
	var end_effector_global = Transform3D(hand_transform.basis, wrist_pos + extension)
	
	return end_effector_global
