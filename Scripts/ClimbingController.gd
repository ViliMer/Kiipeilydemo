class_name ClimbingController
extends Node

var character: CharacterBody3D
var skeleton: Skeleton3D

var left_hand_target: Node3D
var right_hand_target: Node3D
var left_foot_target: Node3D
var right_foot_target: Node3D

var bone_names = {
	"root":				"Physical Bone Root",
	"hips":				"Physical Bone Hips",
	"spine":			"Physical Bone Spine",
	"chest":			"Physical Bone Chest",
	"upper_chest":		"Physical Bone UpperChest",
	"Neck":				"Physical Bone Neck",
	"left_upper_arm":	"Physical Bone LeftUpperArm",
	"left_lower_arm":	"Physical Bone LeftLowerArm",
	"left_hand":		"Physical Bone LeftHand",
	"right_upper_arm":	"Physical Bone RightUpperArm",
	"right_lower_arm":	"Physical Bone RightLowerArm",
	"right_hand":		"Physical Bone RightHand",
	"left_upper_leg":	"Physical Bone LeftUpperLeg",
	"left_lower_leg":	"Physical Bone LeftLowerLeg",
	"left_foot":		"Physical Bone LeftFoot",
	"right_upper_leg":	"Physical Bone RightUpperLeg",
	"right_lower_leg":	"Physical Bone RightLowerLeg",
	"right_foot":		"Physical Bone RightFoot",
}

var attached_holds = {
	"lh": null,
	"rh": null,
	"lf": null,
	"rf": null
}

func init(character_ref: CharacterBody3D) -> void:
	character = character_ref
	skeleton = character.skeleton

	left_hand_target = character.get_node("Left Hand Target")
	right_hand_target = character.get_node("Right Hand Target")
	left_foot_target = character.get_node("Left Foot Target")
	right_foot_target = character.get_node("Right Foot Target")

func enter_climb(start_holds: Dictionary, start_pos: Node3D):
	
	if character.collision_shape:
		character.collision_shape.disabled = true
	
	character.velocity = Vector3.ZERO
	if character.anim:
		character.anim.play("A_TPose")
		character.anim.seek(0.0, true)
		character.anim.play("hand_pose")
		character.anim.seek(0.0, true)
		character.anim.stop()
	
	await move_character_to_start(start_holds, start_pos, 1.0)
	attached_holds = start_holds
	
	if character.left_hand_ik:
		character.left_hand_ik.start(true)
		_add_limb_lock("lh", start_holds["lh"])
	if character.right_hand_ik:
		character.right_hand_ik.start(true)
		_add_limb_lock("rh", start_holds["rh"])
	if character.left_foot_ik:
		character.left_foot_ik.start(true)
		_add_limb_lock("lf", start_holds["lf"])
	if character.right_foot_ik:
		character.right_foot_ik.start(true)
		_add_limb_lock("rf", start_holds["rf"])
	
	character.bone_sim.active = true
	character.bone_sim.physical_bones_start_simulation()

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

func _update_ik_targets(start_holds: Dictionary) -> void:
	if start_holds["lh"]:
		left_hand_target.global_transform = start_holds["lh"].get_hand_grab_transform()
	if start_holds["rh"]:
		right_hand_target.global_transform = start_holds["rh"].get_hand_grab_transform()
	if start_holds["lf"]:
		left_foot_target.global_transform = start_holds["lf"].get_foot_grab_transform()
	if start_holds["rf"]:
		right_foot_target.global_transform = start_holds["rf"].get_foot_grab_transform()


func _add_limb_lock(code: String, hold: Node3D) -> void:

	var bone_name: String
	match code:
		"lh": bone_name = "Physical Bone LeftHand"
		"rh": bone_name = "Physical Bone RightHand"
		"lf": bone_name = "Physical Bone LeftFoot"
		"rf": bone_name = "Physical Bone RightFoot"
		_: return
	
	var physical_bone: PhysicalBone3D = character.bone_sim.find_child(bone_name)
	
	physical_bone.axis_lock_linear_x = true
	physical_bone.axis_lock_linear_y = true
	physical_bone.axis_lock_linear_z = true
	
	if physical_bone == null:
		push_warning("No physical bone found for %s" % bone_name)
		return

func _remove_limb_locks() -> void:

	var bones = [
		"Physical Bone LeftHand",
		"Physical Bone RightHand",
		"Physical Bone LeftFoot",
		"Physical Bone RightFoot"
	]
	
	for bone in bones:
		var physical_bone: PhysicalBone3D = character.bone_sim.find_child(bone)
		physical_bone.axis_lock_linear_x = false
		physical_bone.axis_lock_linear_y = false
		physical_bone.axis_lock_linear_z = false


func update_climbing(delta) -> void:
	# Basic version: simply hold onto the targets
	# IK solvers will take care of posing the character.
	pass

func exit_climb():
	attached_holds = {"lh":null, "rh":null, "lf":null, "rf":null}
	_remove_limb_locks()
