class_name ClimbingController
extends Node

var character: CharacterBody3D
var skeleton: Skeleton3D

var left_hand_target: Node3D
var right_hand_target: Node3D
var left_foot_target: Node3D
var right_foot_target: Node3D

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
	"""
	start_holds = {
		"lh": Hold3D,
		"rh": Hold3D,
		"lf": Hold3D,
		"rf": Hold3D,
	}
	"""
	
	character.global_transform = start_pos.global_transform
	attached_holds = start_holds

	# Place IK targets exactly on holds
	if start_holds["lh"]:
		left_hand_target.global_transform = start_holds["lh"].get_grab_transform()
	if start_holds["rh"]:
		right_hand_target.global_transform = start_holds["rh"].get_grab_transform()
	if start_holds["lf"]:
		left_foot_target.global_transform = start_holds["lf"].get_grab_transform()
	if start_holds["rf"]:
		right_foot_target.global_transform = start_holds["rf"].get_grab_transform()

	# You will later apply pelvis physics here
	# For now we freeze character movement
	character.velocity = Vector3.ZERO
	# Disable animation so bones can move freely
	if character.anim:
		character.anim.play("A_TPose")
		character.anim.seek(0.0, true)
		character.anim.play("hand_pose")
		character.anim.seek(0.0, true)
		character.anim.stop()
		
	if character.collision_shape:
		character.collision_shape.disabled = true

	# Wait 1 frame so animation pose is applied before physics begins
	await get_tree().process_frame
	
	if character.left_hand_ik:
		character.left_hand_ik.start(true)
		_add_limb_lock("lh")
	if character.right_hand_ik:
		character.right_hand_ik.start(true)
		_add_limb_lock("rh")
	if character.left_foot_ik:
		character.left_foot_ik.start(true)
		_add_limb_lock("lf")
	if character.right_foot_ik:
		character.right_foot_ik.start(true)
		_add_limb_lock("rf")
	
	await get_tree().process_frame
	
	character.bone_sim.active = true
	character.bone_sim.physical_bones_start_simulation()


func _add_limb_lock(code: String) -> void:

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



func update_climbing(delta) -> void:
	# Basic version: simply hold onto the targets
	# IK solvers will take care of posing the character.
	pass

func exit_climb():
	attached_holds = {"lh":null, "rh":null, "lf":null, "rf":null}
