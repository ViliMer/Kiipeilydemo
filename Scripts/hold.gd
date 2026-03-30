class_name Hold
extends Node3D

enum HoldType {JUG, CRIMP, SLOPER, PINCH}
var type = HoldType.JUG

@export var hold_name: String

# At the moment this is only used to form joints between limbs and holds. It might not be needed
@onready var static_body: StaticBody3D = $CSGBakedMeshInstance3D/StaticBody3D

@onready var hand_grab_point: Marker3D = $hand_grab_point
@onready var foot_grab_point: Marker3D = $foot_grab_point
@onready var grab_point: Marker3D = $grab_point

func get_hand_grab_transform() -> Transform3D:
	return hand_grab_point.global_transform

func get_foot_grab_transform() -> Transform3D:
	return foot_grab_point.global_transform

func get_grab_point_transform() -> Transform3D:
	return grab_point.global_transform

# This function should return the global transform of a physical bone, so that the grabbing looks as natural as possible.
func get_limb_grab_transform(code: String, physical_bone: PhysicalBone3D, upstream_bone_transform: Transform3D) -> Transform3D:
	
	var hold_t: Transform3D = grab_point.global_transform
	
	# 1. Get limb grab point node
	var grab_node: Node3D
	
	match code:
		"lh", "rh":
			grab_node = physical_bone.get_node("JugGrabPoint")
		"lf", "rf":
			grab_node = physical_bone.get_node("BigFootHoldGrabPoint")
		_:
			push_error("Invalid limb code in get_limb_grab_transform")
			return Transform3D.IDENTITY
	
	if grab_node == null:
		push_error("Missing grab node on physical bone")
		return Transform3D.IDENTITY
	
	
	# 2. Compute desired orientation
	# WIP: use upstream_bone_transform to turn forward (and possibly right) towards the upstream bone so grabbing looks more natural.
	# Furthermore, I should change the basis calculation to depend on the grab_point basis instead of hold basis, so that holds can have many grab points in different orientaions.
	# For this end IK target node bases cannot depend on the grab_point basis because of the sdf is configured to work for finding valid hip positions, and they should instead depend on the hold basis.
	
	# Hold forward = "out of wall"
	var forward: Vector3 = -hold_t.basis.z.normalized() 
	
	var up_hint: Vector3 = hold_t.basis.y.normalized()
	var right: Vector3 = up_hint.cross(forward).normalized()
	var up: Vector3 = forward.cross(right).normalized()
	
	
	var target_basis: Basis
	match code:
		"lh":
			target_basis = Basis(right, up, forward).rotated(forward, PI/2)
		"rh":
			target_basis = Basis(right, up, forward).rotated(forward, -PI/2)
		"lf":
			target_basis = Basis(right, up, forward).rotated(forward, PI/2)
		"rf":
			target_basis = Basis(right, up, forward).rotated(forward, -PI/2)
	
	
	# 3. Target grab transform (where bone grab point should be)
	var target_grab_transform := Transform3D(target_basis, hold_t.origin)
	
	
	# 4. Compute bone local grab offset
	var bone_global: Transform3D = physical_bone.global_transform
	var grab_global: Transform3D = grab_node.global_transform
	
	var local_grab_offset: Transform3D = bone_global.affine_inverse() * grab_global
	
	
	# 5. Solve for bone transform
	var limb_grab_transform: Transform3D = target_grab_transform * local_grab_offset.affine_inverse()
	
	return limb_grab_transform
