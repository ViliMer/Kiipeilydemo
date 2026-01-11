extends PhysicalBone3D

var external_force: Vector3 = Vector3.ZERO
var external_torque: Vector3 = Vector3.ZERO

func _integrate_forces(state):
	if external_force != Vector3.ZERO:
		state.apply_central_force(external_force)

	if external_torque != Vector3.ZERO:
		state.apply_torque(external_torque)

	external_force = Vector3.ZERO
	external_torque = Vector3.ZERO

func get_center_of_mass_world() -> Vector3:
	for child in get_children():
		if child is CollisionShape3D:
			# CollisionShape origin is local to the bone
			return global_transform * child.transform.origin

	# Fallback (should not happen if setup is correct)
	print("WARNING: USING FALLBACK IN get_center_of_mass_world()")
	return global_transform.origin
