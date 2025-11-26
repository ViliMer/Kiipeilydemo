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
