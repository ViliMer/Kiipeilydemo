extends Control

enum JointMotorTesterState { DISABLED, APPLY_TORQUE, APPLY_VELOCITY, DRIVE_TORQUE_MOTORS_TO_POSE, DRIVE_VELOCITY_MOTORS_TO_POSE }
var state = JointMotorTesterState.DISABLED

@onready var torque_mag_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/VBoxContainer2/HBoxContainer/TorqueMagLineEdit
@onready var torque_x_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/VBoxContainer2/HBoxContainer2/TorqueXLineEdit
@onready var torque_y_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/VBoxContainer2/HBoxContainer3/TorqueYLineEdit
@onready var torque_z_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/VBoxContainer2/HBoxContainer4/TorqueZLineEdit

@onready var velocity_mag_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/VBoxContainer3/HBoxContainer/VelocityMagLineEdit
@onready var velocity_x_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/VBoxContainer3/HBoxContainer2/VelocityXLineEdit
@onready var velocity_y_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/VBoxContainer3/HBoxContainer3/VelocityYLineEdit
@onready var velocity_z_line_edit: LineEdit = $MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/VBoxContainer3/HBoxContainer4/VelocityZLineEdit

@onready var disable_motors_btn: Button = $"MarginContainer/VBoxContainer/Disable Motors"
@onready var drive_torque_motors_to_pose_btn: Button = $"MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/Drive Torque Motors To Pose"
@onready var apply_torque_btn: Button = $"MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer/VBoxContainer2/Apply Torque"
@onready var drive_velocity_motors_to_pose_btn: Button = $"MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/Drive Velocity Motors To Pose"
@onready var apply_velocity_btn: Button = $"MarginContainer/VBoxContainer/HBoxContainer/VBoxContainer2/VBoxContainer3/Apply Velocity"

@onready var motor_mode_group := ButtonGroup.new()

func _ready():
	disable_motors_btn.button_group = motor_mode_group
	drive_torque_motors_to_pose_btn.button_group = motor_mode_group
	apply_torque_btn.button_group = motor_mode_group
	drive_velocity_motors_to_pose_btn.button_group = motor_mode_group
	apply_velocity_btn.button_group = motor_mode_group
	
	for btn in [
		disable_motors_btn,
		drive_torque_motors_to_pose_btn,
		apply_torque_btn,
		drive_velocity_motors_to_pose_btn,
		apply_velocity_btn
	]:
		btn.toggled.connect(_on_motor_mode_toggled.bind(btn))

var character: CharacterBody3D

var saved_joint_angles: Dictionary
var previous_joint_angles := {}
var joint_error_integrals := {}

var stored_nodes: Dictionary = {}

var torque_to_be_applied = Vector3(0,0,0)
var torque_multiplier = 0.0

var velocity_to_be_applied = Vector3(0,0,0)
var velocity_multiplier = 0.0

##### CONFIGURATION #####
var bones_to_simulate: Array[StringName] = ["Hips"]
var free_joints: Array[String] =	[
										"Spine Joint", "LowerChest Joint", "Chest Joint",
										"LeftUpperChest Joint", "LeftShoulder Joint", "LeftElbow Joint", "LeftWrist Joint",
										"RightUpperChest Joint", "RightShoulder Joint", "RightElbow Joint", "RightWrist Joint",
										"LeftHip Joint", "LeftKnee Joint", "LeftAnkle Joint",
										"RightHip Joint", "RightKnee Joint", "RightAnkle Joint"
									]
var joints_to_disable: Array[String] = ["Root Joint"]

func init(c: CharacterBody3D):
	character = c

func _input(_event: InputEvent) -> void:
	if not character.state == character.PlayerState.TEST_JOINT_MOTORS:
		return
	if Input.is_action_just_pressed("Save Pose"):
		print("Saving free joint angles...")
		save_joint_angles(free_joints)

func enable():
	character.collision_shape.disabled = true
	character.velocity = Vector3.ZERO
	character.anim.play("hand_pose")
	character.anim.seek(0.0, true)
	character.anim.stop()
	character.bone_sim.active = true
	
	for joint in character.joints.values():
		if joint.name in joints_to_disable:
			var bone = joint.get_node_or_null(joint.node_b)
			if joint.name == "Root Joint":
				bone = joint.get_node_or_null(joint.node_a)
			var parent = bone.get_parent()
			if parent:
				parent.remove_child(bone)
				stored_nodes[bone.name] = {
					"node": bone,
					"parent": parent
				}
			
	character.run_bone_sim(true, bones_to_simulate)
	
	print("Joint test enabled")
	
	visible = true

func disable():
	character.bone_sim.active = false
	character.run_bone_sim(false)
	character.skeleton.reset_bone_poses()
	character.collision_shape.disabled = false
	character.anim.play("Idle")
	character.rotation.x = 0.0
	character.rotation.z = 0.0
	
	print("Joint test disabled")
	
	visible = false
	

func update_motor_test(_delta: float):
	match state:
		JointMotorTesterState.DISABLED:
			pass
		JointMotorTesterState.DRIVE_TORQUE_MOTORS_TO_POSE:
			update_velocity_torque_motors(saved_joint_angles)
		JointMotorTesterState.DRIVE_VELOCITY_MOTORS_TO_POSE:
			update_velocity_motors(saved_joint_angles)
		JointMotorTesterState.APPLY_TORQUE:
			apply_torque(torque_to_be_applied * torque_multiplier, free_joints)
		JointMotorTesterState.APPLY_VELOCITY:
			apply_velocity(velocity_to_be_applied * velocity_multiplier, free_joints)

func save_joint_angles(joint_names: Array[String]) -> Dictionary:
	var result := {}
	if character.joints.values() == null:
		return result

	for joint in character.joints.values():
		if joint.name not in joint_names:
			continue
		var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
		var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)

		if node_a == null or node_b == null:
			continue
		
		var B_joint: Basis = joint.basis
		var Ba: Basis = node_a.global_basis
		var Bb: Basis = node_b.global_basis
		
		# In joint space:
		var rel_basis = B_joint.inverse() * (Ba.inverse() * Bb) * B_joint

		var q : Quaternion = rel_basis.get_rotation_quaternion().normalized()

		result[joint.name] = {
			"qx": q.x,
			"qy": q.y,
			"qz": q.z,
			"qw": q.w
		}

	saved_joint_angles = result
	return result

func apply_torque(torque: Vector3, joint_names: Array[String]) -> void:
	for joint in character.joints.values():
		if joint.name not in joint_names:
			continue
		
		var node_a := joint.get_node_or_null(joint.node_a) as PhysicalBone3D
		var node_b := joint.get_node_or_null(joint.node_b) as PhysicalBone3D
		if node_a == null or node_b == null:
			continue
		var torque_world = joint.global_transform.basis * torque
		node_a.external_torque += torque_world
		node_b.external_torque -= torque_world
		DebugDraw3D.draw_arrow(
			joint.global_transform.origin,
			joint.global_transform.origin + torque_world,
			Color.BLUE,
			0.01
		)

func apply_velocity(velocity: Vector3, joint_names: Array[String]) -> void:
	for joint in character.joints.values():
		if joint.name not in joint_names:
			continue
		var velocity_world = joint.global_transform.basis * velocity
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_world.x)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_world.y)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_world.z)
		DebugDraw3D.draw_arrow(
			joint.global_transform.origin,
			joint.global_transform.origin + velocity_world,
			Color.BLUE,
			0.01
		)

# This function returns P and D in world space
func get_PD_data(joint: Generic6DOFJoint3D, target: Dictionary) -> Dictionary:

	var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
	var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)
	if node_a == null or node_b == null:
		push_warning("Joint '%s' has missing nodes." % joint.name)
		return {"error_vec": Vector3.ZERO,"angular_velocity": Vector3.ZERO, "error_integral": Vector3.ZERO}

	# ---- BASES ----
	var B_joint: Basis = joint.basis
	var Ba: Basis = node_a.global_basis
	var Bb: Basis = node_b.global_basis

	# ---- RELATIVE ROTATION IN JOINT SPACE ----
	var rel_basis = B_joint.inverse() * (Ba.inverse() * Bb) * B_joint
	var q_current: Quaternion = rel_basis.get_rotation_quaternion().normalized()

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
''' Saving this if I need it:
# ---- INTEGRAL TERM ----
	var error_integral: Vector3 = joint_error_integrals.get(joint.name, Vector3.ZERO)
	error_integral += error_vec * delta

	# Anti-windup clamp (important)
	var max_integral := 3.0
	if error_integral.length() > max_integral:
		error_integral = error_integral.normalized() * max_integral

	joint_error_integrals[joint.name] = error_integral
'''

var JOINT_CONSTANTS = {																		#Vector3(5.0, 5.0, 5.0)
	"Spine Joint":			{ "max_torque": 200.0, "Kd": Vector3(5.0, 5.0, 5.0), "omega_max": Vector3(1.0, 1.0, 1.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
	"LowerChest Joint":		{ "max_torque": 120.0, "Kd": Vector3(4.0, 4.0, 4.0), "omega_max": Vector3(1.0, 1.0, 1.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
	"Chest Joint":			{ "max_torque": 120.0, "Kd": Vector3(3.0, 3.0, 3.0), "omega_max": Vector3(1.0, 1.0, 1.0), "error_scale": Vector3(0.125, 0.125, 0.125) }, # x = fwd/back, y = twist, z = side to side
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
}

func update_velocity_torque_motors(target: Dictionary) -> void:
	if character.joints.values() == null or target == null:
		return
	
	var gravity_compensations = character.compute_gravity_compensation_for_joints(free_joints)

	for joint in character.joints.values():
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
		
		# ---- GRAVITY (World Frame) ----
		var gravity_compensation: Vector3 = gravity_compensations[joint_name]
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
		
		var target_torque_world = joint_velocity_correction_torque_world + gravity_compensation
		
		# Cap torque
		var max_torque = JOINT_CONSTANTS[joint_name].max_torque
		if target_torque_world.length() > max_torque:
			print("Joint: " + joint_name + " hit its torque limit")
			target_torque_world = target_torque_world.normalized() * max_torque
		
		node_a.external_torque -= target_torque_world
		node_b.external_torque += target_torque_world

func update_torque_motors(target: Dictionary, _delta: float) -> void:
	if character.joints.values() == null or target == null:
		return
	
	var gravity_compensations = character.compute_gravity_compensation_for_joints(free_joints)

	for joint in character.joints.values():
		var joint_name = joint.name
		if not target.has(joint_name):
			continue
		
		var node_a: PhysicalBone3D = joint.get_node_or_null(joint.node_a)
		var node_b: PhysicalBone3D = joint.get_node_or_null(joint.node_b)
		
		var B_joint: Basis = joint.global_basis
		
		var Kp: Vector3 = JOINT_CONSTANTS[joint_name].Kp
		var Kd: Vector3 = JOINT_CONSTANTS[joint_name].Kd

		var res = get_PD_data(joint, target)
		var error_vec = res["error_vec"]
		var angular_velocity_world = res["angular_velocity"]
		
		# ---- GRAVITY (World Frame) ----
		var gravity_compensation: Vector3 = gravity_compensations[joint_name]
		
		# ---- PER-AXIS PD (DECOUPLED) ----
		var angular_velocity_joint = B_joint.inverse() * angular_velocity_world
		#Note: The joints have angular springs with low stiffness and moderate damping
		var damping: Vector3 = -Kd * angular_velocity_joint
		var damping_world = B_joint * damping
		
		var target_torque: Vector3 = Kp * error_vec + gravity_compensation + damping_world
		
		# Cap torque
		var max_torque = JOINT_CONSTANTS[joint_name].max_torque
		if target_torque.length() > max_torque:
			target_torque = target_torque.normalized() * max_torque
		
		DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + error_vec, Color.YELLOW, 0.01)
		DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + angular_velocity_world, Color.RED, 0.01)
		DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + target_torque, Color.BLUE, 0.01)
		
		node_a.external_torque -= target_torque
		node_b.external_torque += target_torque


func update_velocity_motors(target: Dictionary) -> void:
	if character.joints.values() == null or target == null:
		return

	for joint in character.joints.values():

		var joint_name = joint.name
		if not target.has(joint_name):
			continue
		
		var B_joint: Basis = joint.global_basis
		

		var res = get_PD_data(joint, target)
		var error_vec = res["error_vec"]
		
		# ---- GRAVITY (World Frame) ----
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
		
		omega_target_joint *= -0.1
		
		var vel_world = joint.global_transform.basis * omega_target_joint
		
		DebugDraw3D.draw_arrow(joint.global_transform.origin,joint.global_transform.origin + vel_world,Color.BLUE,0.01)
		DebugDraw3D.draw_arrow(joint.global_transform.origin,joint.global_transform.origin + error_vec,Color.YELLOW,0.01)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega_target_joint.x)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega_target_joint.y)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, omega_target_joint.z)

func restore_node(node_name: String) -> void: # This doesn't actually work. Would need to figure out the right position as well.
	if stored_nodes.has(node_name):
		var data = stored_nodes[node_name]
		data.parent.add_child(data.node)
		stored_nodes.erase(node_name)

func enable_velocity_motors(joint_names: Array[String]) -> void:
	if character.joints.values() == null:
		return
	
	for joint in character.joints.values():
		if joint.name not in joint_names:
			continue
		
		var max_torque = JOINT_CONSTANTS[joint.name].max_torque * 2
		
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)

		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_torque)

func disable_velocity_motors(joint_names: Array[String]) -> void:

	for joint in character.joints.values():
		if joint.name not in joint_names:
			continue
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)

func _on_torque_magnitude_slider_value_changed(value: float) -> void:
	torque_mag_line_edit.text = str(value)
	torque_multiplier = value

func _on_torque_x_slider_value_changed(value: float) -> void:
	torque_x_line_edit.text = str(value)
	torque_to_be_applied.x = value

func _on_torque_yslider_value_changed(value: float) -> void:
	torque_y_line_edit.text = str(value)
	torque_to_be_applied.y = value

func _on_torque_z_slider_value_changed(value: float) -> void:
	torque_z_line_edit.text = str(value)
	torque_to_be_applied.z = value

func _on_velocity_magnitude_slider_value_changed(value: float) -> void:
	velocity_mag_line_edit.text = str(value)
	velocity_multiplier = value


func _on_velocity_x_slider_value_changed(value: float) -> void:
	velocity_x_line_edit.text = str(value)
	velocity_to_be_applied.x = value


func _on_velocity_yslider_value_changed(value: float) -> void:
	velocity_y_line_edit.text = str(value)
	velocity_to_be_applied.y = value


func _on_velocity_z_slider_value_changed(value: float) -> void:
	velocity_z_line_edit.text = str(value)
	velocity_to_be_applied.z = value


func _on_motor_mode_toggled(toggled_on: bool, button: Button) -> void:
	if not toggled_on:
		return  # button was turned off → do nothing

	match button:
		disable_motors_btn:
			state = JointMotorTesterState.DISABLED
			disable_velocity_motors(free_joints)

		drive_torque_motors_to_pose_btn:
			state = JointMotorTesterState.DRIVE_TORQUE_MOTORS_TO_POSE
			disable_velocity_motors(free_joints)

		apply_torque_btn:
			state = JointMotorTesterState.APPLY_TORQUE
			disable_velocity_motors(free_joints)

		drive_velocity_motors_to_pose_btn:
			state = JointMotorTesterState.DRIVE_VELOCITY_MOTORS_TO_POSE
			enable_velocity_motors(free_joints)

		apply_velocity_btn:
			state = JointMotorTesterState.APPLY_VELOCITY
			enable_velocity_motors(free_joints)
