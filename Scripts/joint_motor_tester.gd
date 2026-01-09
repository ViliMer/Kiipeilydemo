extends Control

enum JointMotorTesterState { DISABLED, APPLY_TORQUE, APPLY_VELOCITY, DRIVE_TORQUE_MOTORS_TO_POSE, DRIVE_VELOCITY_MOTORS_TO_POSE }
var state = JointMotorTesterState.DISABLED
# Apply this state machine, so that buttons make sense. Get rid of many bools. Then test velocity motors.
# Did this. Flipped velocity motor direction and seems to work. Investigate why rotation is according to left hand rule (I thought it should be according to right hand rule)
# For some reason it just seems that the velocity motor operates according to the left hand rule.
# There is some weirdness with the velocity motors. Even rotating arount the other axes (that I don't think should produce a rotation) still produce a rotation. Torque motor does not seem to do this.
# The rotation around other axes seem to have some angle that they are converging towards.
# X and Z axes form a plane where the joint is free to rotate. The velocity vector seems to try to bend to the same direction always, whether changing x or z.
# I don't observe the same problem in T pose

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
var joints: Array[Generic6DOFJoint3D]

var saved_joint_angles: Dictionary
var previous_joint_angles := {}
var previous_joint_torques := {}
var joint_error_integrals := {}

var stored_nodes: Dictionary = {}

var torque_to_be_applied = Vector3(0,0,0)
var torque_multiplier = 0.0

var velocity_to_be_applied = Vector3(0,0,0)
var velocity_multiplier = 0.0

##### CONFIGURATION #####
var bones_to_simulate: Array[StringName] = ["Spine"]
var free_joints: Array[String] = ["Spine Joint", "LowerChest Joint", "Chest Joint"] #["LeftKnee Joint", "LeftAnkle Joint"]
var joints_to_disable: Array[String] = ["LeftUpperChest Joint", "LeftShoulder Joint", "LeftElbow Joint", "LeftWrist Joint", "RightUpperChest Joint", "RightShoulder Joint", "RightElbow Joint", "RightWrist Joint"]#["RightWrist Joint"]

func init(c: CharacterBody3D):
	character = c
	joints = character.get_joints()

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
	
	for joint in joints:
		if joint.name in joints_to_disable:
			var bone = joint.get_node_or_null(joint.node_b)
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
	

func update_motor_test(delta: float):
	match state:
		JointMotorTesterState.DISABLED:
			pass
		JointMotorTesterState.DRIVE_TORQUE_MOTORS_TO_POSE:
			update_torque_motors(saved_joint_angles, delta)
		JointMotorTesterState.DRIVE_VELOCITY_MOTORS_TO_POSE:
			update_velocity_motors(saved_joint_angles, delta)
		JointMotorTesterState.APPLY_TORQUE:
			apply_torque(torque_to_be_applied * torque_multiplier, free_joints)
		JointMotorTesterState.APPLY_VELOCITY:
			apply_velocity(velocity_to_be_applied * velocity_multiplier, free_joints)

func save_joint_angles(joint_names: Array[String]) -> Dictionary:
	var result := {}
	if joints == null:
		return result

	for joint in joints:
		if joint.name not in joint_names:
			continue
		var node_a := joint.get_node_or_null(joint.node_a)
		var node_b := joint.get_node_or_null(joint.node_b)

		if node_a == null or node_b == null:
			continue
		
		var Ba = node_a.global_transform.basis
		var Bb = node_b.global_transform.basis
		var Bj = joint.global_transform.basis
		
		var A_in_joint = Bj.inverse() * Ba
		var B_in_joint = Bj.inverse() * Bb
		
		var rel_basis = B_in_joint * A_in_joint.inverse()

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
	for joint in joints:
		if joint.name not in joint_names:
			continue
		var node_a := joint.get_node_or_null(joint.node_a)
		var node_b := joint.get_node_or_null(joint.node_b)
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
	for joint in joints:
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

func get_PID_data(joint: Generic6DOFJoint3D, delta: float) -> Dictionary:
	# Returns:
	# {
	#	"error_vec": Vector3,			# joint-local
	#	"angular_velocity": Vector3,	# joint-local
	#	"error_integral": Vector3		# joint-local
	# }
	
	var node_a = joint.get_node_or_null(joint.node_a)
	var node_b = joint.get_node_or_null(joint.node_b)
	if node_a == null or node_b == null:
		push_warning("Joint '%s' has missing nodes." % joint.name)
		return {"error_vec": Vector3.ZERO,"angular_velocity": Vector3.ZERO, "error_integral": Vector3.ZERO}

	# ---- BASES ----
	var Ba: Basis = node_a.global_transform.basis
	var Bb: Basis = node_b.global_transform.basis
	var Bj: Basis = joint.global_transform.basis

	# ---- TRANSFORMS INTO JOINT SPACE ----
	var A_in_joint: Basis = Bj.inverse() * Ba
	var B_in_joint: Basis = Bj.inverse() * Bb

	# ---- RELATIVE ROTATION (A → B, joint-local) ----
	var rel_basis: Basis = B_in_joint * A_in_joint.inverse()
	var q_current: Quaternion = rel_basis.get_rotation_quaternion().normalized()

	# ---- TARGET QUATERNION (joint-local) ----
	var s = saved_joint_angles.get(joint.name, null)
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

	#var error_vec: Vector3 = q_delta.get_axis().normalized() * q_delta.get_angle()

	# ---- ANGULAR VELOCITY (DERIVATIVE TERM) ----
	var prev_q: Quaternion = previous_joint_angles.get(joint.name, q_current)
	
	if q_current.dot(prev_q) < 0.0:
		prev_q = -prev_q
	
	var q_vel: Quaternion = (q_current * prev_q.inverse()).normalized()
	if q_vel.w < 0.0:
		q_vel = -q_vel

	var angular_velocity := Vector3.ZERO
	
	var v_vel := Vector3(q_vel.x, q_vel.y, q_vel.z)
	var v_vel_len := v_vel.length()
	var vel_angle := 2.0 * atan2(v_vel_len, q_vel.w)
	if v_vel_len > 1e-6:
		angular_velocity = (v_vel / v_vel_len) * (vel_angle / delta)
	
	'''# Debugging
	if angular_velocity.length() > 0.4:
		print("High angluar velocity")
		print("Angular velocity: " + str(angular_velocity.length()))
		#print(angular_velocity)
		#print("Axis length: " + str(q_vel.get_axis().normalized().length()))
		print("v_vel: " + str(v_vel))
		print("v_vel_len: " + str(v_vel_len))
	print("angle: " + str(vel_angle))'''
	
	# ---- INTEGRAL TERM ----
	var error_integral: Vector3 = joint_error_integrals.get(joint.name, Vector3.ZERO)
	error_integral += error_vec * delta

	# Anti-windup clamp (important)
	var max_integral := 3.0
	if error_integral.length() > max_integral:
		error_integral = error_integral.normalized() * max_integral

	joint_error_integrals[joint.name] = error_integral

	# ---- STORE FOR NEXT FRAME ----
	previous_joint_angles[joint.name] = q_current

	return {
		"error_vec": error_vec,
		"angular_velocity": angular_velocity,
		"error_integral": error_integral
	}

var JOINT_CONSTANTS = {
	"Spine Joint":			{ "max_torque": 120.0, "gain_multiplier": 20.0 },
	"LowerChest Joint":		{ "max_torque": 120.0, "gain_multiplier": 15.0 },
	"Chest Joint":			{ "max_torque": 120.0, "gain_multiplier": 10.0 },
	"LeftUpperChest Joint":	{ "max_torque": 120.0, "gain_multiplier": 0.0 },
	"LeftShoulder Joint":	{ "max_torque": 80.0, "gain_multiplier": 0.0 },
	"LeftElbow Joint":		{ "max_torque": 40.0, "gain_multiplier": 0.0 },
	"LeftWrist Joint":		{ "max_torque": 15.0, "gain_multiplier": 0.0 },
	"RightUpperChest Joint":{ "max_torque": 120.0, "gain_multiplier": 0.0 },
	"RightShoulder Joint":	{ "max_torque": 80.0, "gain_multiplier": 0.0 },
	"RightElbow Joint":		{ "max_torque": 40.0, "gain_multiplier": 0.0 },
	"RightWrist Joint":		{ "max_torque": 15.0, "gain_multiplier": 0.0 },
	"LeftHip Joint":		{ "max_torque": 150.0 },
	"LeftKnee Joint":		{ "max_torque": 120.0, "gain_multiplier": 10.0 },
	"LeftAnkle Joint":		{ "max_torque": 50.0, "gain_multiplier": 1.0 },
	"RightHip Joint":		{ "max_torque": 150.0 },
	"RightKnee Joint":		{ "max_torque": 120.0 },
	"RightAnkle Joint":		{ "max_torque": 50.0 },
}

func update_torque_motors(saved: Dictionary, delta: float, strength := 1.0) -> void:
	if joints == null or saved == null:
		return

	# Pretty good values for knee joint
	#var Kp := 5.0 * strength	# proportional gain
	#var Kd := 2.0 * strength	# derivative gain
	#var Ki := 10.0 * strength	# integral gain

	for joint in joints:
		var joint_name = joint.name
		if not saved.has(joint_name):
			continue
		
		
		var gain_multiplier = JOINT_CONSTANTS[joint_name].gain_multiplier
		# Ankle values
		var Kp: float = 1.5 * gain_multiplier	# proportional gain
		var Kd: float = 0.2 * gain_multiplier	# derivative gain
		var Ki: float = 0.1 * gain_multiplier	# integral gain

		var res = get_PID_data(joint, delta)
		var error_vec = res["error_vec"]
		var angular_velocity = res["angular_velocity"]
		var error_integral = res["error_integral"]
		
		# ---- PD CONTROL ----
		var target_torque = (error_vec * Kp) + (error_integral * Ki) - (angular_velocity * Kd)
		
		# Cap torque
		var max_torque = JOINT_CONSTANTS[joint_name].max_torque
		if target_torque.length() > max_torque:
			target_torque = target_torque.normalized() * max_torque
		
		# Cap change in torque
		var prev_torque = previous_joint_torques.get(joint_name, Vector3.ZERO)
		var max_delta = max_torque * 1 * delta  # tune per joint
		var delta_torque = target_torque - prev_torque
		if delta_torque.length() > max_delta:
			delta_torque = delta_torque.normalized() * max_delta

		target_torque = prev_torque + delta_torque
		previous_joint_torques[joint_name] = target_torque

		var torque_world = joint.global_transform.basis * target_torque
		#var Bj: Basis = joint.global_transform.basis
		#DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + Bj.x, Color.RED, 0.01)
		#DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + Bj.y, Color.GREEN, 0.01)
		#DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + Bj.z, Color.YELLOW, 0.01)
		DebugDraw3D.draw_arrow(joint.global_transform.origin, joint.global_transform.origin + joint.global_transform.basis * error_vec, Color.YELLOW, 0.01)
		DebugDraw3D.draw_arrow(
			joint.global_transform.origin,
			joint.global_transform.origin + torque_world,
			Color.BLUE,
			0.01
		)
		var node_a = joint.get_node_or_null(joint.node_a)
		var node_b = joint.get_node_or_null(joint.node_b)
		node_a.external_torque -= torque_world
		node_b.external_torque += torque_world

func update_velocity_motors(saved: Dictionary, delta: float, strength := 1.0) -> void:
	if joints == null or saved == null:
		return

	var Kp := 1.0 * strength	# proportional gain
	var Kd := 0.0 * strength	# derivative gain

	for joint in joints:

		var joint_name = joint.name
		if not saved.has(joint_name):
			continue
		
		var res = get_PID_data(joint, delta)
		var error_vec = res["error_vec"]
		var angular_velocity = res["angular_velocity"]
		
		# ---- PD CONTROL ----
		var target_vel = (error_vec * Kp) - (angular_velocity * Kd)

		var vel_world = joint.global_transform.basis * target_vel * -1
		
		DebugDraw3D.draw_arrow(
			joint.global_transform.origin,
			joint.global_transform.origin + vel_world,
			Color.BLUE,
			0.01
		)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, vel_world.x)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, vel_world.y)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, vel_world.z)


func debug_draw_joint_error(joint: Generic6DOFJoint3D, error_joint_local: Vector3) -> void:
	var origin = joint.global_transform.origin
	
	var world_dir = joint.global_transform.basis * error_joint_local
	
	DebugDraw3D.draw_arrow(origin, origin + world_dir*2, Color.RED, 0.01)

func restore_node(node_name: String) -> void: # This doesn't actually work. Would need to figure out the right position as well.
	if stored_nodes.has(node_name):
		var data = stored_nodes[node_name]
		data.parent.add_child(data.node)
		stored_nodes.erase(node_name)

func enable_velocity_motors(joint_names: Array[String], strength := 1.0) -> void:
	if joints == null:
		return

	var max_force := 10.0 * strength
	
	for joint in joints:
		if joint.name not in joint_names:
			continue
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)

		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)

func disable_velocity_motors(joint_names: Array[String]) -> void:

	for joint in joints:
		if joint.name not in joint_names:
			continue
		joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)
		
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)


func _on_save_pressed() -> void:
	save_joint_angles(free_joints)

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
