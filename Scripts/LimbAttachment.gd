extends Node
class_name LimbAttachment

var code: String
var limb: PhysicalBone3D
var hold: Hold
var joint: Generic6DOFJoint3D

var hold_anchor: Vector3
var limb_anchor: Vector3
var grab_point: Node3D

# Force sensor params
var spring_k: float = 200000.0
var damping: float = 100000.0

var current_force: Vector3 = Vector3.ZERO

func _init(limb_code: String, physical_limb: PhysicalBone3D, climbing_hold: Hold) -> void:
	code = limb_code
	limb = physical_limb
	hold = climbing_hold
	
	hold_anchor = hold.get_grab_transform().origin
	
	match code:
		"lh": grab_point = limb.find_child("JugGrabPoint")
		"rh": grab_point = limb.find_child("JugGrabPoint")
		"lf": grab_point = limb.find_child("BigFootHoldGrabPoint")
		"rf": grab_point = limb.find_child("BigFootHoldGrabPoint")
	
	limb_anchor = grab_point.global_position
	
	joint = Generic6DOFJoint3D.new()
	hold.add_child(joint)
	joint.global_transform.origin = limb_anchor
	
	joint.node_a = limb.get_path()
	joint.node_b = hold.static_body.get_path()
	
	set_joint_default_params()

func set_joint_default_params() -> void:
	# Allow a bit of linear movement for force sensor
	var limit: float = 0.00
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, -limit)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, limit)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, -limit)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, limit)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, -limit)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, limit)
	
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_STIFFNESS, spring_k)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_STIFFNESS, spring_k)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_STIFFNESS, spring_k)
	
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_DAMPING, damping)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_DAMPING, damping)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_DAMPING, damping)
	
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_SPRING, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_SPRING, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_SPRING, true)
	
	# Rotation limits
	# These need work later, they need to be somehow calculated based on the hold orientation, and independent of the limb orientation when attachment is made
	if code == "lh" or code == "rh":
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, -0.5)
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.5)
		
		#joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)
	else:
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, -0.5)
		joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.5)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, -0.5)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.5)
		
		#joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)
		#joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)

func compute_contact_force() -> Vector3:
	var limb_pos: Vector3 = grab_point.global_position
	
	# Hooke's law
	var current_delta: Vector3 = limb_pos - limb_anchor
	current_force = current_delta * spring_k
	
	return current_force
