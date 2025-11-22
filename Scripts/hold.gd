extends Node3D
@onready var grab_point: Marker3D = $grab_point

func get_grab_transform() -> Transform3D:
	# Return the global-space transform of the grab point where limbs attach
	return grab_point.global_transform
