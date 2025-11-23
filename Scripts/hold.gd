extends Node3D

@onready var hand_grab_point: Marker3D = $hand_grab_point
@onready var foot_grab_point: Marker3D = $foot_grab_point

func get_hand_grab_transform() -> Transform3D:
	return hand_grab_point.global_transform

func get_foot_grab_transform() -> Transform3D:
	return foot_grab_point.global_transform
