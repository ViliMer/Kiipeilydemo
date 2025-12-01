class_name Hold
extends Node3D

@export var hold_name: String

@onready var hand_grab_point: Marker3D = $hand_grab_point
@onready var foot_grab_point: Marker3D = $foot_grab_point

func get_left_hand_grab_transform() -> Transform3D:
	return hand_grab_point.global_transform

func get_right_hand_grab_transform() -> Transform3D:
	return hand_grab_point.global_transform
	#var t = hand_grab_point.global_transform
	#var rot = Basis(Vector3(0, 0, 1), deg_to_rad(180.0))
	#t.basis = t.basis * rot
	#return t

func get_foot_grab_transform() -> Transform3D:
	return foot_grab_point.global_transform
