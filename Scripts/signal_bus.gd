extends Node

signal start_route(route: Route)

# Development UI signals

# Hands
signal left_hand_strength_changed(value: float)
signal release_left_hand
signal reach_left_hand
signal left_hand_target_changed(value: int)

signal right_hand_strength_changed(value: float)
signal release_right_hand
signal reach_right_hand
signal right_hand_target_changed(value: int)

# Legs
signal left_leg_strength_changed(value: float)
signal release_left_foot
signal reach_left_foot
signal left_foot_target_changed(value: int)

signal right_leg_strength_changed(value: float)
signal release_right_foot
signal reach_right_foot
signal right_foot_target_changed(value: int)
