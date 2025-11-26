extends Node

signal start_route(route: Route)

# Development UI signals

# Hands
signal left_hand_strength_changed(value)
signal release_left_hand
signal reach_left_hand
signal left_hand_target_changed(value)

signal right_hand_strength_changed(value)
signal release_right_hand
signal reach_right_hand
signal right_hand_target_changed(value)

# Legs
signal left_leg_strength_changed(value)
signal release_left_leg
signal reach_left_leg
signal left_leg_target_changed(value)

signal right_leg_strength_changed(value)
signal release_right_leg
signal reach_right_leg
signal right_leg_target_changed(value)
