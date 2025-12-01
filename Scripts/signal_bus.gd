extends Node

@warning_ignore("unused_signal")
signal start_route(route: Route)

# Development UI signals

# Hands
@warning_ignore("unused_signal")
signal left_hand_strength_changed(value: float)
@warning_ignore("unused_signal")
signal release_left_hand
@warning_ignore("unused_signal")
signal reach_left_hand
@warning_ignore("unused_signal")
signal left_hand_target_changed(value: int)

@warning_ignore("unused_signal")
signal right_hand_strength_changed(value: float)
@warning_ignore("unused_signal")
signal release_right_hand
@warning_ignore("unused_signal")
signal reach_right_hand
@warning_ignore("unused_signal")
signal right_hand_target_changed(value: int)

# Legs
@warning_ignore("unused_signal")
signal left_leg_strength_changed(value: float)
@warning_ignore("unused_signal")
signal release_left_foot
@warning_ignore("unused_signal")
signal reach_left_foot
@warning_ignore("unused_signal")
signal left_foot_target_changed(value: int)

@warning_ignore("unused_signal")
signal right_leg_strength_changed(value: float)
@warning_ignore("unused_signal")
signal release_right_foot
@warning_ignore("unused_signal")
signal reach_right_foot
@warning_ignore("unused_signal")
signal right_foot_target_changed(value: int)
