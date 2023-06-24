int32 velocity_profile   # velocity profile: (3) trapezoidal profile, (2) triangular profile, (1) rectangular profile, (0) step (no profile)
bool following_error     # whether the motor is following the desired position trajectory
bool profile_ongoing     # whether a profile is in progress with Goal Position instruction
bool in_position         # whether the motor has arrived to the desired position
