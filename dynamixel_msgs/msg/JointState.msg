Header header
string name         # joint name
int32[] motor_ids   # motor ids controlling this joint
int32[] motor_temps # motor temperatures, same order as motor_ids

float64 goal_pos    # commanded position (in radians)
float64 present_pos # present joint position (in radians)
float64 error       # error between commanded and current positions (in radians)
float64 velocity    # present joint speed (in radians per second)
float64 current     # present current
bool is_moving      # is joint currently in motion

dynamixel_msgs/MovingStatus moving_status   # detailed motor moving status
