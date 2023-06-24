float64 timestamp   # motor state is at this time
int32 id            # motor id
int32 goal          # commanded position (in encoder units)
int32 position      # present position (in encoder units)
int32 error         # difference between current and goal positions
int32 speed         # present speed (0.111 rpm per unit)
float64 current     # present current draw
float64 voltage     # present voltage (V)
int32 temperature   # present temperature (degrees Celsius)
bool moving         # whether the motor is currently in motion

dynamixel_msgs/MovingStatus moving_status   # detailed motor moving status
