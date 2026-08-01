# Private driver transport message.  A setpoint belongs to exactly one driver
# safety epoch; the driver drops it after stop/fault/re-arm changes the epoch.
std_msgs/Header header
uint64 safety_epoch
string[] joint_names
float64[] positions
float64 speed_scale
