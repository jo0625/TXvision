std_msgs/Header header
bool tracking
string id
# armors_num=1: position/velocity describe the plate; yaw, v_yaw, radii and dz are zero.
# armors_num=2/3/4: position/velocity describe the rotating target center.
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
