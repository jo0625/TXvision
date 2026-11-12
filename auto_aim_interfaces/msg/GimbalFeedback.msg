# Host receive time. The MCU clock is not synchronized to ROS time.
std_msgs/Header header
uint32 sequence
uint32 mcu_timestamp_ms
geometry_msgs/Quaternion orientation
float64 yaw
float64 pitch
uint8 mode
