# 机身坐标下的xyz速度
geometry_msgs/Vector3 cydog_vel
# 这个存放的是狗的rpy[3] 翻滚角 俯仰角 偏航角
geometry_msgs/Vector3 cydog_rpy
# 机身朝向四元数
geometry_msgs/Quaternion facing
# 实际时间戳
uint64 timestamps
float64 time
# 机身角速度
geometry_msgs/Vector3 cydog_angle_vel
# 机身位置
geometry_msgs/Pose cydog_pose