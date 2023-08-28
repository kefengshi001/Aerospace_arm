#!/bin/bash

# 启动rviz
gnome-terminal -- bash -c "
cd /home/k/文档/Aerospace-arm;
export ROS_PACKAGE_PATH=/home/k/文档/Aerospace-arm:$ROS_PACKAGE_PATH;
source /home/k/文档/Aerospace-arm/devel/setup.bash;
roslaunch modified-urdf display.launch ; 
exec bash"
#延迟3秒让rviz完全启动
sleep 3 


# export ROS_PACKAGE_PATH=/home/k/图片/traj_app:$ROS_PACKAGE_PATH    #手动导出一些关键的环境变量，以确保脚本中的命令正确运行
#启动运动
gnome-terminal -- bash -c "
cd /home/k/图片/Aerospace_arm;
export ROS_PACKAGE_PATH=/home/k/图片/Aerospace_arm:$ROS_PACKAGE_PATH; 
source /home/k/图片/Aerospace_arm/devel/setup.bash; 
rosrun my_ros_package pub_points_in_line;
exec bash"

#启动cube（墙面）
gnome-terminal -- bash -c "
cd /home/k/图片/Aerospace_arm;
export ROS_PACKAGE_PATH=/home/k/图片/Aerospace_arm:$ROS_PACKAGE_PATH; 
source /home/k/图片/Aerospace_arm/devel/setup.bash; 
rosrun my_ros_package cube_pub;
exec bash"

#启动末端逆解存在区间轨迹
gnome-terminal -- bash -c "
cd /home/k/图片/Aerospace_arm;
export ROS_PACKAGE_PATH=/home/k/图片/Aerospace_arm:$ROS_PACKAGE_PATH; 
source /home/k/图片/Aerospace_arm/devel/setup.bash; 
rosrun my_ros_package data_to_graph;
exec bash"
