KORTEX_ROS_DIR="/kortex/catkin_workspace"
MONOCLONE_ROS_DIR="/monoclone/monoclone_ros"

source $KORTEX_ROS_DIR/devel/setup.bash
source $MONOCLONE_ROS_DIR/devel/setup.bash
roslaunch kortex_gazebo spawn_kortex_robot.launch start_rviz:=false gripper:=robotiq_2f_85

