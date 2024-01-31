KORTEX_ROS_DIR="/kortex/catkin_workspace"
MONOCLONE_ROS_DIR="/monoclone/monoclone_ros"

source $KORTEX_ROS_DIR/devel/setup.bash
cd $MONOCLONE_ROS_DIR
rm -rf devel
rm -rf build
catkin_make
