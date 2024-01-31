# Copied from Kortex_examples and modified then.

cmake_minimum_required(VERSION 3.0.2)
project(monoclone_ros_teleop)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation actionlib_msgs actionlib kortex_driver)

## Declare a catkin package
catkin_package()

## disable testing for now
# if(CATKIN_ENABLE_TESTING)
#  	  find_package(rostest REQUIRED)
#	  catkin_add_gtest(kortex_examples_tests src/tests/kortex_examples_tests.cc)
#     target_link_libraries(kortex_examples_tests ${catkin_LIBRARIES})
# endif()

# Install

catkin_python_setup()
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/main.py src/camera.py src/observer.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation (cpp)
# install(TARGETS
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
