#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from
# http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# Modified twice by GNAQ to fill the requirements with monoclone teleoperation.

import sys
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from . import gazebo_api as gzapi


class Gen3ArmWithMoveIt(object):
    """Gen3ArmMoveItTrajectories"""

    def __init__(self):

        # Initialize the node
        super(Gen3ArmWithMoveIt, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            # 7-DoF as default
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            # use an optimal planner, the default one tends to produce weird paths.
            # self.arm_group.set_planner_id('SPARS')
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    # possible names: see kortex_move_it_config/gen3_move_it_config/config/7dof/gen3.srdf.xacro
    def set_reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    # Set Joint Positions
    # Example:
    # joint_positions = [pi / 2,  0,  pi / 4,  -pi / 4,  0,  pi / 2,  0.2]
    def set_joint_angles(self, tolerance: float, new_joint_positions: list, abs_param=True):
        # check parameters
        if len(new_joint_positions) < 7:
            raise ValueError('number of joint positions must by 7 as DoF indicated')

        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        current_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in current_joint_positions:
            rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if not abs_param:
            # treat new joint positions as deltas
            for i in range(0, 7):
                new_joint_positions[i] += current_joint_positions[i]
        rospy.loginfo(f"Printing target joint positions : {new_joint_positions}")
        arm_group.set_joint_value_target(new_joint_positions)

        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions:
            rospy.loginfo(p)
        return success

    def get_cartesian_pose_end_effector(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def set_cartesian_pose(self, pose, tolerance, constraints, abs_param=True):
        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Plan and execute
        if not abs_param:
            # treat pose as deltas
            cur_pose = arm_group.get_current_pose()
            pose.position.x += cur_pose.position.x
            pose.position.y += cur_pose.position.y
            pose.position.z += cur_pose.position.z
            pose.orientation.x += cur_pose.orientation.x
            pose.orientation.y += cur_pose.orientation.y
            pose.orientation.z += cur_pose.orientation.z
            pose.orientation.w += cur_pose.orientation.w

        rospy.loginfo(f"Planning and going to the Cartesian Pose : {pose}")
        arm_group.set_pose_target(pose)
        return arm_group.go(wait=True)

    def set_cartesian_position(self, pos: geometry_msgs.msg.Point, tolerance, constraints, abs_param=True):
        pose = self.get_cartesian_pose_end_effector()
        cur_position = pose.position
        pose.position = pos

        if not abs_param:
            pose.position.x += cur_position.x
            pose.position.y += cur_position.y
            pose.position.z += cur_position.z

        return self.set_cartesian_pose(pose, tolerance, constraints, abs_param=True)

    def set_gripper_clip_pose_rel(self, relative_position: float, abs_param=True):
        # We only have to move this joint because all others are mimic!
        gripper_group = self.gripper_group
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()

        if not abs_param:
            relative_position = relative_position + self.get_gripper_clip_pose_rel()

        relative_position = max(min(relative_position, 1), 0)
        try:
            val = gripper_joint.move(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos,
                True)
            return val
        except:
            return False

    def get_gripper_cartesian_pose(self):
        pass

    def get_gripper_clip_pose_rel(self):
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        return ((gripper_joint.value() - gripper_min_absolute_pos) /
                (gripper_max_absolute_pos - gripper_min_absolute_pos))

    def get_finger_pose(self) -> (geometry_msgs.msg.Pose, geometry_msgs.msg.Pose):
        left_finger_pos: gzapi.gz_srv.GetLinkStateResponse = gzapi.get_link_state('left_inner_finger', 'world')
        right_finger_pos: gzapi.gz_srv.GetLinkStateResponse = gzapi.get_link_state('right_inner_finger', 'world')
        return left_finger_pos.link_state.pose, right_finger_pos.link_state.pose

    def reset_arm_retract(self):
        rospy.loginfo('[monoclone:arm] start reset arm states')
        self.set_reach_named_position('retract')
        time.sleep(0.25)
        self.set_gripper_clip_pose_rel(0.0)
        time.sleep(0.25)
        rospy.loginfo('[monoclone:arm] reset arm states done')


"""
def workflow():
    example = Gen3ArmMoveItTrajectories()

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Reaching Named Target Vertical...")
        success &= example.reach_named_position("vertical")
        print(success)

    if success:
        rospy.loginfo("Reaching Joint Angles...")
        success &= example.reach_joint_angles(tolerance=0.01)  # rad
        print(success)

    if success:
        rospy.loginfo("Reaching Named Target Home...")
        success &= example.reach_named_position("home")
        print(success)

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")

        actual_pose = example.get_cartesian_pose()
        actual_pose.position.z -= 0.2
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
        print(success)

    if example.degrees_of_freedom == 7 and success:
        rospy.loginfo("Reach Cartesian Pose with constraints...")
        # Get actual pose
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.y -= 0.3

        # Orientation constraint (we want the end effector to stay the same orientation)
        constraints = moveit_msgs.msg.Constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.orientation = actual_pose.orientation
        constraints.orientation_constraints.append(orientation_constraint)

        # Send the goal
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print(success)

        rospy.loginfo("Closing the gripper 50%...")
        success &= example.reach_gripper_position(0.5)
        print(success)

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")
"""
