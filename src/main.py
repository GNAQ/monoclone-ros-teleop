import signal
import rospy
import zerorpc
import datetime
import gevent
import msgpack_numpy
from typing import List

from teleop import kinova_arm as karm, config
from teleop.utils import PoseWrap
from teleop.task import peg_ins


class Arm(object):
    def __init__(self):
        rospy.loginfo(f'[monoclone:main] setting up arm')
        self.arm = karm.Gen3ArmWithMoveIt()
        self.arm.reset_arm_retract()
        rospy.loginfo(f'[monoclone:main] arm reset done.')

    # check out named positions at: kortex_move_it_config/gen3_move_it_config/config/7dof/gen3.srdf.xacro
    def set_reach_named_position(self, target: str):
        rospy.loginfo(f'[monoclone:main] start set_reach_named_position at {datetime.datetime.now()}')
        if not self.arm.set_reach_named_position(target):
            raise RuntimeError('Teleop node : reach named position failed!')
        rospy.loginfo(f'[monoclone:main] done set_reach_named_position at {datetime.datetime.now()}')
        return f'done set_reach_named_position at {datetime.datetime.now()}'

    def set_joint_angles(self, tolerance: float, joint_positions: list, abs_param=True):
        rospy.loginfo(f'[monoclone:main] start set_joint_angles at {datetime.datetime.now()}')
        if not self.arm.set_joint_angles(tolerance, joint_positions, abs_param):
            raise RuntimeError('Teleop node : reach joint absolute angle failed!')
        rospy.loginfo(f'[monoclone:main] done set_joint_angles at {datetime.datetime.now()}')
        return f'done set_joint_angles at {datetime.datetime.now()}'

    # Currently there is no wrapper for `constraints` param.
    def set_cartesian_pose(self, pose: str, tolerance: float, constraints=None, abs_param=True):
        rospy.loginfo(f'[monoclone:main] start set_cartesian_pose at {datetime.datetime.now()}')
        ros_pose = PoseWrap.from_json_str(pose).to_ros_pose()
        if not self.arm.set_cartesian_pose(ros_pose, tolerance, constraints, abs_param):
            raise RuntimeError('Teleop node : set cartesian absolute pose failed!')
        rospy.loginfo(f'[monoclone:main] done set_cartesian_pose at {datetime.datetime.now()}')
        return f'done set_cartesian_pose at {datetime.datetime.now()}'

    def set_cartesian_position(self, pos: str, tolerance, constraints=None, abs_param=True):
        rospy.loginfo(f'[monoclone:main] start set_cartesian_position at {datetime.datetime.now()}')
        ros_position = PoseWrap.Position.from_json_str(pos).to_ros_position()
        if not self.arm.set_cartesian_position(ros_position, tolerance, constraints, abs_param):
            raise RuntimeError('Teleop node : set cartesian absolute position failed!')
        rospy.loginfo(f'[monoclone:main] done set_cartesian_position at {datetime.datetime.now()}')
        return f'done set_cartesian_position at {datetime.datetime.now()}'

    def set_gripper_clip_pose_rel(self, relative_position: float, abs_param=True):
        rospy.loginfo(f'[monoclone:main] start set_gripper_clip_pose_rel at {datetime.datetime.now()}')
        if not self.arm.set_gripper_clip_pose_rel(relative_position, abs_param):
            raise RuntimeError('Teleop node : set gripper clip relative pose failed!')
        rospy.loginfo(f'[monoclone:main] done set_gripper_clip_pose_rel at {datetime.datetime.now()}')
        return f'done set_gripper_clip_pose_rel at {datetime.datetime.now()}'

    def get_cartesian_pose_end_effector(self):
        result = self.arm.get_cartesian_pose_end_effector()
        return PoseWrap.from_ros_pose(result).to_json_str()

    def get_gripper_cartesian_pose(self):
        return self.arm.get_gripper_cartesian_pose()

    def get_gripper_clip_pose_rel(self):
        return self.arm.get_gripper_clip_pose_rel()

    def get_finger_pose(self) -> (str, str):
        left, right = self.arm.get_finger_pose()
        return PoseWrap.from_ros_pose(left).to_json_str(), PoseWrap.from_ros_pose(right).to_json_str()

    def reset_arm_retract(self):
        self.arm.reset_arm_retract()


def patch():
    # enable ZeroRPC-msgpack to serialize numpy ndarray.
    # see: https://pypi.org/project/msgpack-numpy/
    #      https://github.com/0rpc/zerorpc-python/pull/187/
    msgpack_numpy.patch()
    rospy.loginfo('[monoclone:camera] money patch done')


def graceful_shutdown(threads: List[gevent.Greenlet]):
    # ZeroRPC and rospy has a conflicting issue on exiting,
    # but it's ok to ignore these errors for all processes would eventually exit with a non-graceful manner.
    # see: https://www.cnblogs.com/flipped/p/15500530.html
    for t in threads:
        t.kill()
    rospy.loginfo('[monoclone:main] ZeroRPC Server exited.')


def main():
    patch()
    rospy.init_node('monoclone_ros_teleop')
    task_name = 'peg_ins'
    # Set up ZeroRPC Server on port
    arm_server = zerorpc.Server(Arm())
    arm_server.bind(f'tcp://0.0.0.0:{config.RPC_PORT_ARM}')
    world_server = zerorpc.Server(peg_ins.World())
    world_server.bind(f'tcp://0.0.0.0:{config.RPC_PORT_WORLD}')

    arm_thread = gevent.spawn(arm_server.run)
    world_thread = gevent.spawn(world_server.run)
    # register shutdown hooks for zerorpc server to gracefully exit
    gevent.signal_handler(signal.SIGINT, lambda: graceful_shutdown([arm_thread, world_thread]), signal.SIGINT, None)
    rospy.loginfo('[monoclone:main] Teleop ZeroRPC server starting...')
    gevent.joinall([arm_thread, world_thread])


if __name__ == '__main__':
    main()
    # test()
