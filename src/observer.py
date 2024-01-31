import rospy
import zerorpc
import msgpack_numpy

from teleop import kinova_arm as karm, config
from teleop.utils import PoseWrap
from teleop.task import peg_ins


class Observer(object):
    def __init__(self):
        self.arm = karm.Gen3ArmWithMoveIt()
        self.task_meta = None

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

    def update_task_process(self, _task_process: dict):
        self.task_meta = _task_process

    def get_task_process(self):
        return self.task_meta


class PegInsObserver(Observer):
    def __init__(self):
        super().__init__()
        self.world = peg_ins.World(reset=False)

    def get_peg_legtip_pos(self):
        return self.world.get_peg_legtip_pos()

    def get_hole_surface_center(self):
        return self.world.get_hole_surface_center()

    def get_peg_pose(self):
        return self.world.get_peg_pose()

    def get_hole_pose(self):
        return self.world.get_hole_pose()

    def reset(self, random_peg=False, random_hole=False,
              peg_init_pose: str = 'default', hole_init_pose: str = 'default'):
        self.world.reset(random_peg, random_hole, peg_init_pose, hole_init_pose)


def patch():
    # enable ZeroRPC-msgpack to serialize numpy ndarray.
    # see: https://pypi.org/project/msgpack-numpy/
    #      https://github.com/0rpc/zerorpc-python/pull/187/
    msgpack_numpy.patch()
    rospy.loginfo('[monoclone:observer] money patch done')


def graceful_shutdown(server: zerorpc.Server):
    # ZeroRPC and rospy has a conflicting issue on exiting,
    # but it's ok to ignore these errors for all processes would eventually exit with a non-graceful manner.
    # see: https://www.cnblogs.com/flipped/p/15500530.html
    server.close()
    rospy.loginfo('[monoclone:observer] ZeroRPC Server exited.')


def main():
    patch()
    rospy.init_node('monoclone_ros_obs')
    task_name = 'peg_ins'
    # Set up ZeroRPC Server on port
    server = zerorpc.Server(PegInsObserver())
    server.bind(f'tcp://0.0.0.0:{config.RPC_PORT_OBS}')

    # register shutdown hooks for zerorpc server to gracefully exit
    rospy.on_shutdown(lambda: graceful_shutdown(server))
    rospy.loginfo('[monoclone:observer] Observer server starting...')
    server.run()


if __name__ == '__main__':
    main()
