import rospy
from datetime import datetime
import random as rand

from .. import config, gazebo_api as gzapi, utils


class World(object):
    def __init__(self, reset=True):
        self.config = config.task
        rospy.loginfo('[monoclone:task:peg_ins] start init world')
        if reset:
            self.reset()
        rospy.loginfo('[monoclone:task:peg_ins] done init world')

    # reset function is designed to be idempotent
    def reset(self,
              random_peg=False,
              random_hole=False,
              peg_init_pose: str = 'default',
              hole_init_pose: str = 'default'):
        if peg_init_pose == 'default':
            peg_init_pose = self.config.peg.init_pose
        else:
            peg_init_pose = utils.PoseWrap.from_json_str(peg_init_pose)
        if hole_init_pose == 'default':
            hole_init_pose = self.config.hole.init_pose
        else:
            hole_init_pose = utils.PoseWrap.from_json_str(hole_init_pose)
        if random_peg:
            peg_init_pose.position.x += rand.uniform(*self.config.peg.init_range_x)
            peg_init_pose.position.y += rand.uniform(*self.config.peg.init_range_y)
        if random_hole:
            hole_init_pose.position.x += rand.uniform(*self.config.hole.init_range_x)
            hole_init_pose.position.y += rand.uniform(*self.config.hole.init_range_y)
        gzapi.respawn_model(self.config.peg.model_name, self.config.peg.model_path, peg_init_pose)
        gzapi.respawn_model(self.config.hole.model_name, self.config.hole.model_path, hole_init_pose)

    # plz expand this function to return positions of 2 legs
    def get_peg_legtip_pos(self) -> ([int, int, int], [int, int, int]):
        rospy.loginfo(f'[monoclone:teleop:peg-ins] get_peg_legtip_pos at {datetime.now()}')
        response: gzapi.gz_srv.GetLinkStateResponse = gzapi.get_link_state('peg_leg_1', 'world')
        position = response.link_state.pose.position
        posvec_1L = [position.x, position.y, position.z]
        response: gzapi.gz_srv.GetLinkStateResponse = gzapi.get_link_state('peg_leg_2', 'world')
        position = response.link_state.pose.position
        posvec_2R = [position.x, position.y, position.z]

        orientation = response.link_state.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        # print('pose: ', posvec_1L, '\n''orient: ', quaternion)  # NOTE debug

        # rotate offset vector by orientation quaternion
        result = (posvec_1L + utils.Math.rotate_vector(self.config.peg.leg_tip_offset_vec, quaternion),
                  posvec_2R + utils.Math.rotate_vector(self.config.peg.leg_tip_offset_vec, quaternion))
        rospy.loginfo(f'[monoclone:teleop:peg-ins] '
                      f'peg_leg_tip point : {result}')
        return result

    def get_hole_surface_center(self) -> ([int, int, int], [int, int, int]):
        rospy.loginfo(f'[monoclone:teleop:peg-ins] get_hole_surface_center at {datetime.now()}')
        response: gzapi.gz_srv.GetModelStateResponse = gzapi.get_model_state(self.config.hole.model_name, 'world')
        position = response.pose.position
        orientation = response.pose.orientation
        posvec = [position.x, position.y, position.z]
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        # rotate offset vector by orientation quaternion
        result = (posvec + utils.Math.rotate_vector(self.config.hole.hole_surface_offset_vec_L, quaternion),
                  posvec + utils.Math.rotate_vector(self.config.hole.hole_surface_offset_vec_R, quaternion))
        return result

    def get_peg_pose(self) -> str:
        response = gzapi.get_model_state(self.config.peg.model_name, 'world')
        return utils.PoseWrap.from_ros_pose(response.pose).to_json_str()

    def get_hole_pose(self) -> str:
        response = gzapi.get_model_state(self.config.hole.model_name, 'world')
        return utils.PoseWrap.from_ros_pose(response.pose).to_json_str()
