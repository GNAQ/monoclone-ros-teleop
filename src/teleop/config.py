from . import utils
import numpy as np


class BaseConfig(dict):
    def __init__(self):
        super(BaseConfig, self).__init__()

    def __getattr__(self, key):
        return self.get(key)

    def __setattr__(self, key, value):
        self.__setitem__(key, value)

    def __setitem__(self, key, value):
        super(BaseConfig, self).__setitem__(key, value)
        self.__dict__.update({key: value})


class SimCamera(BaseConfig):
    def __init__(self):
        super(SimCamera, self).__init__()
        self.width = 256
        self.height = 256
        self.fps = 20

        # model files
        self.model_path = f'{MODEL_DIR}/simcam/model.sdf'
        self.model_name = 'simcam'
        self.init_pose = utils.PoseWrap(
            _position=utils.PoseWrap.Position(_x=-0.0086, _y=0.4543, _z=0.3355),
            _orientation=utils.PoseWrap.Orientation(
                _x=0.06010628491640091,
                _y=0.0786399394273758,
                _z=-0.5545494556427002,
                _w=0.8262433409690857)
        )


# Plug insertion task config
class PegInsTaskConfig(BaseConfig):
    def __init__(self):
        super().__init__()
        self.task_name = 'peg_ins'

        self.peg = BaseConfig()
        self.hole = BaseConfig()

        # model files
        self.peg.model_path = f'{MODEL_DIR}/{self.task_name}/peg/model.sdf'
        self.hole.model_path = f'{MODEL_DIR}/{self.task_name}/hole/model.sdf'
        self.peg.model_name = 'peg'
        self.hole.model_name = 'hole'

        # y-axis offset relative to the mesh file centre of plug's leg
        self.peg.leg_tip_offset_vec = [0., 0.0252, 0.]

        # offset of the hold front-surface center relative to its model central (bottom center)
        self.hole.hole_surface_offset_vec_L = [-0.0163, -0.0518, +0.1300]
        self.hole.hole_surface_offset_vec_R = [+0.0163, -0.0518, +0.1300]

        # initial pose of peg
        # rotate 90 deg counter-clockwise looking from -z to +z axis
        self.peg.init_pose = utils.PoseWrap(
            _position=utils.PoseWrap.Position(_x=0.360, _y=-0.120, _z=0.02),
            _orientation=utils.PoseWrap.Orientation(_x=0.0, _y=0.0, _z=-0.7048, _w=0.7048)
        )
        # initial pose of hole
        self.hole.init_pose = utils.PoseWrap(
            _position=utils.PoseWrap.Position(_x=0.55, _y=-0.1850, _z=0.0),
            _orientation=utils.PoseWrap.Orientation(_x=0.0, _y=0.0, _z=-0.7048, _w=0.7048)
        )
        # pose position random ranges
        self.peg.init_range_x = [-0.10, +0.10]
        self.peg.init_range_y = [-0.15, +0.15]
        self.hole.init_range_x = [0, 0]
        self.hole.init_range_y = [-0.05, +0.05]


def init_config(task_name):
    global task
    if task_name == 'peg_ins':
        task = PegInsTaskConfig()
    elif task_name == '':
        pass
    else:
        pass


RPC_PORT_ARM = 9089
RPC_PORT_CAM = 9090
RPC_PORT_WORLD = 9091
RPC_PORT_OBS = 9092
MODEL_DIR = '/monoclone/monoclone_ros/assets'
task = BaseConfig()
simcam = SimCamera()
