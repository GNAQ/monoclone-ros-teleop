import json
import geometry_msgs.msg
import scipy


class Math:
    # rotate a 3d vector `vec` as the `quaternion` param indicated
    @staticmethod
    def rotate_vector(vec, quaternion):
        return scipy.spatial.transform.Rotation.from_quat(quaternion).apply(vec)


# wrap teleop data objects, make it possible to transform between monoclone algorithm and ros-kortex interface
class PoseWrap(object):
    class Position(object):
        def __init__(self, _x=.0, _y=.0, _z=.0):
            self.x, self.y, self.z = _x, _y, _z

        @classmethod
        def from_json_str(cls, json_str):
            d = json.loads(json_str)
            return cls(_x=d['x'], _y=d['y'], _z=d['z'])

        def to_json_str(self):
            d = {'x': self.x, 'y': self.y, 'z': self.z}
            return json.dumps(d)

        @classmethod
        def from_ros_position(cls, p: geometry_msgs.msg.Point):
            return cls(_x=p.x, _y=p.y, _z=p.z)

        def to_ros_position(self):
            return geometry_msgs.msg.Point(x=self.x, y=self.y, z=self.z)

    class Orientation(object):
        def __init__(self, _x=.0, _y=.0, _z=.0, _w=.0):
            self.x, self.y, self.z, self.w = _x, _y, _z, _w

    def __init__(self, _position=Position(), _orientation=Orientation()):
        self.position = _position
        self.orientation = _orientation

    def __repr__(self):
        return self.to_json_str()

    def to_ros_pose(self) -> geometry_msgs.msg.Pose:
        return geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=self.position.x,
                                                                       y=self.position.y,
                                                                       z=self.position.z),
                                      orientation=geometry_msgs.msg.Quaternion(x=self.orientation.x,
                                                                               y=self.orientation.y,
                                                                               z=self.orientation.z,
                                                                               w=self.orientation.w))

    @classmethod
    def from_ros_pose(cls, pose: geometry_msgs.msg.Pose):
        return cls(_position=PoseWrap.Position(pose.position.x, pose.position.y, pose.position.z),
                   _orientation=PoseWrap.Orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                     pose.orientation.w))

    def to_json_str(self):
        d = {'position': {
            'x': self.position.x,
            'y': self.position.y,
            'z': self.position.z},
            'orientation': {
                'x': self.orientation.x,
                'y': self.orientation.y,
                'z': self.orientation.z,
                'w': self.orientation.w
            }}
        return json.dumps(d)

    @classmethod
    def from_json_str(cls, json_str):
        d = json.loads(json_str)
        return cls(_position=cls.Position(*d['position'].values()),
                   _orientation=cls.Orientation(*d['orientation'].values()))
