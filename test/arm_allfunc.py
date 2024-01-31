import json
import msgpack_numpy
import zerorpc


def test_arm():
    arm = zerorpc.Client()
    arm.connect('tcp://127.0.0.1:9089')

    # check reset
    arm.reset_arm_retract()

    # check gets
    result = arm.get_finger_pose()
    print(result)

    result = arm.get_gripper_clip_pose_rel()
    print(result)

    result = arm.get_cartesian_pose_end_effector()
    print(result)

    # check sets
    cur_pose = arm.get_cartesian_pose_end_effector()
    cur_pose = json.loads(cur_pose)
    cur_pose['position']['x'] += 0.1
    cur_pose['position']['y'] += 0.1
    cur_pose = json.dumps(cur_pose)
    arm.set_cartesian_pose(cur_pose, 0.01)

    cur_pose = json.loads(cur_pose)
    cur_pose['position']['x'] -= 0.1
    cur_pose['position']['y'] += 0.1
    cur_pose['position']['z'] += 0.1
    cur_pose = json.dumps(cur_pose['position'])
    arm.set_cartesian_position(cur_pose, 0.01)

    arm.set_gripper_clip_pose_rel(0.1)
    arm.set_gripper_clip_pose_rel(1)
    arm.set_gripper_clip_pose_rel(0)

    arm.set_reach_named_position('home')
    arm.set_reach_named_position('retract')
    arm.set_reach_named_position('vertical')
    arm.set_reach_named_position('home')


if __name__ == '__main__':
    msgpack_numpy.patch()
    test_arm()
    print('arm test done.')
