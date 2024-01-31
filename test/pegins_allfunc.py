import json
import msgpack_numpy
import zerorpc


def test_pegins():
    world = zerorpc.Client()
    world.connect("tcp://127.0.0.1:9091")

    # test reset
    world.reset()

    # test gets
    result = world.get_peg_pose()
    print(result)

    result = world.get_hole_pose()
    print(result)

    result = world.get_peg_legtip_pos()
    print(result)

    result = world.get_hole_surface_center()
    print(result)

    # test reset with params
    peg_pose = world.get_peg_pose()
    peg_pose = json.loads(peg_pose)
    peg_pose['position']['x'] += 1
    peg_pose['position']['y'] += 1
    peg_pose = json.dumps(peg_pose)
    world.reset(False, True, peg_pose, 'default')


if __name__ == '__main__':
    msgpack_numpy.patch()
    test_pegins()
    print('test peg-ins done.')
