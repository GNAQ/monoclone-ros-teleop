import rospy
import time
import gazebo_msgs.srv as gz_srv
import gazebo_msgs.msg as gz_msg
from . import utils

# check gz_srv.XXX for call args and return values

# get link state service
get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', gz_srv.GetLinkState, persistent=True)

# get model state
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', gz_srv.GetModelState, persistent=True)

# delete mesh model
delete_model = rospy.ServiceProxy("/gazebo/delete_model", gz_srv.DeleteModel, persistent=True)

# spawn mesh model
spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", gz_srv.SpawnModel, persistent=True)

# unwrapped set model state
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', gz_srv.SetModelState, persistent=True)


def respawn_model(model_name: str, model_path: str, init_pose: utils.PoseWrap):
    rospy.loginfo(f'[monoclone:gz_api] start respawn {model_name}')

    # delete old model
    rospy.wait_for_service('/gazebo/delete_model')
    result: gz_srv.DeleteModelResponse = delete_model(model_name)
    if not result.success and result.status_message != 'DeleteModel: model does not exist':
        raise RuntimeError(f'[monoclone:gz_api] reset {model_name} failed!')
    time.sleep(0.25)

    # spawn new model
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    with open(model_path, 'r') as model_file:
        sdf_context = model_file.read()
        result: gz_srv.SpawnModelResponse = spawn_sdf_model(
            model_name, sdf_context, '', init_pose.to_ros_pose(), 'world')
        if not result.success:
            raise RuntimeError(f'[monoclone:gz_api] spawned model {model_name} failed!')
    time.sleep(0.25)

    rospy.loginfo(f'[monoclone:gz_api] respawn {model_name} done with pose {init_pose}')
