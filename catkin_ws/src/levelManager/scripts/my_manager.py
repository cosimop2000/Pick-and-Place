#!/usr/bin/python3
import rospy
import rosservice
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_sdf_model(model_path, model_name, model_pose, reference_frame="world"):
    with open(model_path, 'r') as file:
        model_xml = file.read()
        print(model_xml)

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=model_name,
                       model_xml=model_xml,
                       robot_namespace='/foo',
                       initial_pose=model_pose,
                       reference_frame=reference_frame)

def spawn_additional_objects(event):
    # Define paths and names for additional models
    ll = [-0.45, -0.55, -0.5]

    model_path_1 = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/sphere/model.sdf'
    model_name_1 = 'sphere'
    model_pose_1 = Pose(Point(ll[0], -0.50, 1), Quaternion(0, 0, 0, 1))

    model_path_2 = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/cube/model.sdf'
    model_name_2 = 'cube'
    model_pose_2 = Pose(Point(ll[1], -0.50, 1), Quaternion(0, 0, 0, 1))

    model_path_3 = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/cylinder/model.sdf'
    model_name_3 = 'cylinder'
    model_pose_3 = Pose(Point(ll[2], -0.60, 1), Quaternion(0, 0, 0, 1))

    model_path_add = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/cube/model.sdf'
    model_name_add = 'cube_1'
    model_pose_add = Pose(Point(-ll[0], -0.50, 1), Quaternion(0, 0, 0, 1))

    model_path_more = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/cube/model.sdf'
    model_name_more = 'cube_2'
    model_pose_more = Pose(Point(-ll[1], -0.50, 1), Quaternion(0, 0, 0, 1))

        # Spawn additional models
    spawn_sdf_model(model_path_1, model_name_1, model_pose_1)
    spawn_sdf_model(model_path_2, model_name_2, model_pose_2)
    spawn_sdf_model(model_path_3, model_name_3, model_pose_3)

    spawn_sdf_model(model_path_add, model_name_add, model_pose_add)
    spawn_sdf_model(model_path_more, model_name_more, model_pose_more)

if __name__ == '__main__':
    rospy.init_node('sdf_model_spawner')

    model_path_sur = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/ok_copy/model.sdf'
    model_name_sur = 'ok_copy'
    model_pose_sur = Pose(Point(-0.5, -0.55, 0.8), Quaternion(0, 0, 0, 1))  # Adjust pose as needed

    model_path_prova = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/surface/model.sdf'
    model_name_prova = 'surface'
    model_pose_prova = Pose(Point(0.55, -0.58, 0.8), Quaternion(0, 0, 0, 1))

    model_path = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/ok/model.sdf'
    model_name = 'ok'
    model_pose = Pose(Point(0.5, -0.55, 1), Quaternion(0, 0, 0, 1))

    model_path_human = rospkg.RosPack().get_path('levelManager') + '/box_and_objects_models/human/model.sdf'
    model_name_human = 'human'
    model_pose_human = Pose(Point(0, -1.50, 1), Quaternion(0, 0, 0, 1))

    try:
        if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
            rospy.wait_for_service('/gazebo/spawn_sdf_model')


        spawn_sdf_model(model_path_sur, model_name_sur, model_pose_sur)
        rospy.sleep(0.5)
        spawn_sdf_model(model_path_prova, model_name_prova, model_pose_prova)
        rospy.sleep(0.5)
        spawn_sdf_model(model_path, model_name, model_pose)
        print("Model spawned successfully.")

        #spawn_sdf_model(model_path_human, model_name_human, model_pose_human)

        # Schedule callback to spawn additional objects after 5 seconds
        rospy.Timer(rospy.Duration(1), spawn_additional_objects, oneshot=True)

        rospy.spin()

    except rosservice.ROSServiceIOException:
        print("No ROS master execution")
    except rospy.ROSInterruptException as err:
        print(err)
    except rospy.service.ServiceException:
        print("No Gazebo services in execution")
    except rospkg.common.ResourceNotFound:
        print("Package not found")
    except FileNotFoundError as err:
        print(f"Model file not found: {err}")
