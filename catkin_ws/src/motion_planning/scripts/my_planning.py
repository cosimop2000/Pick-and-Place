#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import control_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.msg import ModelStates
import tf.transformations

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def simple_pick_place(pick_pose, place_pose):
    """
    This function plans and executes the movements required to pick an object from
    'pick_pose' and place it at 'place_pose'.
    """
    group.set_pose_reference_frame(reference_frame)
    
    #group.set_position_target([pick_pose.position.x, pick_pose.position.y, pick_pose.position.z])
    #group.set_orientation_target(quaternion)
    group.set_pose_target(pick_pose)

    group.set_start_state_to_current_state()
    # Move to the pick position
    
    #group.set_pose_target(pick_pose)
    print(pick_pose)
    group.set_planning_time(60)  # Increase planning time
    #plan1 = group.go(wait=True)
    #group.stop()

    plan1 = group.plan()
    #print(plan1)
    group.execute(plan1[1], wait=True)
    group.clear_pose_targets()


    if not plan1:
        rospy.logerr("Failed to plan to pick position")
        return
    # Close the gripper
    open_gripper()
    close_gripper(gazebo_model_name)
    
    #return
    # Move to the place position
    group.set_pose_target(place_pose)
    print(place_pose)
    group.set_planning_time(50)  # Increase planning time
    plan2 = group.plan()
    #print(plan1)
    group.execute(plan2[1], wait=True)
    group.clear_pose_targets()

    open_gripper()

    if not plan2:
        rospy.logerr("Failed to plan to place position")
        return

def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    #print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)

def close_gripper(gazebo_model_name, closure=0):
    set_gripper(0.51-closure*10)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)

def get_poses():
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    #print(models)
    obj_poses = ModelStates()
    #print(obj_poses)

    for name, pose in zip(models.name, models.pose):
        if ("sphere" == name) or ("cube" in name) or ("cylinder" == name):

            obj_poses.name.append(name)
            obj_poses.pose.append(pose)
    
    #print(obj_poses)

    return [(i, j) for i, j in zip(obj_poses.name, obj_poses.pose)]

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = -1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(2))

    return action_gripper.get_result()

def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)

def info_show():
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print(planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print(eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print(group_names)

    #print(robot.get_current_state())

def traj(group, scale=0.1):
    waypoints = []

    wpose = group.get_current_pose().pose
    print(wpose)
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    print(plan)
    print(fraction)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def add_table_to_scene():
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = -0.65
    table_pose.pose.position.z = -0.38
    table_pose.pose.orientation.w = 1.0

    table_size = [2.5, 1.0, 0.58]
    
    scene.add_box("table", table_pose, table_size)

def add_base_to_scene():
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = -0.36
    table_pose.pose.orientation.w = 1.0

    table_size = [0.35, 0.09, 0.72]
    
    scene.add_box("base", table_pose, table_size)


def convert_to(pose):
    pass

if __name__ == "__main__":
    print("Initializing node of kinematics")
    #rospy.init_node("send_joints")


    # Initialize MoveIt! commander
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('banana')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Initialize the group for the manipulator
    group = moveit_commander.MoveGroupCommander("arm")

    group.set_planner_id("RRT")

    planning_frame = group.get_planning_frame()
    end_effector_link = group.get_end_effector_link()
    print(end_effector_link)
    group_names = robot.get_group_names()

    current_pose = group.get_current_pose().pose
    print(current_pose)

    group.set_goal_orientation_tolerance(0.15)

    """display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    """
    
    #info_show()
    
    """
    for i in range(100):
        plan, fraction = traj(group)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        group.execute(plan, wait=True)
    print('prova ok')
    """
    
    
    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()
    
    """
    """
    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    # Add the table to the planning scene
    #add_table_to_scene()
    #add_base_to_scene()
    rospy.sleep(5)  # Allow some time for the table to be added to the scene
    
    # Define the reference frame
    reference_frame = "base_link"  # Replace with your reference frame
    group.set_pose_reference_frame(reference_frame)

    obj_poses = get_poses()

    for model_name, model_pose in obj_poses:
        #open_gripper()

        # Get actual model_name at model xyz coordinates
        try:
           gazebo_model_name = get_gazebo_model_name(model_name, model_pose)
        except ValueError as e:
            print(e)
            continue
        
        print(model_pose)
        #break
        
        #group.set_workspace([-10,-10,0,10,10,10])
        #group.set_start_state_to_current_state() 	
        group.clear_path_constraints() 	
        
        #destination = (0, -0.5, 0.84)

        #group.set_pose_reference_frame(reference_frame)
        
        # Convert Euler angles (-90, -90, 0) to quaternion
        roll = 0.0 * (3.141592653589793 / 180.0)  # Convert degrees to radians
        pitch = +90.0 * (3.141592653589793 / 180.0)  # Convert degrees to radians
        yaw = +90.0 * (3.141592653589793 / 180.0)  # Convert degrees to radians
        
        # Define pick and place poses
        pick_pose = geometry_msgs.msg.Pose()
        #pick_pose.position = model_pose.position
        #pick_pose.orientation = model_pose.orientation
        #print(model_pose.position)
        #print(model_pose.orientation)

        quaternion = euler_to_quaternion(roll, pitch, yaw)

        print(quaternion)

        pick_pose.orientation.w= quaternion[3]
        pick_pose.orientation.x= quaternion[0]      #model_pose.orientation.x
        pick_pose.orientation.y= quaternion[1] #model_pose.orientation.y
        pick_pose.orientation.z= quaternion[2] #model_pose.orientation.z
        
        #print(pick_pose.orientation)
        pick_pose.position.x = model_pose.position.x
        pick_pose.position.y = model_pose.position.y
        pick_pose.position.z = model_pose.position.z - 0.743487 + 0.3
        #pick_pose.position.x = 0.1
        #pick_pose.position.y = -0.3
        #pick_pose.position.z = 0.01

        #place_pose = copy.deepcopy(pick_pose)
        place_pose = geometry_msgs.msg.Pose()
        place_pose = pick_pose

        place_pose.position.z = 0.0

        # Perform the pick and place
        simple_pick_place(pick_pose, place_pose)
        break
