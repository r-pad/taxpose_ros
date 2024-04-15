from os import listdir
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import tf
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import numpy as np
import pickle
import ros_numpy
import trimesh
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import pickle
import requests
import moveit_commander
import geometry_msgs
import actionlib
from gripper_util import open_gripper, close_gripper, grasp, get_fingers_pos


def remote_inference():
    #### CALLS REMOTE INFERENCE ####
    address = "128.2.179.162"
    data = {'msg': 'hi'}
    x = pickle.dumps(data, 2)
    endpoint = "http://%s:7788/%s" % (address, "rollout")
    rospy.loginfo("Calling remote inference...\n")
    r = requests.post(endpoint, data=x)
    content = pickle.loads(r.content)
    pickle.dump(content, open('./temp_trans.pkl', 'wb'))
    R = content['R_rel']
    t = content['t_rel']
    rospy.loginfo(f"TAX-Pose received: {R, t}")
    ###############################

    usr_input = input("Continue? ( y for yes )")
    while usr_input.lower() != "y" and usr_input.lower() != "n":
        usr_input = input("Please enter a valid option. (y/n)").lower()
    if usr_input == "n":
        rospy.loginfo("Aborting...")
        exit(0)
    ee_x = float(input("Enter grasp x : "))
    ee_y = float(input("Enter grasp y : "))
    ee_z = float(input("Enter grasp z : "))
    return R, t, [ee_x, ee_y, ee_z]

def grasp_primitive(trans, offset=0):
    """
    Right grasp
    """
    rot = [
            -0.7216643478456781,
            -0.6908384757139052,
            0.0424137523377903,
            -0.011993462151892849,
        ]
    """
    Front grasp
    """
    # rot = [-0.9995630217413629, 0.020709667659615627, -0.0033584965246435516, -0.02082296169610565]
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = trans[0]
    pose_target.position.y = trans[1]
    pose_target.position.z = trans[2] + offset
    pose_target.orientation.x = rot[0]
    pose_target.orientation.y = rot[1]
    pose_target.orientation.z = rot[2]
    pose_target.orientation.w = rot[3]
    return pose_target, rot


def execute_traj_grasp(ee_pos, listener):
    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planner_id("RRTstarkConfigDefault")
    # group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(5.0)
    robot = moveit_commander.RobotCommander()

    gripper_client = actionlib.SimpleActionClient(
            "/franka_gripper/gripper_action", GripperCommandAction
        )
    gripper_client.wait_for_server()

    open_gripper(gripper_client)
    

    # Step 1 go to pregrasp pose
    rospy.loginfo("============ Waypoints Received ===========\n")
    while not rospy.is_shutdown():
        try:
            (ee_trans, ee_rot) = listener.lookupTransform(
                "/world", "/panda_hand_tcp", rospy.Time(0)
            )
            print("EE translation: ", ee_trans)
            print("EE rotation: ", ee_rot)
            break
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue
    wpose, grasp_rot = grasp_primitive(ee_pos, 0.09)
    group.set_start_state(robot.get_current_state())
    group.set_pose_target(wpose)
    succ = False
    budget = 0
    while not succ and budget < 1:
        rospy.loginfo("Planning")
        plan = group.plan()
        succ = plan[0]
        budget += 1
    if not succ:
        group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        rospy.logerr("============ MP Failed!! ===========\n")
        rospy.logerr("Exiting prematurely")

    plan = group.go(wait=True)
    group.clear_pose_targets()
    rospy.loginfo("============ Pre-Grasp Pose Reached ===========\n")

    input("Press enter to continue...")
    
    wpose, grasp_rot = grasp_primitive(ee_pos, 0.01)
    group.set_start_state(robot.get_current_state())
    group.set_pose_target(wpose)
    succ = False
    budget = 0
    while not succ and budget < 1:
        rospy.loginfo("Planning")
        plan = group.plan()
        succ = plan[0]
        budget += 1
    if not succ:
        group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        rospy.logerr("============ MP Failed!! ===========\n")
        rospy.logerr("Exiting prematurely")

    plan = group.go(wait=True)
    group.clear_pose_targets()

    rospy.loginfo("============ Grasp Pose Reached ===========\n")

    # close_gripper(gripper_client)




def main():
   
    # R_pred, t_pred, ee_pos = remote_inference()
    # rospy.signal_shutdown(
    #     "GoalInf Neural Model finished running. Exiting gracefully..."
    # )

    ee_pos = [.5, -.2, .1]
    execute_traj_grasp(ee_pos)


if __name__ == "__main__":

    import argparse, os

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, default="taxpose-mug")

    args = parser.parse_args()
    trialname = args.name

    # Creating result directory based on arguments
    if not os.path.exists(os.path.join(os.getcwd(), trialname)):
        os.makedirs(os.path.join(os.getcwd(), trialname))
    trialnames = os.listdir(os.path.join(os.getcwd(), trialname))
    if len(trialnames) == 0:
        ind = 0
    else:
        base = max([int(name[:3]) for name in trialnames])
        trial_num = "%03d" % base
        _dir = os.path.join(os.getcwd(), trialname, trial_num)
        if len(os.listdir(_dir)) == 0:
            ind = int(base)
        else:
            ind = int(base) + 1
    trial_num = "%03d" % ind
    result_dir = os.path.join(os.getcwd(), trialname, trial_num)
    if not os.path.exists(result_dir):
        print("Creating result directory...")
        os.makedirs(result_dir)
        print("Created result directory")

    # ROS Stuff
    rospy.init_node("pcd_test")
    listener = tf.TransformListener()


    # rospy.init_node("grippter_test")
    # listener = tf.TransformListener()


    main()
