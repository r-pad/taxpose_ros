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
from gripper_util import open_gripper, close_gripper
from video_util import VideoRecorder

from subprocess import Popen

def pcd_callback(data, world_transform):
    # CB function for pointcloud recording
    new_data = do_transform_cloud(data, world_transform)

    pc = ros_numpy.numpify(new_data)
    points = np.zeros((pc.shape[0], 3))
    points[:, 0] = pc["x"]
    points[:, 1] = pc["y"]
    points[:, 2] = pc["z"]

    nonnan_pts = points[~np.isnan(points).any(axis=1)]
    return nonnan_pts[::10]

def pcd_collection():
    rospy.loginfo("Collecting pointcloud from K4A.....")
    point_clouds = {}
    for j in range(3):
        # data = rospy.wait_for_message(f"/k4a_{j}/points2", PointCloud2)
        data = rospy.wait_for_message(f"/k4a_{j}/depth_registered/points_filtered", PointCloud2)
        rospy.loginfo("got point cloud")
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        while not rospy.is_shutdown():
            try:
                world_transform = buffer.lookup_transform(
                    "panda_link0", data.header.frame_id, 
                    rospy.Time.now(), rospy.Duration(0.1)
                )
                print("World transformation: ", world_transform)
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                print(e)
                continue

        rospy.loginfo(f"got world transform\n")

        # publisher = rospy.Publisher("points2_nonnan_filtered", PointCloud2, queue_size=10)
        np_pc = pcd_callback(data, world_transform)

        rospy.loginfo(f"Collected {len(np_pc)} points\n")

        point_clouds[f'k4a_{j}'] = np_pc
    return point_clouds

def goal_inference(points):
    
    return waypoints, pcd_pub_p


def transform2pose(trans, rot=None):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = trans[0]
    pose_target.position.y = trans[1]
    pose_target.position.z = trans[2]
    if rot is not None:
        pose_target.orientation.x = rot[0]
        pose_target.orientation.y = rot[1]
        pose_target.orientation.z = rot[2]
        pose_target.orientation.w = rot[3]
    else:
        pose_target.orientation.w = 1.000000
    return pose_target

def execute_traj_grasp(waypoints, pcd_pub_p):
    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.2)
    # group.set_planner_id("RRTstarkConfigDefault")
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(5.0)
    robot = moveit_commander.RobotCommander()
    gripper_client = actionlib.SimpleActionClient(
        "/franka_gripper/gripper_action", GripperCommandAction
    )

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
    wpose, grasp_rot = grasp_primitives(waypoints[0], ACTION)
    # wpose = tf2pose(waypoints[-1], [0, 0, 0.05], ee_rot)
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
        open_gripper(gripper_client)
        rospy.logerr("Exiting prematurely")
        pcd_pub_p.kill()

    # Video stuff
    if shoot_video:
        recorder.start_movie()

    plan = group.go(wait=True)
    group.clear_pose_targets()
    rospy.loginfo("============ Pre-Grasp Pose Reached ===========\n")
    if ACTION == "cube":
        # Cube
        OFFSET = [-0.01, 0.03, 0.02]
    else:
        # Bowl
        OFFSET=[-0.03, 0.04, 0.03]

    if len(waypoints[0]) ==3:
        wpose = tf2pose(waypoints[0], OFFSET, grasp_rot)
    else:
        wpose = tf2pose(waypoints[0][:3], OFFSET, grasp_rot)
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
        open_gripper(gripper_client)
        rospy.logerr("Exiting prematurely")
        pcd_pub_p.kill()

    plan = group.go(wait=True)
    group.clear_pose_targets()
    rospy.loginfo("============ Pre-Grasp Pose Reached ===========\n")


def execute_traj_goal(waypoints, pcd_pub_p):
    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.2)
    # group.set_planner_id("RRTstarkConfigDefault")
    # group.set_planner_id("RRTConnectkConfigDefault")
    group.set_goal_tolerance(0.01)
    group.set_planning_time(5.0)
    robot = moveit_commander.RobotCommander()
    gripper_client = actionlib.SimpleActionClient(
        "/franka_gripper/gripper_action", GripperCommandAction
    )
    # Step 2 Grasp the object
    close_gripper(gripper_client)
    usr_input = input("Object grasp succeeded? ( y for yes )")
    while usr_input.lower() != "y" and usr_input.lower() != "n":
        usr_input = input("Please enter a valid option. (y/n)").lower()
    if usr_input == "n":
        rospy.logerr("Aborting...")
        open_gripper(gripper_client)
        pcd_pub_p.kill()
        if shoot_video:
            recorder.stop_movie()
            recorder.exit()
        exit(0)

    # Step 3 Go to Goal pose
    while not rospy.is_shutdown():
        try:
            (ee_trans, ee_rot) = listener.lookupTransform(
                "/world", "/panda_hand_tcp", rospy.Time(0)
            )
            break
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue
    
    wpose_post = tf2pose(
        waypoints[-1], ee_rot
    )

    budget = 0
    succ = False
    while not succ and budget < 1:
        group.set_start_state(robot.get_current_state())
        
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(20.0)                                                                  
        # breakpoint()
        # while not rospy.is_shutdown():
        #     try:
        #         latesttime = rospy.Time.now()
        #         br.sendTransform((-waypoints[-1][0], -waypoints[-1][1], -waypoints[-1][2]), (waypoints[-1][3], waypoints[-1][4], waypoints[-1][5], waypoints[-1][6]), latesttime, "/world", "/desired_pose")

        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         continue
        #     rate.sleep()
        group.set_pose_target(wpose_post)

        rospy.loginfo("Planning")
        plan = group.plan()
        succ = plan[0]
        budget += 1
    if not succ:
        group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        rospy.logerr("============ MP Failed!! ===========\n")
        open_gripper(gripper_client)
        rospy.loginfo("Exiting prematurely")
        if shoot_video:
            recorder.stop_movie()
            recorder.exit()

    plan = group.go(wait=True)

    rospy.loginfo("============ Path Planned ===========\n")
    group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()
    rospy.loginfo("============ Path Executed ===========\n")
    open_gripper(gripper_client)
    rospy.loginfo("Execution Done.")


def main(cat, which_goal):
    points = pcd_collection()
    waypoints, pcd_pub_p = goal_inference(np_pc)
    execute_traj_grasp(waypoints, pcd_pub_p)
    execute_traj_goal(waypoints, pcd_pub_p)
    pcd_pub_p.kill()
    rospy.signal_shutdown(
        "GoalInf Neural Model finished running. Exiting gracefully..."
    )

if __name__ == "__main__":

    import argparse, os

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, default="mug_grasping_4cam")

    args = parser.parse_args()
    trialname = args.name


    # Creating result directory based on arguments
    output_path = os.path.join(os.getcwd(), trialname)

    if not os.path.exists(output_path):
        os.makedirs(output_path)
    trialnames = os.listdir(output_path)

    if len(trialnames) == 0:
        ind = 0
    else:
        base = max([int(name[:3]) for name in trialnames])
        trial_num = "%03d" % base
        _dir = os.path.join(os.getcwd(), trialname, f"{cat}_{ACTION}", which_goal, trial_num)
        if len(os.listdir(_dir)) == 0:
            ind = int(base)
        else:
            ind = int(base) + 1

    trial_num = "%03d" % ind

    result_dir = os.path.join(output_path, trial_num)
    if not os.path.exists(result_dir):
        print("Creating result directory...")
        os.makedirs(result_dir)
        print("Created result directory")

    # shoot_video = input("Record a video of this trial? (Y/n)") != "n"
    # if shoot_video:
        # recorder = VideoRecorder(result_dir)

    # ROS Stuff
    rospy.init_node("mug_grasping")

    listener = tf.TransformListener()

    # rate = rospy.Rate(10.0)
    # publisher = rospy.Publisher("scene_pcd", PointCloud2, queue_size=10)

    main()
