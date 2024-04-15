import json
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
from gripper_util import open_gripper, close_gripper, grasp
from scipy.spatial.transform import Rotation as R
import open3d as o3d
def tf2pose(trans, offset, rot=None):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = trans[0] + offset[0]
    pose_target.position.y = trans[1] + offset[1]
    pose_target.position.z = trans[2] + offset[2]
    if rot is not None:
        pose_target.orientation.x = rot[0]
        pose_target.orientation.y = rot[1]
        pose_target.orientation.z = rot[2]
        pose_target.orientation.w = rot[3]
    else:
        pose_target.orientation.w = 1.000000
    return pose_target

def execute_taxpose(json_file, listener, scene_pcd):
    group = moveit_commander.MoveGroupCommander("panda_manipulator")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.2)
    robot = moveit_commander.RobotCommander()
    gripper_client = actionlib.SimpleActionClient(
        "/franka_gripper/gripper_action", GripperCommandAction
    )
    # Step 2 Grasp the object
    # close_gripper(gripper_client)
    grasp()

    saved_tf = json.load(open(json_file, 'r'))
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
    # R_pred = saved_tf['R_rel']
    # t_pred = saved_tf['t_rel']
    R_pred = np.asarray(saved_tf['R'])
    t_pred = np.asarray(saved_tf['t'])
    ee_rot_mat = R.from_quat(ee_rot).as_matrix()
    # Start
    ee_tf_start = np.eye(4)
    ee_tf_start[:3, :3] = ee_rot_mat
    ee_tf_start[:3, -1] = ee_trans
    #Taxpose
    ee_taxpose = np.eye(4)
    ee_taxpose[:3, :3] = R_pred
    ee_taxpose[:3, -1] = t_pred
    # goal pose
    ee_goalpose = ee_taxpose@ee_tf_start
    ee_goal_R = R.from_matrix(ee_goalpose[:3, :3]).as_quat()
    ee_goal_t = ee_goalpose[:3, -1]

    # Visualize the start pose and goal pose as frames in open3d
    def create_frame(R, t):
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1
        )

        mesh_frame.rotate(R)
        mesh_frame.translate(t)
        return mesh_frame

    start_frame = create_frame(ee_tf_start[:3, :3], ee_tf_start[:3, -1])
    goal_frame = create_frame(ee_goalpose[:3, :3], ee_goalpose[:3, -1])
    o3d.visualization.draw_geometries([scene_pcd, start_frame, goal_frame])

    wpose = tf2pose(ee_goal_t, [-0.06,-0.03,0.08], ee_goal_R)
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

    input("Press Enter to continue with the plan...")

    plan = group.go(wait=True)
    group.clear_pose_targets()
    rospy.loginfo("============ TAX-Pose Reached ===========\n")

    input("Press Enter to continue with the next phase...")


    wpose = tf2pose(ee_goal_t, [0,0,0], ee_goal_R)
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

    plan = group.go(wait=True)
    group.clear_pose_targets()
    rospy.loginfo("============ TAX-Pose Reached ===========\n")
    open_gripper(gripper_client)


def main():
    execute_taxpose()

if __name__ == "__main__":
    
    rospy.init_node("pcd_test")
    listener = tf.TransformListener()
    main()