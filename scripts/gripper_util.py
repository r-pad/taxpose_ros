import rospy
import actionlib
import tf
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import String
import numpy as np
import franka_gripper.msg
"""
This file tests the gripper (open <--> close)
"""


def get_fingers_pos(listener):
    # Getting marker transformation
    while not rospy.is_shutdown():
        try:
            (left_trans, left_rot) = listener.lookupTransform(
                "/world", "/panda_leftfinger", rospy.Time(0)
            )
            (right_trans, right_rot) = listener.lookupTransform(
                "/world", "/panda_rightfinger", rospy.Time(0)
            )
            width = np.linalg.norm(np.array(left_trans) - np.array(right_trans))
            print(f"Gripper width: {width}")
            break
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue
    return width


def close_gripper(gripper_client):
    gripper_client.wait_for_server()
    rospy.loginfo("gripper action server started.")
    goal = GripperCommandGoal()
    goal.command.position = 0
    gripper_client.send_goal(goal)
    rospy.loginfo("goal sent.")
    # res = gripper_client.wait_for_result()


def close_gripper_with_effort(gripper_client):
    listener = tf.TransformListener()
    gripper_client.wait_for_server()
    rospy.loginfo("gripper action server started.")
    goal = GripperCommandGoal()
    goal.command.max_effort = 1e6
    gripper_client.send_goal(goal)
    rospy.loginfo("effort goal sent.")
    # width = get_fingers_pos(listener)
    # goal = GripperCommandGoal()
    # goal.command.position = width - 0.01
    # gripper_client.send_goal(goal)
    # rospy.loginfo('position goal sent.')
    # res = gripper_client.wait_for_result()


def open_gripper(gripper_client):
    gripper_client.wait_for_server()
    rospy.loginfo("gripper action server started.")
    goal = GripperCommandGoal()
    goal.command.position = 0.039
    gripper_client.send_goal(goal)
    rospy.loginfo("goal sent.")
    # res = gripper_client.wait_for_result()
def grasp(width=0, speed=0.1, force=1):
    client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()
    client.send_goal(
        franka_gripper.msg.GraspGoal(
            width,
            franka_gripper.msg.GraspEpsilon(0.1, 0.1),
            speed,
            force
        )
    )
    return client.wait_for_result()

def main():
    rospy.init_node("grippter_test")
    listener = tf.TransformListener()
    is_open = False
    width = get_fingers_pos(listener)
    is_open = width < 0.035
    while not rospy.is_shutdown():

        gripper_client = actionlib.SimpleActionClient(
            "/franka_gripper/gripper_action", GripperCommandAction
        )
        gripper_client.wait_for_server()
        if is_open:
            print("Closing")
            close_gripper(gripper_client)
        else:
            print("Opening")
            open_gripper(gripper_client)
        rospy.loginfo("goal sent.")
        # res = gripper_client.wait_for_result()
        input("Hit enter to toggle gripper: ")
        is_open = not is_open


if __name__ == "__main__":
    main()
