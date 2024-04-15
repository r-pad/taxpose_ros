#!/usr/bin/python3

from os import listdir
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, Image
import tf
import numpy as np
import pickle
import ros_numpy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import pickle
import requests
import os

def convert_point_cloud2(cloud_msg, transform, downsample = 2):
    cloud_msg_trans = do_transform_cloud(cloud_msg, transform)
    print('Converting Cloud')
    cloud_arr = ros_numpy.numpify(cloud_msg_trans)
    # cloud_msg_np = ros_numpy.split_rgb_field(cloud_msg_np)

    points = np.zeros((cloud_arr.shape[0], 3))
    points[:, 0] = cloud_arr["x"]
    points[:, 1] = cloud_arr["y"]
    points[:, 2] = cloud_arr["z"]
    
    rgb_arr = cloud_arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)
    
    colors = np.zeros((rgb_arr.shape[0], 3))
    colors[:, 0] = r
    colors[:, 1] = g
    colors[:, 2] = b

    valid_mask = ~np.isnan(points).any(axis=1)
    points_valid = points[valid_mask]
    colors_valid = colors[valid_mask]
    return points_valid[::downsample], colors_valid[::downsample]

def convert_transform_stamped(trans_msg):
    trans_dict = {
        'translation':[
            trans_msg.transform.translation.x,
            trans_msg.transform.translation.y,
            trans_msg.transform.translation.z,
        ],
        'rotation':[
            trans_msg.transform.rotation.x,
            trans_msg.transform.rotation.y,
            trans_msg.transform.rotation.z,
            trans_msg.transform.rotation.w,
        ]
    }
    return trans_dict

def convert_transform_stamped_identity():
    trans_dict = {
        'translation':[
            0,0,0,0
        ],
        'rotation':[
            0,0,0,1
        ]
    }
    return trans_dict

class DataTransfer(object):
    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.address = "cubbins.rpad.cs.cmu.edu"
        self.port = 1958
    def data_collection(self):
        rospy.loginfo("Collecting pointcloud from K4A.....")

        while not rospy.is_shutdown():
            try:
                transform_ee = self.buffer.lookup_transform(
                    "panda_link0", 
                    "panda_EE", 
                    # cloud_msg.header.stamp,
                    rospy.Time.now(),
                    rospy.Duration(0.1)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                print(e)
                continue
        
        while not rospy.is_shutdown():
            try:
                transform_hand = self.buffer.lookup_transform(
                    "panda_link0", 
                    "panda_hand", 
                    # cloud_msg.header.stamp,
                    rospy.Time.now(),
                    rospy.Duration(0.1)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                print(e)
                continue

        trans_ee_dict = convert_transform_stamped(transform_ee)
        trans_hand_dict = convert_transform_stamped(transform_hand)

        sensor_data = {
            'transform_ee':trans_ee_dict,
            'transform_hand':trans_hand_dict,
        }
        
        for j in [0,1,2,3]:
            sensor_data[f'k4a_{j}'] = {}
            cloud_msg = rospy.wait_for_message(f"/k4a_{j}/depth_registered/points_filtered", PointCloud2)

            while not rospy.is_shutdown():
                try:
                    transform = self.buffer.lookup_transform(
                        "panda_link0", 
                        cloud_msg.header.frame_id, 
                        # cloud_msg.header.stamp,
                        rospy.Time.now(),
                        rospy.Duration(0.1)
                    )
                    break
                except (
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ) as e:
                    print(e)
                    continue

            points, colors = convert_point_cloud2(cloud_msg, transform)
            rospy.loginfo(f"Collected {len(points)} points\n")

            image_msg = rospy.wait_for_message(f"/k4a_{j}/rgb/image_rect_color", Image)
            image_arr = ros_numpy.numpify(image_msg)

            depth_msg = rospy.wait_for_message(f"/k4a_{j}/depth_registered/image_rect", Image)
            depth_arr = ros_numpy.numpify(depth_msg)

            trans_dict = convert_transform_stamped(transform)

            assert cloud_msg.header.frame_id == image_msg.header.frame_id
            assert cloud_msg.header.frame_id == depth_msg.header.frame_id

            sensor_data[f'k4a_{j}']['points'] = points
            sensor_data[f'k4a_{j}']['colors'] = colors
            sensor_data[f'k4a_{j}']['image'] = image_arr
            sensor_data[f'k4a_{j}']['depth'] = depth_arr
            sensor_data[f'k4a_{j}']['transform'] = trans_dict

        return sensor_data


    def send_data_to_remote(self, sensor_data):
        #### CALLS REMOTE INFERENCE ####
        
        msg = {
            "sensor_data": sensor_data
        }
        msg_pkl = pickle.dumps(msg, 2)
        endpoint = f"http://{self.address}:{self.port}/rollout"
        rospy.loginfo("Calling remote inference...\n")
        r = requests.post(endpoint, data=msg_pkl)
    
    def send_data_to_local(self, sensor_data, out_dir):
        #### CALLS REMOTE INFERENCE ####
        
        msg = {
            "sensor_data": sensor_data
        }
        trialnames = os.listdir(out_dir)
        ind = len(trialnames)

        pickle.dump(msg, open(os.path.join(out_dir, f'{ind}.pkl'), 'wb'))
    
    def __call__(self, out_dir):
        sensor_data = self.data_collection()
        # self.send_data_to_remote(sensor_data)
        self.send_data_to_local(sensor_data, out_dir)

if __name__ == "__main__":
    cwd = os.getcwd()
    data_dir = os.path.join(cwd, "data/exp_pcd")
    os.makedirs(data_dir, exist_ok=True)
    # ROS Stuff
    rospy.init_node("sensor_data_transfer")
    data_transfer_node = DataTransfer()
    rate = rospy.Rate(10.0)
    
    data_transfer_node(data_dir)
