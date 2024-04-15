import datetime
import os
import subprocess
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import pyransac3d as pyrsc
import matplotlib.pyplot as plt
import tf
import typer
from tax_pose_mug_p2 import execute_taxpose

def get_point_cloud():
    rospy.loginfo("Waiting for point cloud...")
    data = rospy.wait_for_message(f"/points_concatenated", PointCloud2)
    rospy.loginfo("got point cloud")

    assert data.header.frame_id == "world"

    pc_dict = ros_numpy.numpify(data)

    points = np.zeros((pc_dict.shape[0], 3))
    points[:, 0] = pc_dict["x"]
    points[:, 1] = pc_dict["y"]
    points[:, 2] = pc_dict["z"]

    # How to get rgb?
    # rgb = ???

    return points

def clean_pcd(pcd, threshold=0.015):

    # Downsample to 100k points.
    if pcd.shape[0] > 100000:
        pcd = pcd[np.random.choice(pcd.shape[0], 100000, replace=False)]
    
    print("Downsampled to 100k points")
    # pc = o3d.geometry.PointCloud()
    # pc.points = o3d.utility.Vector3dVector(pcd)
    # o3d.visualization.draw_geometries([pc])

    

    # Filter out plane
    plane1 = pyrsc.Plane()
    _, best_inliers = plane1.fit(pcd, threshold)
    pcd = np.delete(pcd, best_inliers, 0)

    print("Deleted plane 1")
    # pc = o3d.geometry.PointCloud()
    # pc.points = o3d.utility.Vector3dVector(pcd)
    # o3d.visualization.draw_geometries([pc])


    # Remove statistical outliers.
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pcd)
    cl, ind = pc.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    pcd= pcd[ind]

    print("Deleted statistical outliers")
    # pc = o3d.geometry.PointCloud()
    # pc.points = o3d.utility.Vector3dVector(pcd)
    # o3d.visualization.draw_geometries([pc])

    # Remove points below table
    z_final_ind = pcd[:, 2] >-0.03
    pcd= pcd[z_final_ind]

    print("Deleted points below table")
    # pc = o3d.geometry.PointCloud()
    # pc.points = o3d.utility.Vector3dVector(pcd)
    # o3d.visualization.draw_geometries([pc])

    return pcd


def segment_mug_rack(pcd):
    # Use open3d to cluster using DBSCAN
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pcd)
    labels = np.array(pc.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pc.colors = o3d.utility.Vector3dVector(colors[:, :3])

    o3d.visualization.draw_geometries([pc])

    # Extract only the 2 largest clusters
    # Count the number of points in each cluster using numpy indexing
    cluster_sizes = np.zeros(max_label + 1)
    for i in range(max_label + 1):
        cluster_sizes[i] = np.sum(labels == i)
        
    # Get the indices of the two largest clusters
    largest_cluster_ind = np.argsort(cluster_sizes)[-1]
    second_largest_cluster_ind = np.argsort(cluster_sizes)[-2]
    
    # Get the points in the two largest clusters. We will decide which is which after
    largest_pcd = pcd[labels == largest_cluster_ind]
    second_largest_pcd = pcd[labels == second_largest_cluster_ind]

    # The cluster with the largest z value is the rack
    if np.max(largest_pcd[:, 2]) > np.max(second_largest_pcd[:, 2]):
        mug_pcd = second_largest_pcd
        rack_pcd = largest_pcd
    else:
        mug_pcd = largest_pcd
        rack_pcd = second_largest_pcd

    mug_pc = o3d.geometry.PointCloud()
    mug_pc.points = o3d.utility.Vector3dVector(mug_pcd)
    mug_pc.paint_uniform_color([1, 0.706, 0])
    rack_pc = o3d.geometry.PointCloud()
    rack_pc.points = o3d.utility.Vector3dVector(rack_pcd)
    rack_pc.paint_uniform_color([0, 0.651, 0.929])

    o3d.visualization.draw_geometries([mug_pc, rack_pc])

    return mug_pcd, rack_pcd
    
def synthetic_bottom(pcd):
    BOTTOM_THRESHOLD = pcd.min(axis=0)[2] + 0.005
    n = 800
    syn_x = np.random.uniform(pcd[pcd[:, 2]<BOTTOM_THRESHOLD][:, 0].min(), pcd[pcd[:, 2]<BOTTOM_THRESHOLD][:, 0].max(), n).reshape(n,1)
    syn_y = np.random.uniform(pcd[pcd[:, 2]<BOTTOM_THRESHOLD][:, 1].min(), pcd[pcd[:, 2]<BOTTOM_THRESHOLD][:, 1].max(), n).reshape(n,1)
    syn_z = np.random.uniform(pcd.min(axis=0)[2], pcd.min(axis=0)[2] + 0.001, n).reshape(n,1)
    syn_bottom = np.hstack([syn_x, syn_y, syn_z])
    pcd = np.vstack([pcd, syn_bottom])
    return pcd


def run_trial(trialdir, listener):

    points = get_point_cloud()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    o3d.visualization.draw_geometries([pcd])

    points = clean_pcd(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    o3d.visualization.draw_geometries([pcd])

    mug_pcd, rack_pcd = segment_mug_rack(points)

    # Add false bottom to the mug.
    mug_pcd = synthetic_bottom(mug_pcd)

    mug_pc = o3d.geometry.PointCloud()
    mug_pc.points = o3d.utility.Vector3dVector(mug_pcd)
    mug_pc.paint_uniform_color([1, 0.706, 0])
    rack_pc = o3d.geometry.PointCloud()
    rack_pc.points = o3d.utility.Vector3dVector(rack_pcd)
    rack_pc.paint_uniform_color([0, 0.651, 0.929])
    
    o3d.visualization.draw_geometries([mug_pc, rack_pc])

    # Save the point clouds to a single npz.
    cloud_file = os.path.join(trialdir, "point_cloud.npz")
    print(f"Saving point clouds to {cloud_file}")
    np.savez(cloud_file, mug=mug_pcd, rack=rack_pcd)


    # Call a subprocess to run inference on the mug and rack point clouds.
    # This will save the results to a file.
    print("Running inference on mug and rack point clouds")
    # Launch the inference subprocess.
    inference_process = subprocess.Popen(
        [
            "/home/beisner/.pyenv/versions/taxpose/bin/python",
            "/home/beisner/code/rpad/taxpose/scripts/inference_from_file.py",
            trialdir,
        ]
    )
    inference_process.wait()

    # Load the results from the inference subprocess.
    print("Loading inference results")
    results_file = os.path.join(trialdir, "pred_transform.json")

    return results_file




    
def main(trial_name=None, logdir = os.path.expanduser("~/catkin_ws/src/taxpose_ros/trials")):
    rospy.init_node("placement_trial")
    listener = tf.TransformListener()
    
    if trial_name is None:
        trial_name = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    trial_dir = os.path.join(logdir, trial_name)

    # Create a directory for this trial.
    if not os.path.exists(trial_dir):
        os.makedirs(trial_dir)

        results_file = run_trial(trial_dir, listener)
    else:
        results_file = os.path.join(trial_dir, "pred_transform.json")

    scene_file = np.load(os.path.join(trial_dir, "point_cloud.npz"))
    mug_pcd = scene_file["mug"]
    rack_pcd = scene_file["rack"]
    mug_pc = o3d.geometry.PointCloud()
    mug_pc.points = o3d.utility.Vector3dVector(mug_pcd)
    mug_pc.paint_uniform_color([1, 0.706, 0])
    rack_pc = o3d.geometry.PointCloud()
    rack_pc.points = o3d.utility.Vector3dVector(rack_pcd)
    rack_pc.paint_uniform_color([0, 0.651, 0.929])
    combined_pc = o3d.geometry.PointCloud()
    combined_pc.points = o3d.utility.Vector3dVector(np.vstack([mug_pcd, rack_pcd]))
    o3d.visualization.draw_geometries([combined_pc])


    input("Move the mug to the grasping position, reenable, then press enter to continue.")

    execute_taxpose(results_file, listener, combined_pc)



if __name__ == "__main__":
    typer.run(main)
