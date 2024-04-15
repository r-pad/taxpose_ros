import datetime
import os
import subprocess
import time
import typer
import rospy
import tf

from video_util import VideoRecorder
from send_all_data import DataTransfer
from tax_pose_mug_p1 import execute_traj_grasp
from tax_pose_mug_p2 import execute_taxpose

def run_trial(trial_dir, data_transfer_node, listener):

    # Capture data using the cameras.
    data_transfer_node(trial_dir)

    # Launch the inference subprocess.
    inference_process = subprocess.Popen(
        [
            "/home/beisner/.pyenv/versions/taxpose/bin/python",
            "/home/beisner/code/rpad/taxpose/scripts/pcd_process_inference.py",
            "--data-path",
            os.path.join(trial_dir, f"0.pkl"),
        ]
    )
    inference_process.wait()
    

    # Wait for the inference subprocess to finish.

    # Visually inspect the predictions.
    input("Do things look good? (Press enter to continue)")

    ee_x = float(input("Enter grasp x : "))
    ee_y = float(input("Enter grasp y : "))
    ee_z = float(input("Enter grasp z : "))

    # Begin recording a video.
    # Create the recorder.
    recorder = VideoRecorder(trial_dir)
    recorder.start_movie()
    

    try:
        # Execute grasp trajectory.
        ee_pos = [ee_x, ee_y, ee_z]
        execute_traj_grasp(ee_pos, listener)

        while True:
            c = input("continue... (y/n))")
            if c == "n" or c == "N":
                raise Exception("User quit")
            elif c == "y" or c == "Y":
                break

        # Execute the placement trajectory.
        res_file = os.path.join(trial_dir, f"0_pred.json")
        execute_taxpose(res_file, listener)

        time.sleep(5)

    

    # End recording the video.
    except Exception as e:
        print(e)

    recorder.stop_movie()

def main(logdir: str = "./trials"):
    # Set up the logging directory.
    os.makedirs(logdir, exist_ok=True)

    # Trial name is the current time.
    trial_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    trial_dir = os.path.join(logdir, trial_name)
    os.makedirs(trial_dir, exist_ok=True)


    rospy.init_node("taxpose_node")
    data_transfer_node = DataTransfer()    
    listener = tf.TransformListener()

    run_trial(trial_dir, data_transfer_node, listener)





    




if __name__ == "__main__":
    typer.run(main)