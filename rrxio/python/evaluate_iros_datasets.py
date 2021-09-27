#!/usr/bin/env python

"""
This file is part of RRxIO - Robust Radar Visual/Thermal Odometry
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement
import sys
import os
import rospkg
import time
from threading import Thread

import evaluate_ground_truth


def run(N_feature):
    result_base_directory = rosbag_dir + "/results/evaluation/" + str(N_feature) + "/"

    N_rovio_features = " N:=" + str(N_feature)

    base_configs = [
        {"launch_file": "rrxio_evaluate_rosbag.launch", "name": str(N_feature) + "_rrxio_visual", "config": "rrxio_iros_datasets_visual.info",
         "use_vel": True, "camera_config": camera_config_visual, "params": params_visual + " timeshift_cam_imu:=0.002"},
        {"launch_file": "rrxio_evaluate_rosbag.launch", "name": str(N_feature) + "_rrxio_thermal", "config": "rrxio_iros_datasets_thermal.info",
         "use_vel": True, "camera_config": camera_config_thermal, "params": params_thermal + " timeshift_cam_imu:=-0.004"}
    ]

    configs = [
        {'name': "base", 'changes': {}},
    ]

    datasets = [
        {"name": "mocap_easy", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},  # 61m 94s
        {"name": "mocap_medium", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},  # 65m 87s
        {"name": "mocap_difficult", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},  # 93m 87s
        {"name": "mocap_dark", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},  # 111m, 135s
        {"name": "mocap_dark_fast", "start_time": 0, "ground_truth_type": evaluate_ground_truth.VICON},  # 75m, 86s
        
        {"name": "gym", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},  # 74m, 84s
        {"name": "indoor_floor", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},  # 240m, 206s
        {"name": "outdoor_campus", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},  # 102m, 100s
        {"name": "outdoor_street", "start_time": 0, "ground_truth_type": evaluate_ground_truth.PSEUDO_GT},  # 202m, 186s
    ]

    platform = "nuc"
    default_params = "shutdown_when_done:=True enable_rviz:=False"
    export_directory = result_base_directory + platform + "/"

    rospack = rospkg.RosPack()

    evaluation_dir = result_base_directory + "evaluation_full_align/"
    eval_config = "align_type: posyaw\n" \
                  "align_num_frames: -1"

    if suppress_console_output:
        tail = " >/dev/null 2>&1"
    else:
        tail = ""

    ctr = 0
    runs = len(base_configs) * len(configs) * len(datasets)
    start_time = time.time()
    for base_config in base_configs:
        for dataset in datasets:
            euroc_name = dataset["name"]
            for config in configs:
                ctr += 1
                print("############################################################")
                print(N_feature + ": Progress %d / %d remaining time %0.2fmin" % (ctr, runs, (runs - ctr) * (time.time() - start_time) / ctr / 60.0 * 1.5))
                print("############################################################")

                config_name = base_config['name'] + "_" + config["name"]
                export_directory_run = export_directory + config_name + "/" + platform + "_" + config_name + "_" + euroc_name

                # copy config
                if not os.path.isdir(export_directory_run):
                    os.makedirs(export_directory_run)
                config_file = rospack.get_path('rrxio') + "/launch/configs/" + base_config["config"]

                os.system("cp " + config_file + " " + export_directory_run + "/" + config_name + ".info")

                config_file = export_directory_run + "/" + config_name + ".info"
                f = open(config_file, "r")
                config_str = "".join(f.readlines())
                f.close()
                for old, new in config['changes'].items():
                    config_str = config_str.replace(old, new)

                f = open(config_file, "w")
                f.write(config_str)
                f.close()

                params = ""
                if "params" in config.keys():
                    params = config["params"]

                if base_config["use_vel"]:
                    params += params_radar
                else:
                    params += "topic_radar_trigger:=no_radar_trigger topic_radar_scan:=no_radar"

                for k in range(N_trials):
                    cmd = "roslaunch rrxio " + base_config["launch_file"] + \
                          " bag_start:=" + str(dataset["start_time"]) + \
                          " " + default_params + \
                          " rosbag:=" + euroc_name + \
                          " ground_truth_csv:=" + euroc_name + "_gt.csv" + \
                          " ground_truth_type:=" + dataset["ground_truth_type"] + \
                          " export_directory:=" + export_directory_run + \
                          " config:=" + config_file + \
                          " rosbag_dir:=" + rosbag_dir + \
                          " " + params + \
                          N_rovio_features + \
                          " " + base_config["params"] + \
                          " id:=" + N_feature
                    print(cmd)
                    os.system(cmd + tail)
                    if N_trials > 1:
                        os.system(
                            "mv " + export_directory_run + "/stamped_traj_estimate.txt" + " " + export_directory_run + "/stamped_traj_estimate" + str(
                                k) + ".txt")

                config_file_eval = open(export_directory + config_name + "/" + platform + "_" + config_name + "_" + euroc_name + "/eval_cfg.yaml", "w")
                config_file_eval.write(eval_config)
                config_file_eval.close()

    print(N_feature + ": Starting evaluation...")

    if not os.path.isdir(evaluation_dir):
        os.mkdir(evaluation_dir)

    ws = "  "
    s = "Datasets:\n"
    for euroc_dataset in datasets:
        if euroc_dataset["ground_truth_type"] != evaluate_ground_truth.FINAL_POSE:
            name = euroc_dataset['name']
            s += ws + name + ":\n"
            s += ws + ws + "label: " + name.replace("_", "") + "\n"

    ws = "  "
    s += "Algorithms:\n"
    algos = []
    for base_config in base_configs:
        for config in configs:
            algos.append(base_config['name'] + "_" + config["name"])

    for algo in algos:
        s += ws + algo + ":\n"
        s += ws + ws + "fn: traj_est\n"
        s += ws + ws + "label: " + algo.replace("_", "") + "\n"

    s += "RelDistances: [12,24,36,48,60]"
    eval_config = open(result_base_directory + "/evaluation_config.yaml", "w")
    eval_config.write(s)
    eval_config.close()

    cmd = "rosrun rpg_trajectory_evaluation analyze_trajectories.py " + result_base_directory + "/evaluation_config.yaml" \
                                                                                                " --output_dir=" + evaluation_dir + \
          " --results_dir=" + result_base_directory + \
          " --platform nuc --odometry_error_per_dataset --overall_odometry_error --plot_trajectories --rmse_table --rmse_boxplot --png --no_sort_names"

    if N_trials > 1:
        cmd += " --mul_trials " + str(N_trials)
    print(cmd)
    os.system(cmd + tail)


def evaluate():
    threads = []
    start = time.time()
    for N_feature in N_features:
        threads.append(Thread(target=run, args=(N_feature,)))
        threads[-1].start()
    time.sleep(1)
    while True:
        for i in range(len(threads)):
            if not threads[i].is_alive():
                print("###### Thread " + N_features[i] + " is done  :)")
                threads[i].join(1)
                if len(threads) == 1:
                    threads = []
                else:
                    del threads[i]
                break
            else:
                time.sleep(1)
        if len(threads) == 0:
            break
    print("Done in %0.2f minutes" % ((time.time() - start) / 60))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun eveluate_iros_datasets.py <path-to-radar_thermal_visual_inertial_datasets_iros_2021>")
    else:
        rosbag_dir = sys.argv[1]
        suppress_console_output = True
        N_trials = 1
        camera_config_visual = "rovio_visual"
        camera_config_thermal = "rovio_thermal"

        N_features = ["25", "15", "10"]
        params_radar = "topic_radar_trigger:=sensor_platform/radar/trigger topic_radar_scan:=/sensor_platform/radar/scan"
        params_visual = "camera_config:=" + rosbag_dir + "rovio_visual.yaml topic_cam:=/sensor_platform/camera_visual/img"
        params_thermal = "camera_config:=" + rosbag_dir + "rovio_thermal.yaml topic_cam:=/sensor_platform/camera_thermal/img"

        evaluate()
