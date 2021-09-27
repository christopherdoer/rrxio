#!/usr/bin/env python

"""
This file is part of RRxIO - Robust Radar Visual/Thermal Odometry
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement

import threading
import numpy as np
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations

VICON = "vicon"
PSEUDO_GT = "pseudo_gt"
FINAL_POSE = "final_pose"


class GroundTruth:
    def __init__(self, csv_file_path, dataset_type):

        if dataset_type == VICON:
            gt_raw = np.genfromtxt(csv_file_path, delimiter=',', skip_header=1)
            self.t = gt_raw[:, 0] * 1.0e-9
            self.p = gt_raw[:, 1:4] - gt_raw[0, 1:4]
            self.q = gt_raw[:, 4:8]
            self.v = gt_raw[:, 8:11]
            self.bw = gt_raw[:, 11:14]
            self.ba = gt_raw[:, 14:17]

        elif dataset_type == PSEUDO_GT:
            gt_raw = np.genfromtxt(csv_file_path, delimiter=' ', skip_header=1)
            self.t = gt_raw[:, 0]
            self.p = gt_raw[:, 1:4]
            q_xyzw = gt_raw[:, 4:8]
            self.q = np.hstack((q_xyzw[:, 3].reshape(-1, 1), q_xyzw[:, :3]))
            self.v = None

        self.step = 5
        self.trajectory_lengths = np.cumsum(np.linalg.norm(self.p[self.step::self.step, :] - self.p[:-self.step:self.step, :], axis=1))
        print("##### Loaded trajectory with length %0.2fm and duration %0.2fs" % (self.trajectory_lengths[-1], self.t[-1] - self.t[0]))


def quat_to_euler(q):
    return tf.transformations.euler_from_quaternion(q[[1, 2, 3, 0]])


class EvaluateGroundTruth:
    def __init__(self, rosbag_name, rosbag_dir, ground_truth_csv, pose_topic, odom_topic, export_directory, ground_truth_type):

        self.rosbag_name = rosbag_name
        self.rosbag_dir = rosbag_dir

        if len(export_directory) > 0:
            self.export_directory = export_directory
        else:
            self.export_directory = ""

        if ground_truth_type != FINAL_POSE:
            gt_csv_path = rosbag_dir + ground_truth_csv
            self.ground_truth = GroundTruth(gt_csv_path, ground_truth_type)

        self.ground_truth_type = ground_truth_type
        self.last_timestamp_odometry = None
        self.timestamp_first_odometry = None
        self.pose_data = []
        self.start_time = None
        self.last_pub_walltime = time.time()

        self.lock = threading.Lock()

        self.sub_pose = rospy.Subscriber(pose_topic, PoseStamped, self.callback_pose, queue_size=1000000)
        self.sub_odom = rospy.Subscriber(odom_topic, Odometry, self.callback_odometry, queue_size=1000000)

    def callback_pose(self, msg=PoseStamped()):
        if self.start_time is None:
            self.start_time = time.time()

        with self.lock:
            self.last_timestamp_odometry = time.time()

        p = np.hstack((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

        self.pose_data.append(np.hstack((msg.header.stamp.to_sec(), p, q[1:], q[0])))

    def callback_odometry(self, msg=Odometry()):
        if self.start_time is None:
            self.start_time = time.time()

        with self.lock:
            self.last_timestamp_odometry = time.time()

        p = np.hstack((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        q = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

        self.pose_data.append(np.hstack((msg.header.stamp.to_sec(), p, q[1:], q[0])))

    def print_(self, s):
        rospy.loginfo("[evaluate_ground_truth] : " + s)

    def export(self):

        time_taken = self.last_timestamp_odometry - self.start_time
        time_data = self.pose_data[-1][0] - self.pose_data[0][0]
        runtime = "%0.2f &  %0.2f & %0.2f" % (time_taken, time_data, time_data / time_taken)
        self.print_(runtime)

        if len(self.export_directory) > 0:
            self.print_("Exporting to %s" % self.export_directory)
            dataset = self.export_directory[self.export_directory.rfind("/") + 1:]
            algo = self.export_directory[self.export_directory[:self.export_directory.rfind("/")].rfind("/") + 1:self.export_directory.rfind("/")]
            export_path_eval = self.export_directory[:self.export_directory[:self.export_directory.rfind("/")].rfind("/")]

            timing_file = open(export_path_eval + "/" + "eval.txt", "a")
            timing_file.write(dataset + "_" + algo + " & " + runtime + "\n")
            timing_file.close()

            data_vio = np.vstack(self.pose_data)

            if not os.path.isdir(self.export_directory):
                os.makedirs(self.export_directory)
            if self.ground_truth_type != FINAL_POSE:
                csv_gt = self.export_directory + "/" + "stamped_groundtruth.txt"
                csv_vio = self.export_directory + "/" + "stamped_traj_estimate.txt"
                data_gt = np.hstack(
                    (self.ground_truth.t.reshape(-1, 1), self.ground_truth.p, self.ground_truth.q[:, 1:], self.ground_truth.q[:, 0].reshape(-1, 1)))
                np.savetxt(csv_gt, data_gt, delimiter=' ')
                np.savetxt(csv_vio, data_vio, delimiter=' ')
            elif self.ground_truth_type == FINAL_POSE:
                positions = data_vio[:, 1:4]

                step = 2
                trajectory_lengths = np.cumsum(np.linalg.norm(positions[step::step, :] - positions[:-step:step, :], axis=1))
                p_err_final = positions[-1, :] - positions[0, :]
                p_err_final_norm = np.linalg.norm(p_err_final)
                p_err_final_norm_rel = p_err_final_norm / trajectory_lengths[-1] * 100
                eul_0 = np.hstack(quat_to_euler(data_vio[0, 4:8]))
                eul_N = np.hstack(quat_to_euler(data_vio[-1, 4:8]))
                eul_final_deg = np.rad2deg(np.mod(eul_0 - eul_N + np.pi, 2 * np.pi) - np.pi)

                final_err_file = open(export_path_eval + "/" + "final_err.txt", "a")
                final_err_file.write("%s, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n" % (
                    dataset + "_" + algo, trajectory_lengths[-1], p_err_final[0], p_err_final[1], p_err_final[2], p_err_final_norm,
                    p_err_final_norm_rel, eul_final_deg[0], eul_final_deg[1], eul_final_deg[2]))
                final_err_file.close()


if __name__ == "__main__":
    rospy.init_node('evaluate_ground_truth', anonymous=True)

    rosbag_name = rospy.get_param('~rosbag_name', "")
    rosbag_dir = rospy.get_param('~rosbag_dir', "")
    ground_truth_csv = rospy.get_param('~ground_truth_csv', "")
    pose_topic = rospy.get_param('~pose_topic', "")
    odom_topic = rospy.get_param('~odom_topic', "")
    export_directory = rospy.get_param('~export_directory', "")
    ground_truth_type = rospy.get_param('~ground_truth_type', VICON)
    evaluator = EvaluateGroundTruth(rosbag_name, rosbag_dir, ground_truth_csv, pose_topic, odom_topic, export_directory, ground_truth_type)
    while not rospy.core.is_shutdown():
        if evaluator.last_timestamp_odometry is not None and time.time() - evaluator.last_timestamp_odometry > 1:
            break
        time.sleep(0.1)

    evaluator.export()
