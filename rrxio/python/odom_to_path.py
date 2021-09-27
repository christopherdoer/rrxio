#!/usr/bin/env python

"""
This file is part of RRxIO - Robust Radar Visual/Thermal Odometry
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement
import rospy
import threading
import time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np

import tf.transformations


def quat_to_rot_mat(q):
    return tf.transformations.quaternion_matrix(q[[1, 2, 3, 0]])[0:3, 0:3]


def quat_to_euler(q):
    return tf.transformations.euler_from_quaternion(q[[1, 2, 3, 0]])


def rot_mat_to_euler(R):
    T = np.eye(4)
    T[:3, :3] = R
    return tf.transformations.euler_from_matrix(T)


def from_q_msg(q):
    return np.array([q.w, q.x, q.y, q.z])


class OdomToPath:
    def __init__(self, topic_odom, filter_name, export_file, name):
        self.export_file = export_file
        self.name = name
        self.path_pub = rospy.Publisher(topic_odom + "/path", Path, queue_size=1)
        self.path = Path()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.lock = threading.Lock()
        self.last_time_callback = None
        self.filter_name = filter_name

        self.odom_sub = rospy.Subscriber(topic_odom, Odometry, self.callback_rovio, queue_size=10)

    def callback_rovio(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose.orientation = msg.pose.pose.orientation
        cur_pose.pose.position.z = msg.pose.pose.position.z
        cur_pose.pose.position.x = - msg.pose.pose.position.y
        cur_pose.pose.position.y = msg.pose.pose.position.x

        if len(self.path.poses) == 0 or (msg.header.stamp - self.path.poses[-1].header.stamp).to_sec() > 1 / 10:
            self.path.header = msg.header
            self.path.poses.append(cur_pose)
            self.path_pub.publish(self.path)

        with self.lock:
            self.last_time_callback = time.time()

    def evaluate(self):
        positions = np.vstack([np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) for msg in self.path.poses])

        step = 2
        trajectory_lengths = np.cumsum(np.linalg.norm(positions[step::step, :] - positions[:-step:step, :], axis=1))
        p_err_final = positions[-1, :] - positions[0, :]
        p_err_final_norm = np.linalg.norm(p_err_final)

        R_final = quat_to_rot_mat(from_q_msg(self.path.poses[-1].pose.orientation))
        R_0 = quat_to_rot_mat(from_q_msg(self.path.poses[0].pose.orientation))
        eul_err = rot_mat_to_euler(R_final.dot(R_0.transpose()))
        eul_err_deg = np.rad2deg(eul_err)
        eul_final = rot_mat_to_euler(R_final)
        eul_final_deg = np.rad2deg(eul_final)

        yaw_err_deg = np.rad2deg(quat_to_euler(from_q_msg(self.path.poses[-1].pose.orientation))[2] -
                                 quat_to_euler(from_q_msg(self.path.poses[0].pose.orientation))[2])

        self.print_("##################################################################")
        self.print_("Evaluation %s" % self.filter_name)
        self.print_("##################################################################")
        self.print_("Trajectory length: %0.2fm" % trajectory_lengths[-1])
        self.print_("Final pose: %0.2fm, %0.2fm, %0.2fm,  %0.2fdeg, %0.2fdeg, %0.2fdeg"
                    % (positions[-1, 0], positions[-1, 1], positions[-1, 2], eul_final_deg[0], eul_final_deg[1], eul_final_deg[2]))
        self.print_("Position error 3D: %0.2fm, %0.2fm, %0.2fm --> %0.2fm --> %0.2f percent"
                    % (p_err_final[0], p_err_final[1], p_err_final[2], p_err_final_norm, p_err_final_norm / trajectory_lengths[-1] * 100))
        self.print_("Attitude error 3D: %0.2fdeg, %0.2fdeg, %0.2fdeg" % (eul_final_deg[0], eul_final_deg[1], eul_final_deg[2]))

        if len(export_file) > 0:
            self.print_("Exporting to %s" % (export_file + ".txt"))
            file = open(export_file + ".txt", "a")
            file.write("%s, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n" % (
                self.name, trajectory_lengths[-1], p_err_final[0], p_err_final[1], p_err_final[2], p_err_final_norm,
                p_err_final_norm / trajectory_lengths[-1] * 100,
                eul_final_deg[0], eul_final_deg[1], eul_final_deg[2], yaw_err_deg))
            file.close()

    def print_(self, s):
        rospy.loginfo("[odom_to_path_%s] : %s" % (self.filter_name, s))


if __name__ == "__main__":
    rospy.init_node('odom_to_path')

    topic_odom = rospy.get_param("~topic_odom", "")
    filter_name = rospy.get_param("~filter_name", "ROVIO")
    export_file = rospy.get_param("~export_file", "")
    name = rospy.get_param("~name", "")

    if len(topic_odom) > 0 and len(filter_name) > 0:
        odom_to_path = OdomToPath(topic_odom, filter_name, export_file, name)

        while not rospy.is_shutdown():
            with odom_to_path.lock:
                if odom_to_path.last_time_callback is not None and time.time() - odom_to_path.last_time_callback > 2.0:
                    break
            rospy.sleep(0.5)

        odom_to_path.evaluate()

    else:
        rospy.logerr("Parameters not setup correctly: topic_odom=%s, filter_name=%s" % (topic_odom, filter_name))
