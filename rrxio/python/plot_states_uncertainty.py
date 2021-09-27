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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf.transformations

import matplotlib.pyplot as plt


def quat_to_euler(q):
    return tf.transformations.euler_from_quaternion(q[[1, 2, 3, 0]])


class DataIdx:
    def __init__(self):
        self.t = 0
        self.p = 1
        self.sigma_p = 4
        self.eul_deg = 7
        self.sigma_eul_deg = 10
        self.v = 13
        self.sigma_v = 16

        self.bias_acc = 1
        self.sigma_bias_acc = 4
        self.bias_gyro_deg = 7
        self.sigma_bias_gyro_deg = 10


class PlotStatesUncertainty:
    def __init__(self, topic_prefix):
        self.last_timestamp_odometry = None

        self.odometry_data = []
        self.velocity_data = []
        self.bias_data = []

        self.lock = threading.Lock()

        self.sub_odom = rospy.Subscriber(topic_prefix + "odometry", Odometry, self.callback_odometry, queue_size=1000000)
        self.sub_bias = rospy.Subscriber(topic_prefix + "imu_biases", Imu, self.callback_imu, queue_size=1000000)

    def run(self):
        a = 1
        while not rospy.is_shutdown():
            with self.lock:
                if self.last_timestamp_odometry is not None:
                    time_diff = time.time() - self.last_timestamp_odometry
                else:
                    time_diff = 0.
            if time_diff > 0.25:
                break
            else:
                time.sleep(0.1)

        self.sub_odom.unregister()
        self.plot()

    def plot(self):
        odometry_data = np.vstack(self.odometry_data)
        odometry_data[:, DataIdx().t] -= odometry_data[0, DataIdx().t]

        fig_odom, (ax_p, ax_v, ax_eul) = plt.subplots(3, 3, sharex=True)
        fig_odom.suptitle('Odometry', fontsize="x-large")

        fig_odom_var, (ax_p_var, ax_v_var, ax_eul_var) = plt.subplots(3, 3, sharex=True)
        fig_odom_var.suptitle('Odometry Variances', fontsize="x-large")

        xyz = 'xyz'
        for k in range(3):
            ax_p[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().p + k], label=xyz[k])
            ax_p[k].set_title("p_" + xyz[k])
            ax_p[k].grid(True)

            ax_v[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().v + k], label=xyz[k])
            ax_v[k].set_title("v_" + xyz[k])
            ax_v[k].grid(True)

            ax_eul[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().eul_deg + k], label=xyz[k])
            ax_eul[k].set_title("eul_" + xyz[k])
            ax_eul[k].grid(True)

            ax_p_var[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().sigma_p + k], label=xyz[k])
            ax_p_var[k].set_title("sigma p_" + xyz[k])
            ax_p_var[k].grid(True)

            ax_v_var[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().sigma_v + k], label=xyz[k])
            ax_v_var[k].set_title("sigma v_" + xyz[k])
            ax_v_var[k].grid(True)

            ax_eul_var[k].plot(odometry_data[:, DataIdx().t], odometry_data[:, DataIdx().sigma_eul_deg + k], label=xyz[k])
            ax_eul_var[k].set_title("sigma eul_" + xyz[k])
            ax_eul_var[k].grid(True)

        bias_data = np.vstack(self.bias_data)
        bias_data[:, DataIdx().t] -= bias_data[0, DataIdx().t]
        fig_bias, (ax_acc, ax_gyro) = plt.subplots(2, 3, sharex=True)
        fig_bias.suptitle('Bias', fontsize="x-large")

        fig_bias_var, (ax_acc_var, ax_gyro_var) = plt.subplots(2, 3, sharex=True)
        fig_bias_var.suptitle('Bias Variances', fontsize="x-large")

        for k in range(3):
            ax_acc[k].plot(bias_data[:, DataIdx().t], bias_data[:, DataIdx().bias_acc + k], label=xyz[k])
            ax_acc[k].set_title("acc_" + xyz[k])
            ax_acc[k].grid(True)

            ax_gyro[k].plot(bias_data[:, DataIdx().t], bias_data[:, DataIdx().bias_gyro_deg + k], label=xyz[k])
            ax_gyro[k].set_title("gyro_" + xyz[k])
            ax_gyro[k].grid(True)

            ax_acc_var[k].plot(bias_data[:, DataIdx().t], bias_data[:, DataIdx().sigma_bias_acc + k], label=xyz[k])
            ax_acc_var[k].set_title("sigma acc_" + xyz[k])
            ax_acc_var[k].grid(True)

            ax_gyro_var[k].plot(bias_data[:, DataIdx().t], bias_data[:, DataIdx().sigma_bias_gyro_deg + k], label=xyz[k])
            ax_gyro_var[k].set_title("sigma gyro_" + xyz[k])
            ax_gyro_var[k].grid(True)

        while not rospy.is_shutdown():
            plt.pause(0.1)

    def callback_odometry(self, msg=Odometry()):
        with self.lock:
            self.last_timestamp_odometry = time.time()
            p = np.hstack((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
            sigma_p = np.sqrt(np.hstack((msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14])))
            q = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
            eul_deg = np.rad2deg(quat_to_euler(q))
            sigma_eul_deg = np.rad2deg(np.sqrt(np.hstack((msg.pose.covariance[21], msg.pose.covariance[28], msg.pose.covariance[35]))))
            v = np.hstack((msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            sigma_v = np.sqrt(np.hstack((msg.twist.covariance[0], msg.twist.covariance[7], msg.twist.covariance[14])))

            self.odometry_data.append(np.hstack((msg.header.stamp.to_sec(), p, sigma_p, eul_deg, sigma_eul_deg, v, sigma_v)))

    def callback_imu(self, msg=Imu()):
        with self.lock:
            bias_acc = np.hstack((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
            sigma_bias_acc = np.sqrt(
                np.hstack((msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[4], msg.linear_acceleration_covariance[8])))
            bias_gyro_deg = np.rad2deg(np.hstack((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)))
            sigma_bias_gyro = np.rad2deg(
                np.sqrt(np.hstack((msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[8]))))
            self.bias_data.append(np.hstack((msg.header.stamp.to_sec(), bias_acc, sigma_bias_acc, bias_gyro_deg, sigma_bias_gyro)))


if __name__ == "__main__":
    rospy.init_node('plot_states_uncertainty', anonymous=True)

    topic_prefix = rospy.get_param('~topic_prefix', "/rrxio/rovio/")

    plotting = PlotStatesUncertainty(topic_prefix)
    plotting.run()
