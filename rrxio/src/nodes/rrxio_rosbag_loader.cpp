/*
 * Copyright (c) 2021, Christopher Doer
 * Copyright (c) 2014, Autonomous Systems Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "rrxio/RRxIOFilter.hpp"
#include "rrxio/RRxIONode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <random>

#define foreach BOOST_FOREACH

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_      = 15;  // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_   = 4;   // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8;   // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_      = 1;   // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_     = 0;   // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_, nLevels_, patchSize_, nCam_, nPose_>> mtFilter;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rovio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string rootdir       = ros::package::getPath("rovio");  // Leaks memory
  std::string filter_config = rootdir + "/cfg/rovio.info";

  nh_private.param("filter_config", filter_config, filter_config);

  ROS_INFO_STREAM("RRxIO started");

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID)
  {
    std::string camera_config;
    if (nh_private.getParam("camera" + std::to_string(camID) + "_config", camera_config))
    {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  mpFilter->refreshProperties();

  // Node
  rovio::RovioNode<mtFilter> rovioNode(nh, nh_private, mpFilter);
  //  rovio::RovioNode rovioNode(nh, nh_private, mpFilter);
  rovioNode.makeTest();
  double resetTrigger = 0.0;
  nh_private.param("record_odometry", rovioNode.forceOdometryPublishing_, rovioNode.forceOdometryPublishing_);
  nh_private.param("record_pose_with_covariance_stamped",
                   rovioNode.forcePoseWithCovariancePublishing_,
                   rovioNode.forcePoseWithCovariancePublishing_);
  nh_private.param("record_transform", rovioNode.forceTransformPublishing_, rovioNode.forceTransformPublishing_);
  nh_private.param("record_extrinsics", rovioNode.forceExtrinsicsPublishing_, rovioNode.forceExtrinsicsPublishing_);
  nh_private.param("record_imu_bias", rovioNode.forceImuBiasPublishing_, rovioNode.forceImuBiasPublishing_);
  nh_private.param("record_pcl", rovioNode.forcePclPublishing_, rovioNode.forcePclPublishing_);
  nh_private.param("record_markers", rovioNode.forceMarkersPublishing_, rovioNode.forceMarkersPublishing_);
  nh_private.param("record_patch", rovioNode.forcePatchPublishing_, rovioNode.forcePatchPublishing_);
  nh_private.param("reset_trigger", resetTrigger, resetTrigger);

  std::cout << "Recording";
  if (rovioNode.forceOdometryPublishing_)
    std::cout << ", odometry";
  if (rovioNode.forceTransformPublishing_)
    std::cout << ", transform";
  if (rovioNode.forceExtrinsicsPublishing_)
    std::cout << ", extrinsics";
  if (rovioNode.forceImuBiasPublishing_)
    std::cout << ", imu biases";
  if (rovioNode.forcePclPublishing_)
    std::cout << ", point cloud";
  if (rovioNode.forceMarkersPublishing_)
    std::cout << ", markers";
  if (rovioNode.forcePatchPublishing_)
    std::cout << ", patch data";
  std::cout << std::endl;

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);
  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  nh_private.param("imu_topic_name", imu_topic_name, imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  nh_private.param("cam0_topic_name", cam0_topic_name, cam0_topic_name);
  std::string cam1_topic_name = "/cam1/image_raw";
  nh_private.param("cam1_topic_name", cam1_topic_name, cam1_topic_name);
  std::string topic_vel = "";
  nh_private.param("topic_vel", topic_vel, topic_vel);

  // Radar ego veloity
  std::string topic_radar_trigger = "";
  nh_private.param("topic_radar_trigger", topic_radar_trigger, topic_radar_trigger);
  std::string topic_radar_scan = "";
  nh_private.param("topic_radar_scan", topic_radar_scan, topic_radar_scan);

  double timeshift_cam_imu = 0.0;
  nh_private.param("timeshift_cam_imu", timeshift_cam_imu, timeshift_cam_imu);

  double bag_start = 0.0;
  nh_private.param("bag_start", bag_start, bag_start);

  double bag_duration = -1.;
  nh_private.param("bag_duration", bag_duration, bag_duration);

  double sigma_v_b_x = 0.0;
  nh_private.param("sigma_v_b_x", sigma_v_b_x, sigma_v_b_x);

  double sigma_v_b_y = 0.0;
  nh_private.param("sigma_v_b_y", sigma_v_b_y, sigma_v_b_y);

  double sigma_v_b_z = 0.0;
  nh_private.param("sigma_v_b_z", sigma_v_b_z, sigma_v_b_z);

  int max_frame_ctr = 1000000;
  nh_private.param("max_frame_ctr", max_frame_ctr, max_frame_ctr);

  std::default_random_engine generator(ros::WallTime::now().nsec);
  std::normal_distribution<double> nd_v_b_x(0, sigma_v_b_x);
  std::normal_distribution<double> nd_v_b_y(0, sigma_v_b_y);
  std::normal_distribution<double> nd_v_b_z(0, sigma_v_b_z);

  std::string odometry_topic_name  = rovioNode.pubOdometry_.getTopic();
  std::string transform_topic_name = rovioNode.pubTransform_.getTopic();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for (int camID = 0; camID < mtFilter::mtState::nCam_; camID++)
  {
    extrinsics_topic_name[camID] = rovioNode.pubExtrinsics_[camID].getTopic();
  }
  std::string imu_bias_topic_name = rovioNode.pubImuBias_.getTopic();
  std::string pcl_topic_name      = rovioNode.pubPcl_.getTopic();
  std::string u_rays_topic_name   = rovioNode.pubMarkers_.getTopic();
  std::string patch_topic_name    = rovioNode.pubPatch_.getTopic();

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  topics.push_back(std::string(cam1_topic_name));
  topics.push_back(std::string(topic_vel));
  topics.push_back(std::string(topic_radar_trigger));
  topics.push_back(std::string(topic_radar_scan));
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));

  ROS_INFO_STREAM("[rovio]: Subscribing radar_trigger on: " << topic_radar_trigger);
  ROS_INFO_STREAM("[rovio]: Subscribing radar_scan on: " << topic_radar_scan);

  bool isTriggerInitialized = false;
  double lastTriggerTime    = 0.0;
  ros::Time start           = ros::TIME_MIN;
  uint frame_ctr            = 0;

  for (rosbag::View::iterator it = view.begin(); it != view.end() && ros::ok(); it++)
  {
    if (start == ros::TIME_MIN)
      start = it->getTime();

    if ((it->getTime() - start).toSec() < bag_start)
      continue;

    if (bag_duration > 0 && (it->getTime() - start).toSec() - bag_start - bag_duration > 0)
      break;

    if (it->getTopic() == imu_topic_name)
    {
      sensor_msgs::Imu::ConstPtr imuMsg = it->instantiate<sensor_msgs::Imu>();
      if (imuMsg != NULL)
        rovioNode.imuCallback(imuMsg);
    }
    else if (it->getTopic() == cam0_topic_name && frame_ctr < max_frame_ctr)
    {
      sensor_msgs::ImagePtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL)
      {
        imgMsg->header.stamp = ros::Time().fromSec(imgMsg->header.stamp.toSec() + timeshift_cam_imu);
        rovioNode.imgCallback0(imgMsg);
        ++frame_ctr;
      }
      else
      {
        sensor_msgs::CompressedImageConstPtr compImgMsg = it->instantiate<sensor_msgs::CompressedImage>();
        if (compImgMsg != NULL)
        {
          cv_bridge::CvImageConstPtr ptr;
          cv::Mat cv_img = cv_bridge::toCvCopy(compImgMsg)->image;
          cv::resize(cv_img, cv_img, cv::Size(), 0.5, 0.5);
          sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(compImgMsg->header, "mono8", cv_img).toImageMsg();
          imgMsg->header.stamp         = ros::Time().fromSec(imgMsg->header.stamp.toSec() + timeshift_cam_imu);
          imgMsg->header.frame_id      = compImgMsg->header.frame_id;
          rovioNode.imgCallback0(imgMsg);
          ++frame_ctr;
        }
      }
    }
    else if (it->getTopic() == topic_vel)
    {
      geometry_msgs::TwistStampedPtr velMsg = it->instantiate<geometry_msgs::TwistStamped>();
      if (velMsg != NULL)
      {
        const auto noise_x = nd_v_b_x(generator);
        const auto noise_y = nd_v_b_y(generator);
        const auto noise_z = nd_v_b_z(generator);
        velMsg->twist.linear.x += noise_x;
        velMsg->twist.linear.y += noise_y;
        velMsg->twist.linear.z += noise_z;

        rovioNode.velocityCallback(velMsg);
      }
    }

    else if (it->getTopic() == topic_radar_trigger)
    {
      std_msgs::HeaderConstPtr trigger_msg = it->instantiate<std_msgs::Header>();
      if (trigger_msg != NULL)
        rovioNode.radarTriggerCallback(trigger_msg);
    }
    else if (it->getTopic() == topic_radar_scan)
    {
      sensor_msgs::PointCloud2ConstPtr radar_scan_msg = it->instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan_msg != NULL)
        rovioNode.radarScanCallback(radar_scan_msg);
    }

    ros::spinOnce();
  }

  bagIn.close();

  return 0;
}
