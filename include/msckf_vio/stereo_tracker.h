#ifndef MSCKF_VIO_STEREO_TRACKER_H
#define MSCKF_VIO_STEREO_TRACKER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <msckf_vio/CameraMeasurement.h>

#include <gvio/gvio.hpp>


namespace msckf_vio {

static gvio::CameraProperty load_camera_property(const std::string &prefix,
                                                 const int cam_index,
                                                 ros::NodeHandle &nh) {
  // Load camera parameters
  // -- Resolution
  std::vector<int> resolution_temp(2);
  nh.getParam(prefix + "/resolution", resolution_temp);
  gvio::Vec2 resolution{resolution_temp[0], resolution_temp[1]};
  // -- Camera model
  std::string camera_model;
  nh.getParam(prefix + "/camera_model", camera_model);
  // -- Intrinsics
  std::vector<double> intrinsics_temp(4);
  nh.getParam(prefix + "/intrinsics", intrinsics_temp);
  gvio::Mat3 K;
  const double fx = intrinsics_temp[0];
  const double fy = intrinsics_temp[1];
  const double cx = intrinsics_temp[2];
  const double cy = intrinsics_temp[3];
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  // -- Distortion model
  std::string distortion_model;
  nh.getParam(prefix + "/distortion_model", distortion_model);
  std::cout << distortion_model << std::endl;
  // -- Distortion coefficients
  std::vector<double> D_temp(4);
  nh.getParam(prefix + "/distortion_coeffs", D_temp);
  gvio::Vec4 D;
  D << D_temp[0], D_temp[1], D_temp[2], D_temp[3];
  ROS_INFO("Loaded camera parameters!");

  return gvio::CameraProperty(cam_index, camera_model, K, distortion_model, D, resolution);
}

static gvio::GimbalModel load_gimbal_model(ros::NodeHandle &nh) {
  gvio::GimbalModel gimbal_model;

  // -- tau_s
  vector<double> tau_s_temp(6);
  nh.getParam("gimbal_cam/tau_s", tau_s_temp);
  gimbal_model.tau_s(0) = tau_s_temp[0];
  gimbal_model.tau_s(1) = tau_s_temp[1];
  gimbal_model.tau_s(2) = tau_s_temp[2];
  gimbal_model.tau_s(3) = tau_s_temp[3];
  gimbal_model.tau_s(4) = tau_s_temp[4];
  gimbal_model.tau_s(5) = tau_s_temp[5];
  // -- tau_d
  vector<double> tau_d_temp(6);
  nh.getParam("gimbal_cam/tau_d", tau_d_temp);
  gimbal_model.tau_d(0) = tau_d_temp[0];
  gimbal_model.tau_d(1) = tau_d_temp[1];
  gimbal_model.tau_d(2) = tau_d_temp[2];
  gimbal_model.tau_d(3) = tau_d_temp[3];
  gimbal_model.tau_d(4) = tau_d_temp[4];
  gimbal_model.tau_d(5) = tau_d_temp[5];
  // -- w1
  vector<double> w1_temp(3);
  nh.getParam("gimbal_cam/w1", w1_temp);
  gimbal_model.w1(0) = w1_temp[0];
  gimbal_model.w1(1) = w1_temp[1];
  gimbal_model.w1(2) = w1_temp[2];
  // -- w2
  vector<double> w2_temp(3);
  nh.getParam("gimbal_cam/w2", w2_temp);
  gimbal_model.w2(0) = w2_temp[0];
  gimbal_model.w2(1) = w2_temp[1];
  gimbal_model.w2(2) = w2_temp[2];
  // -- theta1_offset
  double theta1_offset_temp = 0.0;
  nh.getParam("gimbal_cam/theta1_offset", theta1_offset_temp);
  gimbal_model.theta1_offset = theta1_offset_temp;
  // -- theta2_offset
  double theta2_offset_temp = 0.0;
  nh.getParam("gimbal_cam/theta2_offset", theta2_offset_temp);
  gimbal_model.theta2_offset = theta2_offset_temp;

  return gimbal_model;
}

class StereoTracker {
public:
  StereoTracker(ros::NodeHandle &nh);
  StereoTracker(const StereoTracker&) = delete;
  StereoTracker operator=(const StereoTracker&) = delete;
  ~StereoTracker();

  bool initialize();

  typedef boost::shared_ptr<StereoTracker> Ptr;
  typedef boost::shared_ptr<const StereoTracker> ConstPtr;

private:
  ros::NodeHandle nh;

  // Subscribers and publishers
  message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereo_sub;
  ros::Subscriber gimbal_sub;
  ros::Publisher feature_pub;

  // Tracker
  gvio::GimbalModel gimbal_model;
  gvio::StereoKLTTracker stereo_tracker;

  // Images
  cv_bridge::CvImageConstPtr cam0_img_ptr;
  cv_bridge::CvImageConstPtr cam1_img_ptr;

  // Gimbal
  Eigen::Vector2d joint_angles;

  // Display
  bool show_matches = false;
  bool show_tracking = false;

  void stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img,
                      const sensor_msgs::ImageConstPtr& cam1_img);

  void gimbalCallback(const geometry_msgs::Vector3 &msg);
};

typedef StereoTracker::Ptr StereoTrackerPtr;
typedef StereoTracker::ConstPtr StereoTrackerConstPtr;

} // end namespace msckf_vio
#endif
