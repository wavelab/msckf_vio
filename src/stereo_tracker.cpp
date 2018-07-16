#include <msckf_vio/stereo_tracker.h>

namespace msckf_vio {

StereoTracker::StereoTracker(ros::NodeHandle &nh) : nh{nh}, stereo_sub(10) {}

bool StereoTracker::initialize() {
  // Subscribe to stereo cameras
  this->cam0_img_sub.subscribe(nh, "cam0_image", 10);
  this->cam1_img_sub.subscribe(nh, "cam1_image", 10);
  this->stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
  this->stereo_sub.registerCallback(&StereoTracker::stereoCallback, this);

  // Subscribe to Gimbal joint angles
  this->gimbal_sub = nh.subscribe("gimbal", 0, &StereoTracker::gimbalCallback, this);

  // Publisher for feature
  feature_pub = nh.advertise<CameraMeasurement>("features", 3);

  // Setup stereo tracker
  gvio::CameraProperty cam0 = load_camera_property("cam0", 0, nh);
  gvio::CameraProperty cam1 = load_camera_property("gimbal_cam", 1, nh);
  gvio::GimbalModel gimbal_model = load_gimbal_model(nh);
  this->stereo_tracker = gvio::StereoKLTTracker(cam0, cam1, gimbal_model, 2, 20);

  // Settings
  nh.getParam("show_matches", this->show_matches);
  nh.getParam("show_tracking", this->show_tracking);

  return true;
}

void StereoTracker::stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img,
                                   const sensor_msgs::ImageConstPtr& cam1_img) {
  // Get cam0 and cam1 image pointers
  this->cam0_img_ptr = cv_bridge::toCvShare(
    cam0_img,
    sensor_msgs::image_encodings::MONO8
  );
  this->cam1_img_ptr = cv_bridge::toCvShare(
    cam1_img,
    sensor_msgs::image_encodings::MONO8
  );

  // Update stereo tracker
  this->stereo_tracker.update(this->cam0_img_ptr->image,
                              this->cam1_img_ptr->image,
                              this->joint_angles(0),
                              this->joint_angles(1));

  // Get lost tracks
  auto lost_tracks = this->stereo_tracker.getLostTracks();

  // Features
  CameraMeasurementPtr feature_msg_ptr(new CameraMeasurement);
  feature_msg_ptr->header.stamp = this->cam0_img_ptr->header.stamp;

  int i = 0;
  for (const auto &item: this->stereo_tracker.features.buffer) {
    const gvio::FeatureTrack track = item.second;

    const cv::Point2f cam0_point = track.track0.back().kp.pt;
    const cv::Point2f cam1_point = track.track1.back().kp.pt;
    const cv::Point2f cam0_uv = this->stereo_tracker.cam0.undistortPoint(cam0_point);
    const cv::Point2f cam1_uv = this->stereo_tracker.cam1.undistortPoint(cam1_point);

    feature_msg_ptr->features.push_back(FeatureMeasurement());
    feature_msg_ptr->features[i].id = track.track_id;
    feature_msg_ptr->features[i].u0 = cam0_uv.x;
    feature_msg_ptr->features[i].v0 = cam0_uv.y;
    feature_msg_ptr->features[i].u1 = cam1_uv.x;
    feature_msg_ptr->features[i].v1 = cam1_uv.y;

    i++;
  }
  this->feature_pub.publish(feature_msg_ptr);

  this->stereo_tracker.showMatches(this->cam0_img_ptr->image,
                                   this->cam1_img_ptr->image);
  this->stereo_tracker.showTracking(this->cam0_img_ptr->image);
  cv::waitKey(1);
}

void StereoTracker::gimbalCallback(const geometry_msgs::Vector3 &msg) {
	this->joint_angles = Eigen::Vector2d{msg.x, msg.y};
}

} // namespace msckf_vio
