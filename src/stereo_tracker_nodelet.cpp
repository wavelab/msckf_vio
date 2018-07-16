#include <msckf_vio/stereo_tracker_nodelet.h>

namespace msckf_vio {

void StereoTrackerNodelet::onInit() {
  this->img_processor_ptr.reset(new StereoTracker(getPrivateNodeHandle()));
  if (!img_processor_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Image Processor...");
    return;
  }
  return;
}

PLUGINLIB_DECLARE_CLASS(
  msckf_vio,
  StereoTrackerNodelet,
  msckf_vio::StereoTrackerNodelet,
  nodelet::Nodelet
);

} // namespace msckf_vio
