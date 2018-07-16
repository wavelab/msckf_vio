#ifndef STEREO_TRACKER_NODELET_H
#define STEREO_TRACKER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <msckf_vio/stereo_tracker.h>

namespace msckf_vio {

class StereoTrackerNodelet : public nodelet::Nodelet {
public:
  StereoTrackerNodelet() { return; }
  ~StereoTrackerNodelet() { return; }

private:
  virtual void onInit();
  StereoTrackerPtr img_processor_ptr;
};

} // namespace msckf_vio
#endif
