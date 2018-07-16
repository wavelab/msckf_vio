#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <msckf_vio/image_processor.h>


TEST(ImageProcessor, test) {
  ros::NodeHandle nh;

  msckf_vio::ImageProcessor img_processor();
}
