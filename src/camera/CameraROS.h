#ifndef CameraROS_H
#define CameraROS_H

#include "Camera.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <mutex>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <string>

using namespace std;

class CameraROS : public Camera {
 public:
  // Static methods
  static vector<CameraInfo> getCameraList();

  // Interface function
  CameraROS(unsigned int camNum, CameraTriggerMode triggerMode);

  CameraSettings getCameraSettings();
  void setCameraSettings(CameraSettings);
  void startCapture();
  void stopCapture();
  CameraFrame getFrame();
  size_t getFrameSizeBytes();
  size_t getFrameWidth();
  size_t getFrameHeight();
  ~CameraROS();
  void image_cb(const sensor_msgs::Image& image);

  virtual void get_input(const std::string& input_name,
                         std::shared_ptr<void> input_ptr) override;

 private:
  enum class Esoftware_trigger_state { idle, awaiting_image, received_image };

  std::unique_ptr<ros::NodeHandle> m_nh_ptr;
  std::unique_ptr<ros::AsyncSpinner> m_spinner_ptr;
  // ros::Subscriber m_projector_time_sub;
  // ros::Subscriber m_camera_time_sub;
  ros::Subscriber m_camera_image_sub;
  // const std::string m_projector_time_topic =
  // "/versavis/projector/image_time";
  // const std::string m_camera_time_topic = "/versavis/cam1/image_time";
  const std::string m_camera_image_topic = "/versavis/cam1/image_raw";
  const std::string m_ros_node_name = "SLStudio_ROS_camera";
  const size_t m_frame_width = 1024;  // Grasshopper mode 2sss
  const size_t m_frame_height = 768;  // Grasshopper mode  2
  const unsigned int m_exposure_time_us = 8333;
  boost::mutex m_mutex;
  Esoftware_trigger_state m_sw_trig_state = Esoftware_trigger_state::idle;
  sensor_msgs::Image m_sw_trig_buffer;
  const int m_max_retries = 50;
  ros::Time m_expected_image_time;
  double m_image_time_tolerance_s = 0.0005;
  std::vector<sensor_msgs::Image> m_hw_trig_buffer;

  int retrieve_frame(CameraFrame& frame);
};

#endif
