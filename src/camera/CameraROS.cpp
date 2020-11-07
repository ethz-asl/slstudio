#include "CameraROS.h"
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

vector<CameraInfo> CameraROS::getCameraList() {
  // Initialise vector to be returned
  vector<CameraInfo> ret = {};

  CameraInfo cam_info;
  cam_info.vendor = "ROS";
  cam_info.model = "Grasshopper3 over ROS Driver";
  cam_info.busID = 69;  // Nice
  ret.push_back(cam_info);

  return ret;
}

CameraROS::CameraROS(unsigned int camNum, CameraTriggerMode triggerMode)
    : Camera(triggerMode) {
  // Init ros node
  cout << "[CameraROS] Initialising ROS Node " << endl;
  ros::M_string empty_args = {};
  ros::init(empty_args, m_ros_node_name);
  m_nh_ptr = std::make_unique<ros::NodeHandle>();

  // Set up subscriber
  cout << "[CameraROS] Setting up ROS subscriber " << endl;
  m_camera_image_sub =
      m_nh_ptr->subscribe(m_camera_image_topic, 5, &CameraROS::image_cb, this);

  // We use an Asyncrhonous Spinner since we dont have access to the main loop
  // Spinning is required or else subscriber will never receive messages
  cout << "[CameraROS] Setting up Asynchronous ROS Spinner " << endl;
  m_spinner_ptr = std::make_unique<ros::AsyncSpinner>(1);
  m_spinner_ptr->start();

  return;
}

CameraSettings CameraROS::getCameraSettings() {
  // Get settings:
  CameraSettings settings;
  return settings;
}

void CameraROS::setCameraSettings(CameraSettings settings) {}

void CameraROS::startCapture() { capturing = true; }

void CameraROS::stopCapture() { capturing = false; }

CameraFrame CameraROS::getFrame() {
  CameraFrame frame;

  if (triggerMode == triggerModeSoftware) {
    std::this_thread::sleep_for(std::chrono::microseconds(m_exposure_time_us));

    {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      m_sw_trig_state = Esoftware_trigger_state::awaiting_image;
    }

    bool success = false;

    for (int i = 0; i < m_max_retries; i++) {
      // cout << "[CameraROS] Try no:" << i << endl;

      std::this_thread::sleep_for(
          std::chrono::microseconds(m_exposure_time_us / 2));

      {
        boost::mutex::scoped_lock mutex_lock(m_mutex);
        if (m_sw_trig_state == Esoftware_trigger_state::received_image) {
          success = true;
          break;
        }
      }
    }

    if (success) {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      frame.memory = &m_sw_trig_buffer.data[0];
      frame.height = m_sw_trig_buffer.height;
      frame.width = m_sw_trig_buffer.width;
      frame.sizeBytes = m_sw_trig_buffer.step;

      cout << frame.height << endl;
      cout << frame.width << endl;
    }

    {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      m_sw_trig_state = Esoftware_trigger_state::idle;
    }
  }

  return frame;
}

size_t CameraROS::getFrameSizeBytes() {}

size_t CameraROS::getFrameWidth() { return m_frame_width; }

size_t CameraROS::getFrameHeight() { return m_frame_height; }

void CameraROS::image_cb(const sensor_msgs::Image &image) {
  /**
  cout << "[CameraROS] Image recevied" << endl;
  cout << "Capturing: " << capturing << endl;
  cout << "Trigger mode: "
       << ((triggerMode == triggerModeSoftware)
               ? "Software"
               : (triggerMode == triggerModeHardware) ? "Hardware" : "Unknown")
       << endl;
  cout << "Software trigger state: "
       << ((m_sw_trig_state == Esoftware_trigger_state::awaiting_image)
               ? "Awaiting image"
               : (m_sw_trig_state == Esoftware_trigger_state::idle)
                     ? "Idle"
                     : (m_sw_trig_state ==
                        Esoftware_trigger_state::received_image)
                           ? "Received image"
                           : "Unknown")
       << endl;
  **/

  if (capturing) {
    if (triggerMode == triggerModeSoftware &&
        m_sw_trig_state == Esoftware_trigger_state::awaiting_image) {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      // cout << "[CameraROS] Retrived image if dims " << image.width << "x"
      //     << image.height << endl;
      m_sw_trig_buffer = image;
      m_sw_trig_state = Esoftware_trigger_state::received_image;
    }
  }
}

CameraROS::~CameraROS() {
  m_spinner_ptr->stop();
  ros::shutdown();
}
