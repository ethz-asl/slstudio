#include "CameraROS.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
  cout << "[CameraROS] Starting Asynchronous ROS Spinner " << endl;
  m_spinner_ptr->start();
  cout << "[CameraROS] Started Asynchronous ROS Spinner " << endl;

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

      // If image is in BayerGB8, convert to Mono8
      if (!sensor_msgs::image_encodings::isMono(m_sw_trig_buffer.encoding)) {
        m_sw_trig_buffer =
            *cv_bridge::toCvCopy(m_sw_trig_buffer,
                                 sensor_msgs::image_encodings::MONO8)
                 ->toImageMsg();
      }

      frame.memory = &m_sw_trig_buffer.data[0];
      frame.height = m_sw_trig_buffer.height;
      frame.width = m_sw_trig_buffer.width;
      frame.sizeBytes = m_sw_trig_buffer.step;

      // cout << frame.height << endl;
      // cout << frame.width << endl;
    }

    {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      m_sw_trig_state = Esoftware_trigger_state::idle;
    }
  }

  if (triggerMode == triggerModeHardware) {
    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = std::chrono::seconds(0);

    while (duration.count() * 1000 < m_exposure_time_us * 4) {  // 30Hz
      // while (duration.count() * 1000 < m_exposure_time_us * 2) { // 60 Hz
      int status = retrieve_frame(frame);
      if (status == 1 || status == -1) {
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        auto end = std::chrono::system_clock::now();
        duration = end - start;
        // std::cout << "[CameraROS] getFrame duration: "
        //          << duration.count() * 1000 << std::endl;
      }
    }
  }

  return frame;
}

size_t CameraROS::getFrameSizeBytes() {}

size_t CameraROS::getFrameWidth() { return m_frame_width; }

size_t CameraROS::getFrameHeight() { return m_frame_height; }

void CameraROS::image_cb(const sensor_msgs::Image& image) {
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
    } else if (triggerMode == triggerModeHardware) {
      boost::mutex::scoped_lock mutex_lock(m_mutex);
      // cout << "[CameraROS] Image received with timestamp: "
      //   << image.header.stamp << endl;
      m_hw_trig_buffer.emplace_back(image);
    }
  }
}

CameraROS::~CameraROS() {
  m_spinner_ptr->stop();
  m_nh_ptr->shutdown();
  ros::shutdown();
  while (ros::ok()) {
    cout << "Waiting for ROS to complete shutdown..." << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  cout << "[CameraROS] : ROS Shutdown Complete" << endl;
}

void CameraROS::get_input(const std::string& input_name,
                          std::shared_ptr<void> input_ptr) {
  if (input_name == "expected_image_time") {
    boost::mutex::scoped_lock mutex_lock(m_mutex);
    m_expected_image_time = *std::static_pointer_cast<ros::Time>(input_ptr);
    // std::cout << "[CameraROS] Received expected image time: "
    //          << m_expected_image_time << std::endl;
  }
}

int CameraROS::retrieve_frame(CameraFrame& frame) {
  int result = 0;  // -1 : Missed frame, 0 : Frame not yet arrived, 1 : Frame
                   // acquired successfully

  if (m_hw_trig_buffer.size() > 0) {
    boost::mutex::scoped_lock lock(m_mutex);
    auto it = m_hw_trig_buffer.begin();
    while (it != m_hw_trig_buffer.end()) {
      double delta_t = (it->header.stamp - m_expected_image_time).toSec();

      // std::cout << "[CameraROS] " << m_expected_image_time << " - "
      //          << it->header.stamp << " = " << delta_t << std::endl;

      if (delta_t < -1.0 * m_image_time_tolerance_s) {
        // If current image is before the expected trigger time, we delete it
        // since it is no longer of use
        std::cout << "[CameraROS] Deleting frame, outdated" << std::endl;
        it = m_hw_trig_buffer.erase(it);
      } else if (delta_t >= -1.0 * m_image_time_tolerance_s &&
                 delta_t <= m_image_time_tolerance_s) {
        // If current image matches the trigger time, we place it in the Camera
        // frame and delete it from the vector

        // If image is in BayerGB8, convert to Mono8

        if (!sensor_msgs::image_encodings::isMono(it->encoding)) {
          *it = *cv_bridge::toCvCopy(*it, sensor_msgs::image_encodings::MONO8)
                     ->toImageMsg();
        }

        frame.memory = &it->data[0];
        frame.height = it->height;
        frame.width = it->width;
        frame.sizeBytes = it->step;

        std::cout << "[CameraROS] Found matching frame " << std::endl;
        it = m_hw_trig_buffer.erase(it);
        result = 1;
        break;
      }

      else {
        // If image is in the future, means we have most likely missed the frame
        // that matches the current trigger timing
        std::cout << "[CameraROS] Missed this frame" << std::endl;
        result = -1;
        break;
      }
    }
  }
  return result;
}
