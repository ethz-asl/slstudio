#ifndef CameraSpinnaker_H
#define CameraSpinnaker_H

#include "Camera.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

using namespace std;

enum class Ecamera_type { blackfly, grasshopper, size };

class CameraSpinnaker : public Camera {
 public:
  // Static methods
  static vector<CameraInfo> getCameraList();
  static vector<CameraInfo> getCameraListFromSingleInterface(
      Spinnaker::InterfacePtr interface_ptr);
  static int PrintDeviceInfo(Spinnaker::CameraPtr pCam);
  // Interface function
  CameraSpinnaker(unsigned int camNum, CameraTriggerMode triggerMode);
  CameraSettings getCameraSettings();
  void setCameraSettings(CameraSettings);
  void startCapture();
  void stopCapture();
  CameraFrame getFrame();
  size_t getFrameSizeBytes();
  size_t getFrameWidth();
  size_t getFrameHeight();
  std::string get_device_name();
  ~CameraSpinnaker();

 private:
  Spinnaker::CameraPtr m_cam_ptr = nullptr;
  Spinnaker::SystemPtr m_sys_ptr = nullptr;
  Spinnaker::CameraPtr retrieveCameraPtrWithCamNum(unsigned int camNum);
  float m_exposure_time_micro_s = 0.0;
  Ecamera_type m_camera_type = Ecamera_type::size;
};

#endif
