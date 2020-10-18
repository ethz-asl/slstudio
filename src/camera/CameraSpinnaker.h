#ifndef CameraSpinnaker_H
#define CameraSpinnaker_H

#include "Camera.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

using namespace std;

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
  ~CameraSpinnaker();

 private:
  Spinnaker::CameraPtr m_cam_ptr;
  Spinnaker::SystemPtr m_sys_ptr;
  Spinnaker::CameraPtr retrieveCameraPtrWithCamNum(unsigned int camNum);
};

#endif
