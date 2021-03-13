#include "Camera.h"

#ifdef WITH_CAMERAIIDC
#include "CameraIIDC.h"
#endif
#ifdef WITH_CAMERAXIMEA
#include "CameraXIMEA.h"
#endif
#ifdef WITH_CAMERAIDSIMAGING
#include "CameraIDSImaging.h"
#endif
#ifdef WITH_CAMERAPOINTGREY
#include "CameraPointGrey.h"
#endif
#ifdef WITH_CAMERASPINNAKER
#include "CameraSpinnaker.h"
#endif
#ifdef WITH_CAMERAROS
#include "CameraROS.h"
#endif

// Global camera enumerator
std::vector<std::vector<CameraInfo> > Camera::GetInterfaceCameraList() {
  std::vector<std::vector<CameraInfo> > ret;

#if defined(WITH_CAMERAIIDC) && !defined(WITH_CAMERAROS)
  std::vector<CameraInfo> iidccameras = CameraIIDC::getCameraList();
  ret.push_back(iidccameras);
#endif
#ifdef WITH_CAMERAXIMEA
  std::vector<CameraInfo> ximeacameras = CameraXIMEA::getCameraList();
  ret.push_back(ximeacameras);
#endif
#ifdef WITH_CAMERAIDSIMAGING
  std::vector<CameraInfo> idscameras = CameraIDSImaging::getCameraList();
  ret.push_back(idscameras);
#endif
#if defined(WITH_CAMERAPOINTGREY) && !defined(WITH_CAMERAROS)
  std::vector<CameraInfo> ptgreycameras = CameraPointGrey::getCameraList();
  ret.push_back(ptgreycameras);
#endif
#if defined(WITH_CAMERASPINNAKER) && !defined(WITH_CAMERAROS)
  std::vector<CameraInfo> spinnaker_cameras = CameraSpinnaker::getCameraList();
  ret.push_back(spinnaker_cameras);
#endif
#ifdef WITH_CAMERAROS
  std::vector<CameraInfo> ros_cameras = CameraROS::getCameraList();
  ret.push_back(ros_cameras);
#endif
  return ret;
}

// Camera factory
Camera* Camera::NewCamera(unsigned int interfaceNum, unsigned int camNum,
                          CameraTriggerMode triggerMode) {
  interfaceNum += 1;

#if defined(WITH_CAMERAIIDC) && !defined(WITH_CAMERAROS)
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraIIDC(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAXIMEA
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraXIMEA(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAIDSIMAGING
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraIDSImaging(camNum, triggerMode);
#endif
#if defined(WITH_CAMERAPOINTGREY) && !defined(WITH_CAMERAROS)
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraPointGrey(camNum, triggerMode);
#endif
#if defined(WITH_CAMERASPINNAKER) && !defined(WITH_CAMERAROS)
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraSpinnaker(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAROS
  interfaceNum -= 1;
  if (interfaceNum == 0) return new CameraROS(camNum, triggerMode);
#endif

  return (Camera*)NULL;
}
