#include "CameraSpinnaker.h"
#include <cstring>

void PrintError(FlyCapture2::Error error) { error.PrintErrorTrace(); }

vector<CameraInfo> CameraSpinnaker::getCameraList() {
  vector<CameraInfo> ret = {};

  Spinnaker::SystemPtr system_ptr = Spinnaker::System::GetInstance();

  // Retrieve list of interfaces from the system
  // Note: Interface lists must be cleared manually. This must be done prior to
  // releasing the system and while the interface list is still in scope.
  Spinnaker::InterfaceList interfaceList = system_ptr->GetInterfaces();
  unsigned int numInterfaces = interfaceList.GetSize();
  cout << "Number of interfaces detected: " << numInterfaces << endl << endl;

  // We search each interface for cameras
  for (unsigned int i = 0; i < numInterfaces; i++) {
    // Select interface
    auto interfacePtr = interfaceList.GetByIndex(i);
    // Get camera info for interface
    auto camerainfo_in_interface =
        getCameraListFromSingleInterface(interfacePtr);
    // Append vector of camerainfo to ret
    ret.insert(ret.end(), camerainfo_in_interface.begin(),
               camerainfo_in_interface.end());
  }

  return ret;
}

CameraSpinnaker::CameraSpinnaker(unsigned int camNum,
                                 CameraTriggerMode triggerMode)
    : Camera(triggerMode) {
  FlyCapture2::Error error;

  // Connect to camera
  FlyCapture2::BusManager busManager;
  FlyCapture2::PGRGuid camGuid;
  busManager.GetCameraFromIndex(camNum, &camGuid);
  error = cam.Connect(&camGuid);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  //    // Configure video mode and frame rate (legacy modes)
  //    FlyCapture2::VideoMode videoMode = FlyCapture2::VIDEOMODE_640x480Y8;
  //    FlyCapture2::FrameRate frameRate = FlyCapture2::FRAMERATE_30;
  //    cam.SetVideoModeAndFrameRate(videoMode, frameRate);

  // Configure Format7 mode (2x2 binning)
  FlyCapture2::Format7ImageSettings format7Settings;
  format7Settings.mode = FlyCapture2::MODE_4;
  format7Settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
  format7Settings.width = 3376 / 2;
  format7Settings.height = 2704 / 2;
  format7Settings.offsetX = 0;
  format7Settings.offsetY = 0;

  // Validate and set mode
  FlyCapture2::Format7PacketInfo packetInfo;
  bool valid;
  error = cam.ValidateFormat7Settings(&format7Settings, &valid, &packetInfo);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);
  // packetsize configures maximum frame rate
  error = cam.SetFormat7Configuration(&format7Settings,
                                      packetInfo.recommendedBytesPerPacket);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  // Turn off gamma
  FlyCapture2::Property property;
  property.type = FlyCapture2::AUTO_EXPOSURE;
  property.onOff = false;
  error = cam.SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  property.type = FlyCapture2::GAMMA;
  property.onOff = true;
  property.absControl = true;
  property.absValue = 1.0;
  error = cam.SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  // Get the camera information
  FlyCapture2::CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  std::cout << camInfo.vendorName << "  " << camInfo.modelName << "  "
            << camInfo.serialNumber << std::endl;

  // Set reasonable default settings
  CameraSettings settings;
  // settings.shutter = 8.33;
  settings.shutter = 33.33;
  settings.gain = 0.0;
  this->setCameraSettings(settings);

  return;
}

CameraSettings CameraSpinnaker::getCameraSettings() {
  FlyCapture2::Property property;

  // Get settings:
  CameraSettings settings;

  property.type = FlyCapture2::SHUTTER;
  cam.GetProperty(&property);
  settings.shutter = property.absValue;

  property.type = FlyCapture2::GAIN;
  cam.GetProperty(&property);
  settings.gain = property.absValue;

  return settings;
}

void CameraSpinnaker::setCameraSettings(CameraSettings settings) {
  FlyCapture2::Property property;
  property.onOff = true;
  property.absControl = true;

  property.type = FlyCapture2::SHUTTER;
  property.absValue = settings.shutter;
  cam.SetProperty(&property);

  property.type = FlyCapture2::GAIN;
  property.absValue = settings.gain;
  cam.SetProperty(&property);
}

void CameraSpinnaker::startCapture() {
  FlyCapture2::Error error;

  CameraSettings settings = this->getCameraSettings();
  std::cout << "\tShutter: " << settings.shutter << "ms" << std::endl;
  std::cout << "\tGain: " << settings.gain << "dB" << std::endl;

  if (triggerMode == triggerModeHardware) {
    // Configure for hardware trigger
    FlyCapture2::TriggerMode triggerMode;
    triggerMode.onOff = true;
    triggerMode.polarity = 0;
    triggerMode.source = 0;
    triggerMode.mode = 14;
    error = cam.SetTriggerMode(&triggerMode);
    if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

    error = cam.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  } else if (triggerMode == triggerModeSoftware) {
    // Configure software trigger
    FlyCapture2::TriggerMode triggerMode;
    triggerMode.onOff = true;
    triggerMode.polarity = 0;
    triggerMode.source = 7;  // software
    triggerMode.mode = 0;
    error = cam.SetTriggerMode(&triggerMode);
    if (error != FlyCapture2::PGRERROR_OK) PrintError(error);
  }

  // Set the trigger timeout to 1000 ms
  FlyCapture2::FC2Config config;
  config.grabTimeout = 1000;
  error = cam.SetConfiguration(&config);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  error = cam.StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  capturing = true;
}

void CameraSpinnaker::stopCapture() {
  FlyCapture2::Error error = cam.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  capturing = false;
}

CameraFrame CameraSpinnaker::getFrame() {
  FlyCapture2::Error error;

  if (triggerMode == triggerModeSoftware) {
    // Fire software trigger
    // broadcasting not supported on some platforms
    cam.FireSoftwareTrigger(false);
  }

  // Retrieve the image
  FlyCapture2::Image rawImage;
  error = cam.RetrieveBuffer(&rawImage);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  CameraFrame frame;

  frame.timeStamp = rawImage.GetTimeStamp().cycleCount;
  frame.height = rawImage.GetRows();
  frame.width = rawImage.GetCols();
  frame.memory = rawImage.GetData();

  return frame;
}

size_t CameraSpinnaker::getFrameSizeBytes() {
  FlyCapture2::Format7ImageSettings format7Settings;
  unsigned int dummy1;
  float dummy2;
  FlyCapture2::Error error =
      cam.GetFormat7Configuration(&format7Settings, &dummy1, &dummy2);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  return format7Settings.width * format7Settings.height;
}

size_t CameraSpinnaker::getFrameWidth() {
  FlyCapture2::Format7ImageSettings format7Settings;
  unsigned int dummy1;
  float dummy2;
  FlyCapture2::Error error =
      cam.GetFormat7Configuration(&format7Settings, &dummy1, &dummy2);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  return format7Settings.width;
}

size_t CameraSpinnaker::getFrameHeight() {
  FlyCapture2::Format7ImageSettings format7Settings;
  unsigned int dummy1;
  float dummy2;
  FlyCapture2::Error error =
      cam.GetFormat7Configuration(&format7Settings, &dummy1, &dummy2);
  if (error != FlyCapture2::PGRERROR_OK) PrintError(error);

  return format7Settings.height;
}

CameraSpinnaker::~CameraSpinnaker() {
  if (capturing && triggerMode == triggerModeHardware) {
    // Stop camera transmission
    cam.StopCapture();
  }

  // Gracefulle destruct the camera
  cam.Disconnect();
}

static vector<CameraInfo> CameraSpinnaker::getCameraListFromSingleInterface(
    Spinnaker::InterfacePtr interface_ptr) {
  vector<CameraInfo> result = {};

  try {
    // Retrieve node map to access Interface information
    Spinnaker::GenApi::INodeMap& nodeMapInterface =
        interface_ptr->GetTLNodeMap();

    // Print interface display name
    Spinnaker::GenApi::CStringPtr ptrInterfaceDisplayName =
        nodeMapInterface.GetNode("InterfaceDisplayName");
    if (IsAvailable(ptrInterfaceDisplayName) &&
        IsReadable(ptrInterfaceDisplayName)) {
      Spinnaker::GenICam::gcstring interfaceDisplayName =
          ptrInterfaceDisplayName->GetValue();
      cout << interfaceDisplayName << endl;
    } else {
      cout << "Interface display name not readable" << endl;
    }

    // Update list of cameras on the interface
    interface_ptr->UpdateCameras();

    // Retrieve list of cameras from the interface
    // Note: Camera list must be released while it is still in scope
    Spinnaker::CameraList camList = interface_ptr->GetCameras();

    // Retrieve number of cameras
    unsigned int numCameras = camList.GetSize();

    // Return if no cameras detected
    if (numCameras == 0) {
      cout << "\tNo devices detected." << endl << endl;
      return result;
    }

    // For each detected camera we extract the CameraInfo and append to result
    for (unsigned int i = 0; i < numCameras; i++) {
      CameraInfo camera_info;

      // Select camera
      Spinnaker::CameraPtr pCam = camList.GetByIndex(i);

      // Tentatively interpret the index as the busID
      camera_info.busID = (unsigned int)i;
      cout << "\tDevice " << i << " ";

      // Retrieve TL device nodemap; please see NodeMapInfo example for
      // additional comments on transport layer nodemaps
      Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

      // *** NOTES ***
      // Grabbing node information requires first retrieving the node and
      // then retrieving its information. There are two things to keep in
      // mind. First, a node is distinguished by type, which is related
      // to its value's data type. Second, nodes should be checked for
      // availability and readability/writability prior to making an
      // attempt to read from or write to the node.
      //

      // Extract Vendor name
      Spinnaker::GenApi::CStringPtr ptrDeviceVendorName =
          nodeMapTLDevice.GetNode("DeviceVendorName");
      if (IsAvailable(ptrDeviceVendorName) && IsReadable(ptrDeviceVendorName)) {
        Spinnaker::GenICam::gcstring deviceVendorName =
            ptrDeviceVendorName->ToString();

        camera_info.vendor = deviceVendorName.c_str();

        cout << deviceVendorName << " ";
      }

      // Extract Device Model
      Spinnaker::GenApi::CStringPtr ptrDeviceModelName =
          nodeMapTLDevice.GetNode("DeviceModelName");
      if (IsAvailable(ptrDeviceModelName) && IsReadable(ptrDeviceModelName)) {
        Spinnaker::GenICam::gcstring deviceModelName =
            ptrDeviceModelName->ToString();

        camera_info.model = deviceModelName.c_str();

        cout << deviceModelName << endl << endl;
      }

      // Append camera info to result
      result.push_back(camera_info);
    }

    //
    // Clear camera list before losing scope
    //
    // *** NOTES ***
    // Camera lists (and interface lists) must be cleared manually while in
    // the same scope that the system is released. However, in cases like this
    // where scope is lost, camera lists (and interface lists) will be cleared
    // automatically.
    //
    camList.Clear();
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  return result;
}
