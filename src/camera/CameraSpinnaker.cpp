#include "CameraSpinnaker.h"
#include <cstring>

void PrintError(FlyCapture2::Error error) { error.PrintErrorTrace(); }

vector<CameraInfo> CameraSpinnaker::getCameraList() {
  // Initialise vector to be returned
  vector<CameraInfo> ret = {};

  // Create system ptr
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

  // Clear interface list
  interfaceList.Clear();

  // Release system ptr
  system_ptr->ReleaseInstance();

  return ret;
}

CameraSpinnaker::CameraSpinnaker(unsigned int camNum,
                                 CameraTriggerMode triggerMode)
    : Camera(triggerMode) {
  // Get CameraPtr
  m_cam_ptr = this->retrieveCameraPtrWithCamNum(camNum);

  if (m_cam_ptr == nullptr) {
    cout << "Warning: Camera not found!" << endl;
    throw;
  }

  // Configure camera
  m_cam_ptr->PixelFormat = Spinnaker::PixelFormatInfoSelector_B8;
  // m_cam_ptr->Width = 3376;
  // m_cam_ptr->Height = 2704;
  // m_cam_ptr->BinningHorizontal = 2;
  // m_cam_ptr->BinningVertical = 2;
  m_cam_ptr->OffsetX = 0;
  m_cam_ptr->OffsetY = 0;

  // Configure Width
  // Configure Height
  // Configure OffsetX
  // Configure OffsetY
  // Disable Auto exposure
  // Set gamma to 1.0

  // Turn off auto exposure, set gamma to 1.0

  try {
    // Initialise camera
    m_cam_ptr->Init();

    // Print device info
    PrintDeviceInfo(m_cam_ptr);

    // Apply mono 8 pixel format
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->PixelFormat) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->PixelFormat)) {
      m_cam_ptr->PixelFormat.SetValue(PixelFormat_Mono8);
      cout << "Pixel format set to "
           << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..."
           << endl;
    } else {
      cout << "Pixel format not available..." << endl;
      // throw;
    }

    // Set Offset X
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->OffsetX) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->OffsetX)) {
      m_cam_ptr->OffsetX.SetValue(0);
      cout << "Offset X set to " << pCam->OffsetX.GetValue() << "..." << endl;
    } else {
      cout << "Offset X not available..." << endl;
      // throw;
    }

    // Set Offset Y
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->OffsetY) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->OffsetY)) {
      m_cam_ptr->OffsetY.SetValue(0);
      cout << "Offset Y set to " << pCam->OffsetY.GetValue() << "..." << endl;
    } else {
      cout << "Offset Y not available..." << endl;
      // throw;
    }

    // Set Width
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Width) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Width) &&
        m_cam_ptr->Width.GetInc() != 0 && m_cam_ptr->Width.GetMax() != 0) {
      m_cam_ptr->Width.SetValue(m_cam_ptr->Width.GetMax());
      cout << "Width set to " << m_cam_ptr->Width.GetValue() << "..." << endl;
    } else {
      cout << "Width not available..." << endl;
      // throw;
    }

    // Set Height
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Height) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Height) &&
        m_cam_ptr->Height.GetInc() != 0 && m_cam_ptr->Height.GetMax() != 0) {
      m_cam_ptr->Height.SetValue(m_cam_ptr->Height.GetMax());
      cout << "Height set to " << m_cam_ptr->Height.GetValue() << "..." << endl;
    } else {
      cout << "Height not available..." << endl;
      // throw;
    }

    // Disable Auto exposure
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->ExposureAuto) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->ExposureAuto)) {
      m_cam_ptr->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
      cout << "Automatic exposure disabled..." << endl;
    } else {
      cout << "Unable to disable automatic exposure." << endl << endl;
      // throw;
    }

    // Disable Gamma
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->GammaEnable) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->GammaEnable)) {
      Spinnaker::GenApi::IBoolean boolean = false;
      m_cam_ptr->GammaEnable.SetValue(boolean);
      cout << "Set GammaEnable to " << boolean << endl;
    } else {
      cout << "Unable to disable Gamma" << endl;
    }

    // Disable Autogain
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->GainAuto) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->GainAuto)) {
      Spinnaker::GenApi::IBoolean boolean = false;
      m_cam_ptr->GainAuto.SetValue(boolean);
      cout << "Set GainAuto to " << boolean << endl;
    } else {
      cout << "Unable to disable Auto-gain" << endl;
    }

  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

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

  try {
    // Note: In Spinnaker, Shutter is referred to as Exposure Time
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->ExposureTime)) {
      settings.shutter = m_cam_ptr->ExposureTime.GetValue();
    } else {
      cout << "Unable to read exposure time" << endl;
    }

    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Gain)) {
      settings.gain = m_cam_ptr->Gain.GetValue();
    } else {
      cout << "Unable to read gain value" << endl;
    }

  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

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
  try {
    m_cam_ptr->EndAcquisition();
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

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
    this->StopCapture();
  }

  // Gracefully destruct the camera
  m_cam_ptr->DeInit();
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
        Spinnaker::GenApi::IsReadable(ptrInterfaceDisplayName)) {
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
      if (IsAvailable(ptrDeviceVendorName) &&
          Spinnaker::GenApi::IsReadable(ptrDeviceVendorName)) {
        Spinnaker::GenICam::gcstring deviceVendorName =
            ptrDeviceVendorName->ToString();

        camera_info.vendor = deviceVendorName.c_str();

        cout << deviceVendorName << " ";
      }

      // Extract Device Model
      Spinnaker::GenApi::CStringPtr ptrDeviceModelName =
          nodeMapTLDevice.GetNode("DeviceModelName");
      if (IsAvailable(ptrDeviceModelName) &&
          Spinnaker::GenApi::IsReadable(ptrDeviceModelName)) {
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

Spinnaker::CameraPtr CameraSpinnaker::retrieveCameraPtrWithCamNum(
    unsigned int camNum) {
  // Initialse return ptr
  Spinnaker::CameraPtr = nullptr;

  // Vector that stores #cameras for each interface
  vector<int> cams_per_interface = {};

  // Create system ptr
  Spinnaker::SystemPtr system_ptr = Spinnaker::System::GetInstance();

  // Retrieve list of interfaces from the system
  Spinnaker::InterfaceList interfaceList = system_ptr->GetInterfaces();
  unsigned int numInterfaces = interfaceList.GetSize();

  // Go through interfaces to fill up
  for (unsigned int i = 0; i < numInterfaces; i++) {
    // Select interface
    auto interfacePtr = interfaceList.GetByIndex(i);

    // Check number of cameras in interface and append to cams_per_interface
    auto caminfo_vec = getCameraListFromSingleInterface(interfacePtr);
    cams_per_interface.push_back(caminfo_vec.size());
  }

  // We now find out which interface index and camera index does camNum
  // corresponds to

  int temp = (int)camNum + 1;

  int interface_indice = -1;
  int camera_indice = -1;

  for (unsigned int i = 0; i < numInterfaces; i++) {
    // If camera is in the interface
    if (temp <= cams_per_interface[i]) {
      interface_indice = i;
      camera_indice = temp - 1;

    } else {  // If camera is not in this interface, we update temp
      temp -= cams_per_interface[i];  // This will definitely be a positive,
                                      // non-zero number
    }
  }

  if (interface_indice > 0 && camera_indice > 0) {
    // We retrieve camera ptr
    ret = interfaceList.GetByIndex(interface_indice)
              ->GetCameras()
              .GetByIndex(camera_indice);
  } else {
    cout << "Warning: Invalid Camera Number: " << canNum << endl;
  }

  // Clear interface list
  interfaceList.Clear();

  // Release system ptr
  system_ptr->ReleaseInstance();

  return ret;
}

int CameraSpinnaker::PrintDeviceInfo(CameraPtr pCam) {
  int result = 0;
  cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;
  try {
    Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetTLDeviceNodeMap();
    Spinnaker::GenApi::FeatureList_t features;
    Spinnaker::GenApi::CCategoryPtr category =
        nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && Spinnaker::GenApi::IsReadable(category)) {
      category->GetFeatures(features);
      Spinnaker::GenApi::FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it) {
        Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
        cout << pfeatureNode->GetName() << " : ";
        Spinnaker::GenApi::CValuePtr pValue = (CValuePtr)pfeatureNode;
        cout << (Spinnaker::GenApi::IsReadable(pValue) ? pValue->ToString()
                                                       : "Node not readable");
        cout << endl;
      }
    } else {
      cout << "Device control information not available." << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}
