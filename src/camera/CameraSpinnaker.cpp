#include "CameraSpinnaker.h"
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

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
  // Initialise System pointer
  m_sys_ptr = Spinnaker::System::GetInstance();

  // Get CameraPtr
  m_cam_ptr = this->retrieveCameraPtrWithCamNum(camNum);

  if (m_cam_ptr == nullptr) {
    cout << "Warning: Camera not found! Aborting..." << endl;
    throw;
  }

  // Initialise camera
  try {
    m_cam_ptr->Init();
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Print device info
  cout << endl << "***** Print device info start *****" << endl;

  try {
    PrintDeviceInfo(m_cam_ptr);
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  cout << "***** Print device info end *****" << endl << endl;

  // Set no pixel color filter
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->PixelColorFilter) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->PixelColorFilter)) {
      m_cam_ptr->PixelColorFilter.SetValue(Spinnaker::PixelColorFilter_None);
      cout << "Disabled all pixel color filters" << endl;
    } else {
      cout << "Failed to disable pixel color filters" << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set pixel format to be Mono8
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->PixelFormat) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->PixelFormat)) {
      m_cam_ptr->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono8);
      cout << "Pixel format set to "
           << m_cam_ptr->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..."
           << endl;
    } else {
      cout << "Pixel format not available..." << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set Offset X
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->OffsetX) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->OffsetX)) {
      m_cam_ptr->OffsetX.SetValue(0);
      cout << "Offset X set to " << m_cam_ptr->OffsetX.GetValue() << "..."
           << endl;
    } else {
      cout << "Offset X not available..." << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set Offset Y
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->OffsetY) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->OffsetY)) {
      m_cam_ptr->OffsetY.SetValue(0);
      cout << "Offset Y set to " << m_cam_ptr->OffsetY.GetValue() << "..."
           << endl;
    } else {
      cout << "Offset Y not available..." << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set Width (Might not be able to be changed)
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Width) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Width) &&
        m_cam_ptr->Width.GetInc() != 0 && m_cam_ptr->Width.GetMax() != 0) {
      m_cam_ptr->Width.SetValue(m_cam_ptr->Width.GetMax());
      cout << "Width set to " << m_cam_ptr->Width.GetValue() << "..." << endl;
    } else {
      cout << "Width not available..." << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set Height (Might not be able to be changed)
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Height) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Height) &&
        m_cam_ptr->Height.GetInc() != 0 && m_cam_ptr->Height.GetMax() != 0) {
      m_cam_ptr->Height.SetValue(m_cam_ptr->Height.GetMax());
      cout << "Height set to " << m_cam_ptr->Height.GetValue() << "..." << endl;
    } else {
      cout << "Height not available..." << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Auto exposure
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->ExposureAuto) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->ExposureAuto)) {
      m_cam_ptr->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
      cout << "Automatic exposure disabled..." << endl;
    } else {
      cout << "Unable to disable automatic exposure." << endl << endl;
      // throw;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Gamma
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->GammaEnable) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->GammaEnable)) {
      m_cam_ptr->GammaEnable.SetValue(false);
      cout << "Set GammaEnable to " << false << endl;
    } else {
      cout << "Unable to disable Gamma" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set gamma value to 1.0
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Gamma) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Gamma)) {
      m_cam_ptr->Gamma.SetValue(1.0f);
      cout << "Set Gamma to " << 1.0f << endl;
    } else {
      cout << "Unable to disable Gamma" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Autogain
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->GainAuto) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->GainAuto)) {
      m_cam_ptr->GainAuto.SetValue(Spinnaker::GainAuto_Off);
      cout << "Set GainAuto to " << false << endl;
    } else {
      cout << "Unable to disable Auto-gain" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set stream buffer count mode to Manual
  try {
    if (Spinnaker::GenApi::IsReadable(
            m_cam_ptr->TLStream.StreamBufferCountMode) &&
        Spinnaker::GenApi::IsWritable(
            m_cam_ptr->TLStream.StreamBufferCountMode)) {
      m_cam_ptr->TLStream.StreamBufferCountMode.SetValue(
          Spinnaker::StreamBufferCountMode_Manual);
      cout << "Set Stream Buffer Count Mode to Manual" << endl;
    } else {
      cout << "Unable set Stream Buffer Count Mode to Manual" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set to retrieve only the newest image
  try {
    if (Spinnaker::GenApi::IsReadable(
            m_cam_ptr->TLStream.StreamBufferHandlingMode) &&
        Spinnaker::GenApi::IsWritable(
            m_cam_ptr->TLStream.StreamBufferHandlingMode)) {
      m_cam_ptr->TLStream.StreamBufferHandlingMode.SetValue(
          Spinnaker::StreamBufferHandlingMode_NewestOnly);
      cout << "Set only show newest image" << endl;
    } else {
      cout << "Unable to set to only show newest image" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Sharpness Correction
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->SharpeningEnable) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->SharpeningEnable)) {
      m_cam_ptr->SharpeningEnable.SetValue(false);
      cout << "Disable Sharpness Correction" << endl;
    } else {
      cout << "Unable to disable sharpness correction" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Sharpness Auto
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->SharpeningAuto) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->SharpeningAuto)) {
      m_cam_ptr->SharpeningAuto.SetValue(false);
      cout << "Disable Auto-Sharpen" << endl;
    } else {
      cout << "Unable to disable Auto-Sharpen" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Disable Saturation Auto
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->SaturationEnable) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->SaturationEnable)) {
      m_cam_ptr->SaturationEnable.SetValue(false);
      cout << "Disable Auto-Saturation" << endl;
    } else {
      cout << "Unable to disable Auto-Saturation" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set Black Level to Zero
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->BlackLevel) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->BlackLevel)) {
      m_cam_ptr->BlackLevel.SetValue(0.0f);
      cout << "Set black level to " << m_cam_ptr->BlackLevel.GetValue() << endl;
    } else {
      cout << "Unable to set black value to zero" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set reasonable default settings
  CameraSettings settings;
  settings.shutter = 16.667;
  // settings.shutter = 33.333;
  settings.gain = 0.0;
  this->setCameraSettings(settings);
  return;
}

CameraSettings CameraSpinnaker::getCameraSettings() {
  // Get settings:
  CameraSettings settings;

  try {
    // Note: In Spinnaker, Shutter is referred to as Exposure Time
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->ExposureTime)) {
      // Convert exposure time in microseconds to shutter which is in ms
      settings.shutter = m_cam_ptr->ExposureTime.GetValue() / 1000.0;
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
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->ExposureTime) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->ExposureTime)) {
      // Note exposure time is in micro-seconds
      // In settings, shutter is in ms so we need to perform a conversion
      m_exposure_time_micro_s = settings.shutter * 1000.0f;
      m_cam_ptr->ExposureTime.SetValue(settings.shutter * 1000.0f);
      cout << "Set exposure time to [micro s]: "
           << m_cam_ptr->ExposureTime.GetValue() << endl;
    } else {
      cout << "Could not set exposure time" << endl;
    }

    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Gain) &&
        Spinnaker::GenApi::IsWritable(m_cam_ptr->Gain)) {
      m_cam_ptr->Gain.SetValue(settings.gain);
      // m_cam_ptr->Gain.SetValue(-10.0f);
      cout << "Set gain to: " << m_cam_ptr->Gain.GetValue() << endl;
    } else {
      cout << "Could not set gain" << endl;
    }

  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }
}

void CameraSpinnaker::startCapture() {
  cout << "Starting capture" << endl;

  // Print camera settings
  CameraSettings settings = this->getCameraSettings();
  std::cout << "\tShutter: " << settings.shutter << " ms" << std::endl;
  std::cout << "\tGain: " << settings.gain << " dB" << std::endl;

  // Make sure trigger mode is disabled before we configure it
  try {
    if (m_cam_ptr->TriggerMode.GetAccessMode() != Spinnaker::GenApi::RW) {
      cout << "Unable to disable trigger mode. Aborting..." << endl;
      throw;
    }
    m_cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
    cout << "Trigger mode disabled..." << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  if (triggerMode == triggerModeHardware) {
    // Configure for hardware trigger
    try {
      // Set the trigger source to hardware (using 'Line0')
      if (m_cam_ptr->TriggerSource == NULL ||
          m_cam_ptr->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW) {
        cout << "Unable to set trigger mode (node retrieval). Aborting..."
             << endl;
        throw;
      }
      m_cam_ptr->TriggerSource.SetValue(Spinnaker::TriggerSource_Line0);
      cout << "Trigger source set to hardware (line 0)..." << endl;
    } catch (Spinnaker::Exception& e) {
      cout << "Error: " << e.what() << endl;
    }
  } else if (triggerMode == triggerModeSoftware) {
    // Set the trigger source to software
    try {
      if (m_cam_ptr->TriggerSource == NULL ||
          m_cam_ptr->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW) {
        cout << "Unable to set trigger mode (node retrieval). Aborting..."
             << endl;
        throw;
      }
      m_cam_ptr->TriggerSource.SetValue(Spinnaker::TriggerSource_Software);
      cout << "Trigger source set to software..." << endl;
    } catch (Spinnaker::Exception& e) {
      cout << "Error: " << e.what() << endl;
    }
  }

  // Set trigger selector
  try {
    if (m_cam_ptr->TriggerSelector == NULL ||
        m_cam_ptr->TriggerSelector.GetAccessMode() != Spinnaker::GenApi::RW) {
      cout << "Unable to set trigger selector. Aborting..." << endl;
      throw;
    }
    m_cam_ptr->TriggerSelector.SetValue(
        Spinnaker::TriggerSelectorEnums::TriggerSelector_FrameStart);
    cout << "Set Trigger Selector to FrameStart" << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  /** Set trigger activation
  try {
    if (m_cam_ptr->TriggerActivation == NULL ||
        m_cam_ptr->TriggerActivation.GetAccessMode() != Spinnaker::GenApi::RW)
  { cout << "Unable to set trigger activation. Aborting..." << endl; throw;
    }
    m_cam_ptr->TriggerActivation.SetValue(
        Spinnaker::TriggerActivationEnums::TriggerActivation_RisingEdge);
    cout << "Set Trigger Activation to Rising Edge" << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }
  **/

  // Turn trigger mode back on
  try {
    if (m_cam_ptr->TriggerSource == NULL ||
        m_cam_ptr->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW) {
      cout << "Unable to set trigger mode (node retrieval). Aborting..."
           << endl;
      throw;
    }
    m_cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_On);
    cout << "Trigger mode turned back on..." << endl << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Set acquisition mode to continuous
  try {
    if (m_cam_ptr->AcquisitionMode == NULL ||
        m_cam_ptr->AcquisitionMode.GetAccessMode() != Spinnaker::GenApi::RW) {
      cout << "Unable to set acquisition mode to continuous. Aborting..."
           << endl;
      throw;
    }
    m_cam_ptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
    cout << "Acquisition mode set to continuous..." << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Begin acquiring images
  try {
    m_cam_ptr->BeginAcquisition();
    cout << "Acquisition start" << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  capturing = true;
}

void CameraSpinnaker::stopCapture() {
  try {
    cout << "Stopping capture" << endl;

    // Set trigger to be disabled
    if (m_cam_ptr->TriggerMode == NULL ||
        m_cam_ptr->TriggerMode.GetAccessMode() != Spinnaker::GenApi::RW) {
      cout << "Unable to disable trigger mode. Aborting..." << endl;
      throw;
    }
    m_cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
    cout << "Trigger mode disabled..." << endl;

  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  // Stop getting images
  try {
    m_cam_ptr->EndAcquisition();
    cout << "End acquisition" << endl;
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }

  capturing = false;
}

CameraFrame CameraSpinnaker::getFrame() {
  CameraFrame frame;

  // Activate software trigger
  if (triggerMode == triggerModeSoftware) {
    try {
      if (m_cam_ptr->TriggerSoftware == NULL ||
          m_cam_ptr->TriggerSoftware.GetAccessMode() != Spinnaker::GenApi::WO) {
        cout << "Unable to execute trigger..." << endl;
      }
      m_cam_ptr->TriggerSoftware.Execute();
      // cout << "Executed software trigger" << endl;

    } catch (Spinnaker::Exception& e) {
      cout << "Error: " << e.what() << endl;
    }
  }

  // Fill in frame
  Spinnaker::ImagePtr img_ptr = nullptr;

  int attempts = 100;

  for (int i = 0; i < attempts; i++) {
    std::this_thread::sleep_for(
        std::chrono::microseconds((uint32_t)(10 * m_exposure_time_micro_s)));

    try {
      img_ptr = m_cam_ptr->GetNextImage();
    } catch (Spinnaker::Exception& e) {
      cout << "Error: " << e.what() << endl;
    }

    try {
      if (img_ptr != nullptr &&
          img_ptr->GetImageStatus() == Spinnaker::IMAGE_NO_ERROR &&
          !img_ptr->IsIncomplete()) {
        // cout << "Image Acquisition Successful!" << endl;
        frame.timeStamp = img_ptr->GetTimeStamp();
        frame.height = img_ptr->GetHeight();
        frame.width = img_ptr->GetWidth();
        frame.memory = (unsigned char*)img_ptr->GetData();
        frame.sizeBytes = img_ptr->GetBufferSize();
        break;
      } else {
        cout << "Image Acquisition Failed! Attempt " << i << "/" << attempts
             << endl;
      }
    } catch (Spinnaker::Exception& e) {
      cout << "Error: " << e.what() << endl;
    }
  }

  return frame;
}

size_t CameraSpinnaker::getFrameSizeBytes() {
  return getFrameWidth() * getFrameHeight();
}

size_t CameraSpinnaker::getFrameWidth() {
  size_t answer = 0;
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Width)) {
      answer = (size_t)m_cam_ptr->WidthMax.GetValue();
    } else {
      cout << "Unable to read width" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }
  return answer;
}

size_t CameraSpinnaker::getFrameHeight() {
  size_t answer = 0;
  try {
    if (Spinnaker::GenApi::IsReadable(m_cam_ptr->Height)) {
      answer = (size_t)m_cam_ptr->Height.GetValue();
    } else {
      cout << "Unable to read height" << endl;
    }
  } catch (Spinnaker::Exception& e) {
    cout << "Error: " << e.what() << endl;
  }
  return answer;
}

CameraSpinnaker::~CameraSpinnaker() {
  if (capturing) {
    // Stop camera transmission
    this->stopCapture();
  }

  // Gracefully destruct the camera
  m_cam_ptr->DeInit();
  m_cam_ptr = nullptr;  // Need to add this or else will have the error Can't
                        // clear a camera because something still holds a
                        // reference to the camera [-1004]
  cout << "Deinitialised camera" << endl;

  // Clear system pointer
  m_sys_ptr->ReleaseInstance();
  cout << "Release system pointer" << endl;
}

vector<CameraInfo> CameraSpinnaker::getCameraListFromSingleInterface(
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
      Spinnaker::CameraPtr m_cam_ptr = camList.GetByIndex(i);

      // Tentatively interpret the index as the busID
      camera_info.busID = (unsigned int)i;
      cout << "\tDevice " << i << " ";

      // Retrieve TL device nodemap; please see NodeMapInfo example for
      // additional comments on transport layer nodemaps
      Spinnaker::GenApi::INodeMap& nodeMapTLDevice =
          m_cam_ptr->GetTLDeviceNodeMap();

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
            ptrDeviceVendorName->ToString() + " (Spinnaker)";

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
  Spinnaker::CameraPtr ret;

  // Vector that stores #cameras for each interface
  vector<int> cams_per_interface = {};

  // Retrieve list of interfaces from the system
  Spinnaker::InterfaceList interfaceList = m_sys_ptr->GetInterfaces();
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

  if (interface_indice >= 0 && camera_indice >= 0) {
    // We retrieve camera ptr
    ret = interfaceList.GetByIndex(interface_indice)
              ->GetCameras()
              .GetByIndex(camera_indice);
    cout << "Received camera pointer" << endl;
  } else {
    cout << "Warning: Invalid Camera Number: " << camNum << endl;
  }

  // Clear interface list
  interfaceList.Clear();

  return ret;
}

int CameraSpinnaker::PrintDeviceInfo(Spinnaker::CameraPtr m_cam_ptr) {
  int result = 0;
  cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;
  try {
    Spinnaker::GenApi::INodeMap& nodeMap = m_cam_ptr->GetTLDeviceNodeMap();
    Spinnaker::GenApi::FeatureList_t features;
    Spinnaker::GenApi::CCategoryPtr category =
        nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && Spinnaker::GenApi::IsReadable(category)) {
      category->GetFeatures(features);
      Spinnaker::GenApi::FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it) {
        Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
        cout << pfeatureNode->GetName() << " : ";
        Spinnaker::GenApi::CValuePtr pValue =
            (Spinnaker::GenApi::CValuePtr)pfeatureNode;
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
