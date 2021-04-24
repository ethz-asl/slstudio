#ifndef PROJECTORLC4500VERSAVIS_H
#define PROJECTORLC4500VERSAVIS_H

#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <versavis/TimeNumbered.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <utility>
#include "Lightcrafter_4500_pattern_api.h"
#include "Projector.h"

// Projector implementation for LightCrafter 4500 USB Api
class ProjectorLC4500Versavis : public Projector {
 public:
  // Interface function
  ProjectorLC4500Versavis(unsigned int);
  // Define preset pattern sequence and upload to GPU
  void setPattern(unsigned int patternNumber, const unsigned char *tex,
                  unsigned int texWidth, unsigned int texHeight);
  void displayPattern(unsigned int patternNumber);
  //  Upload and display pattern on the fly
  void displayTexture(const unsigned char *tex, unsigned int width,
                      unsigned int height);
  void displayBlack();
  void displayWhite();

  void display_8_bit_image(int image_indice, int pattern_number);

  void getScreenRes(unsigned int *nx, unsigned int *ny);

  ~ProjectorLC4500Versavis();

  void sub_cb(const versavis::TimeNumberedConstPtr &time_numbered_ptr);

  virtual void loadParam(const std::string &param_name,
                         std::shared_ptr<void> param_ptr) override;

  virtual void init() override;

  virtual std::shared_ptr<void> getOutput(
      const std::string &output_name) override;

  void startProjection();

 private:
  void rosInit();
  void lc4500Iinit();
  void loadPatternSequence();
  void showError(const std::string &err);
  std::vector<single_pattern> m_pattern_sequence = {};
  std::vector<single_pattern> getCalibrationPatternSequence();
  std::vector<single_pattern> getScanningPatternSequenceSoftware();
  std::vector<single_pattern> getScanningPaternSequenceHardware();
  std::vector<single_pattern> getScanningPattern2Plus1Software();
  std::vector<single_pattern> getScanningPattern2Plus1Hardware();

  unsigned int nPatterns;
  bool isRunning;
  std::unique_ptr<ros::NodeHandle> m_nh_ptr;
  std::unique_ptr<ros::AsyncSpinner> m_spinner_ptr;
  ros::Subscriber m_sub;
  std::string m_projector_trigger_topic = "/versavis/projector/image_time";
  unsigned int m_counter;
  boost::mutex m_mutex;

  std::string m_ros_node_name = "SLStudio";
  bool m_is_hardware_triggered = false;
  Lightcrafter_4500_pattern_api m_projector;
  const unsigned char m_rgb_white[3] = {0, 60, 0};
  bool m_first_time_hardware_triggered = false;
  bool m_is_in_calibration_mode = false;
  bool m_is_2_plus_1_mode = false;

  // const unsigned int m_software_trigger_timings_us[2] = {8333, 8333};
  //  const unsigned int m_hardware_triggered_timings_us[2] = {8333, 8333};
  const bool m_is_30_hz_8333_us_exposure = false;
  const unsigned int m_software_trigger_timings_us[2] = {16667, 16667};
  const unsigned int m_hardware_triggered_timings_us[2] = {16667, 16667};
  const std::vector<int> m_calibration_image_indices = {3, 4, 5, 6};
  const std::vector<int> m_2_plus_1_image_indices = {11, 12, 13,
                                                     14};  //  PSP 2+1
  const std::vector<int> m_scanning_image_indices = {
      7, 8, 9, 10};  // Grasshopper Bin   Center

  bool m_display_horizontal_pattern = true;
  bool m_display_vertical_pattern = true;

  std::pair<std::chrono::time_point<std::chrono::system_clock>, ros::Time>
      m_buffer;
  ros::Time m_trigger_time;
  int m_pattern_no = -1;
  double m_trigger_tolerance = 0.01;
};

#endif
