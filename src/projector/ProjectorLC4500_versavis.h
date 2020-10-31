#ifndef PROJECTORLC4500_VERSAVIS_H
#define PROJECTORLC4500_VERSAVIS_H

#include <sys/types.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <versavis/TimeNumbered.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "Projector.h"

// Projecotr implementation for LightCrafter 4500 USB Api
class ProjectorLC4500_versavis : public Projector {
 public:
  // Interface function
  ProjectorLC4500_versavis(unsigned int);
  // Define preset pattern sequence and upload to GPU
  void setPattern(unsigned int patternNumber, const unsigned char *tex,
                  unsigned int texWidth, unsigned int texHeight);
  void displayPattern(unsigned int patternNumber);
  // Upload and display pattern on the fly
  void displayTexture(const unsigned char *tex, unsigned int width,
                      unsigned int height);
  void displayBlack();
  void displayWhite();
  void getScreenRes(unsigned int *nx, unsigned int *ny);
  ~ProjectorLC4500_versavis();

  void sub_cb(const versavis::TimeNumberedConstPtr &time_numbered_ptr);

 private:
  unsigned int nPatterns;
  bool isRunning;
  std::unique_ptr<ros::NodeHandle> m_nh_ptr;
  std::unique_ptr<ros::AsyncSpinner> m_spinner_ptr;
  ros::Subscriber m_sub;
  std::string m_projector_trigger_topic = "/versavis/projector/image_time";
  unsigned int m_counter;
  boost::mutex m_mutex;
  void ros_init();
  void lc4500_init();
  void showError(std::string err);
  std::string m_ros_node_name = "SLStudio";
};

#endif
