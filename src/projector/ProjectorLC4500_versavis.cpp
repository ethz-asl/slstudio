#include "ProjectorLC4500_versavis.h"

#include <QThread>
#include <iostream>

#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_usb.h"

#include <chrono>
#include <thread>

ProjectorLC4500_versavis::ProjectorLC4500_versavis(unsigned int)
    : nPatterns(0), isRunning(false) {
  // We are just going to fix it to shining 6 patterns
  nPatterns = 6;

  // Init ros node
  ros::M_string empty_args = {};
  ros::init(empty_args, "SLStudio");
  m_nh_ptr = std::make_unique<ros::NodeHandle>();

  // Set up subscriber
  m_sub = m_nh_ptr->subscribe(m_projector_trigger_topic, 5,
                              &ProjectorLC4500_versavis::sub_cb, this);

  // We use an Asyncrhonous Spinner since we dont have access to the main loop
  // Spinning is required or else subscriber will never receive messages
  m_spinner_ptr = std::make_unique<ros::AsyncSpinner>(1);
  m_spinner_ptr->start();
}

void ProjectorLC4500_versavis::setPattern(unsigned int patternNumber,
                                          const unsigned char *tex,
                                          unsigned int texWidth,
                                          unsigned int texHeight) {}

void ProjectorLC4500_versavis::displayPattern(unsigned int pattern_no) {
  // If it is the first pattern, we detect the start of the trigger before we
  // end the function to ensure synchronisation
  if (pattern_no == 0) {
    unsigned int current_count;
    unsigned int updated_count;

    {
      boost::mutex::scoped_lock lock(m_mutex);
      current_count = m_counter;
    }
    updated_count = current_count;

    {
      boost::mutex::scoped_lock lock(m_mutex);
      updated_count = m_counter;
    }

    std::cout << "Waiting for hardware trigger to be detected ";
    // Wait until count is updated (versavis has triggered)
    while (current_count == updated_count) {
      std::cout << ".";

      // Sleep for half a ms
      std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(500)));
      // Check if there are any updates to m_counter
      {
        boost::mutex::scoped_lock lock(m_mutex);
        updated_count = m_counter;
      }
    }
    std::cout << std::endl;

    std::cout << "Start image sequence!" << std::endl;
  }
}

void ProjectorLC4500_versavis::displayTexture(const unsigned char *tex,
                                              unsigned int texWidth,
                                              unsigned int texHeight) {}

void ProjectorLC4500_versavis::displayBlack() {}

void ProjectorLC4500_versavis::displayWhite() {}

void ProjectorLC4500_versavis::getScreenRes(unsigned int *nx,
                                            unsigned int *ny) {
  // Native resolution of Lightcrafter 4500
  *nx = 912;
  *ny = 1140;
}

ProjectorLC4500_versavis::~ProjectorLC4500_versavis() {
  m_spinner_ptr->stop();
  ros::shutdown();
}

void ProjectorLC4500_versavis::sub_cb(
    const versavis::TimeNumberedConstPtr &time_numbered_ptr) {
  // std::cout << "Message received!" << std::endl;

  boost::mutex::scoped_lock lock(m_mutex);
  m_counter = time_numbered_ptr->number;

  // std::cout << "Versavis counter: " << m_counter << std::endl;
}
