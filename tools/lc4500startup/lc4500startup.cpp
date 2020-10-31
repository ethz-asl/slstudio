#include <iostream>

#include <chrono>
#include <thread>
#include "lcr4500_4_projector.h"

using namespace std;

int main() {
  cout << "lc4500startup: preparing LightCrafter 4500 for duty..." << endl;

  unsigned int exposure_period_us = 1000000;
  unsigned int frame_period_us = 1000000;

  Lcr4500_4_projector projector{};

  cout << "Initialising" << endl;
  if (projector.init() < 0) {
    cout << "Failed to init" << endl;
    return -1;
  }

  // Print out basic projector information as a sanity check
  projector.print_projector_info();

  single_pattern pattern_arr[6];
  // Refer to DLPC350_AddToPatLut in dlpc350_api.cpp for what each variable
  // does
  // 0 = Internal trigger, 1 = External positive, 2 = External negative, 3
  // = No Input Trigger (Continue from previous)

  pattern_arr[0].trigger_type = 0;
  pattern_arr[0].pattern_number = 0;
  pattern_arr[0].bit_depth = 8;
  pattern_arr[0].led_select = 7;
  pattern_arr[0].image_indice = 9;
  pattern_arr[0].invert_pattern = false;
  pattern_arr[0].insert_black_frame = true;
  pattern_arr[0].buffer_swap = true;
  pattern_arr[0].trigger_out_prev = false;

  pattern_arr[1].trigger_type = 3;
  pattern_arr[1].pattern_number = 1;
  pattern_arr[1].bit_depth = 8;
  pattern_arr[1].led_select = 7;
  pattern_arr[1].image_indice = 9;
  pattern_arr[1].invert_pattern = false;
  pattern_arr[1].insert_black_frame = true;
  pattern_arr[1].buffer_swap = false;
  pattern_arr[1].trigger_out_prev = false;

  pattern_arr[2].trigger_type = 3;
  pattern_arr[2].pattern_number = 2;
  pattern_arr[2].bit_depth = 8;
  pattern_arr[2].led_select = 7;
  pattern_arr[2].image_indice = 9;
  pattern_arr[2].invert_pattern = false;
  pattern_arr[2].insert_black_frame = true;
  pattern_arr[2].buffer_swap = false;
  pattern_arr[2].trigger_out_prev = false;

  pattern_arr[3].trigger_type = 0;
  pattern_arr[3].pattern_number = 0;
  pattern_arr[3].bit_depth = 8;
  pattern_arr[3].led_select = 7;
  pattern_arr[3].image_indice = 10;
  pattern_arr[3].invert_pattern = false;
  pattern_arr[3].insert_black_frame = true;
  pattern_arr[3].buffer_swap = true;
  pattern_arr[3].trigger_out_prev = false;

  pattern_arr[4].trigger_type = 3;
  pattern_arr[4].pattern_number = 1;
  pattern_arr[4].bit_depth = 8;
  pattern_arr[4].led_select = 7;
  pattern_arr[4].image_indice = 10;
  pattern_arr[4].invert_pattern = false;
  pattern_arr[4].insert_black_frame = true;
  pattern_arr[4].buffer_swap = false;
  pattern_arr[4].trigger_out_prev = false;

  pattern_arr[5].trigger_type = 3;
  pattern_arr[5].pattern_number = 2;
  pattern_arr[5].bit_depth = 8;
  pattern_arr[5].led_select = 7;
  pattern_arr[5].image_indice = 10;
  pattern_arr[5].invert_pattern = false;
  pattern_arr[5].insert_black_frame = true;
  pattern_arr[5].buffer_swap = false;
  pattern_arr[5].trigger_out_prev = false;

  // Add patterns
  cout << "Adding patterns" << endl;
  for (auto const& pattern : pattern_arr) {
    projector.append_pattern_sequence(pattern);
  }

  // Send patterns
  cout << "Sending patterns" << endl;
  if (projector.send_pattern_sequence(exposure_period_us, frame_period_us) <
      0) {
    cout << "Failed to send pattern" << endl;
    return -1;
  }

  // Start playing pattern
  cout << "Start playing pattern" << endl;
  if (projector.set_pat_seq_start() < 0) {
    cout << "Failed to start playing pattern" << endl;
    return -1;
  }

  // Let pattern play for 20 seconds
  cout << "Playing patter for 20 seconds " << endl;
  std::this_thread::sleep_for(std::chrono::microseconds(20000000));

  // Stop playing pattern
  cout << "Stop playing pattern" << endl;
  if (projector.set_pat_seq_stop() < 0) {
    cout << "Failed to stop playing pattern" << endl;
    return -1;
  }

  // Close Projector
  cout << "Closing" << endl;
  if (projector.close() < 0) {
    cout << "Failed to close" << endl;
    return -1;
  }

  return 0;
}
