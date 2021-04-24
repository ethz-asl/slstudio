#ifndef LIGHTCRAFTER_4500_PATTERN_API_H
#define LIGHTCRAFTER_4500_PATTERN_API_H

#include <memory>
#include <string>
#include <vector>

struct single_pattern {
  int trigger_type = 0;
  int pattern_number = 0;
  int bit_depth = 0;
  int led_select = 0;
  int image_indice = 0;
  bool invert_pattern = false;
  bool insert_black_frame = false;
  bool buffer_swap = false;
  bool trigger_out_prev = false;
};

class Lightcrafter_4500_pattern_api {
 public:
  Lightcrafter_4500_pattern_api();
  ~Lightcrafter_4500_pattern_api();

  int init();
  int close();
  int append_pattern_sequence(const single_pattern& pattern);
  int set_pattern_sequence(const std::vector<single_pattern>& pattern_vec);
  int play_pattern_sequence(const std::vector<single_pattern>& pattern_vec,
                            unsigned int exposure_period_us,
                            unsigned int frame_period_us);
  int clear_pattern_sequence();
  int send_pattern_sequence(unsigned int exposure_period_us,
                            unsigned int frame_period_us);
  int set_led_currents(unsigned char r, unsigned char g, unsigned char b);
  int get_led_currents(unsigned char& r, unsigned char& g, unsigned char& b);
  void print_projector_info();
  int set_pattern_mode();
  int set_video_mode();
  void showError(const std::string& err);
  int set_pat_seq_start();
  int set_pat_seq_pause();
  int set_pat_seq_stop();
  void sleep_ms(int ms);

 private:
  const int m_max_retries = 10;
  int set_pat_seq_mode(unsigned int desired_mode);
  std::vector<single_pattern> m_pattern_store = {};
  void check_and_fix_buffer_swaps();
  int validate_pattern();
};

#endif  // LIGHTCRAFTER_4500_PATTERN_API_H
