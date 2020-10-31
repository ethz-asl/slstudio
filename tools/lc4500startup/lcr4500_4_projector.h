#ifndef LCR4500_4_PROJECTOR_H
#define LCR4500_4_PROJECTOR_H

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

class Lcr4500_4_projector {
 public:
  Lcr4500_4_projector();
  ~Lcr4500_4_projector();
  int init();
  int close();
  int append_pattern_sequence(const single_pattern& pattern);
  int clear_pattern_sequence();
  int send_pattern_sequence(unsigned int exposure_period_us,
                            unsigned int frame_period_us);
  int set_led_currents(unsigned char r, unsigned char g, unsigned char b);
  int get_led_currents(unsigned char& r, unsigned char& g, unsigned char& b);
  void print_projector_info();
  int set_pattern_mode();
  int set_video_mode();
  void show_error(const std::string& err);
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

#endif  // LCR4500_4_PROJECTOR_H
