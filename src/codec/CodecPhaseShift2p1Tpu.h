#ifndef CODECPhaseShift2p1Tpu_H
#define CODECPhaseShift2p1Tpu_H

#include "Codec.h"

class EncoderPhaseShift2p1Tpu : public Encoder {
 public:
  EncoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows,
                          CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth);

 private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift2p1Tpu : public Decoder {
 public:
  DecoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows,
                          CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
  ~DecoderPhaseShift2p1Tpu();

 private:
  std::vector<cv::Mat> frames;
  std::vector<cv::Point2d> shiftHistory;
  cv::Mat_<float> *lastShading;
};

#endif  // CODECPhaseShift2p1Tpu_H
