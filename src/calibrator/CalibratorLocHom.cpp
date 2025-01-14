#include "CalibratorLocHom.h"
#include "CodecCalibration.h"

#include "cvtools.h"

#include <algorithm>

#include <QSettings>

CalibratorLocHom::CalibratorLocHom(unsigned int _screenCols,
                                   unsigned int _screenRows)
    : Calibrator(_screenCols, _screenRows) {
  // Create encoder/decoder
  encoder = new EncoderCalibration(screenCols, screenRows, CodecDirBoth);
  decoder = new DecoderCalibration(screenCols, screenRows, CodecDirBoth);

  this->N = encoder->getNPatterns();

  frameSeqs.resize(N);

  for (unsigned int i = 0; i < N; i++)
    patterns.push_back(encoder->getEncodingPattern(i));
}

CalibrationData CalibratorLocHom::calibrate() {
  QSettings settings("SLStudio");

  // Checkerboard parameters
  unsigned int checkerSize = settings.value("calibration/checkerSize").toInt();
  unsigned int checkerRows = settings.value("calibration/checkerRows").toInt();
  unsigned int checkerCols = settings.value("calibration/checkerCols").toInt();

  std::cout << "Checker grid size [mm]: " << checkerSize << std::endl;

  // Number of saddle points on calibration pattern
  cv::Size patternSize(checkerCols, checkerRows);

  // Number of calibration sequences
  unsigned nFrameSeq = frameSeqs.size();

  vector<cv::Mat> up(nFrameSeq), vp(nFrameSeq), shading(nFrameSeq),
      mask(nFrameSeq);

  // Decode frame sequences
  for (unsigned int i = 0; i < nFrameSeq; i++) {
    vector<cv::Mat> frames = frameSeqs[i];
    for (unsigned int f = 0; f < frames.size(); f++) {
      decoder->setFrame(f, frames[f]);
#if 0
                cv::imwrite(QString("frames[%1].png").arg(f).toStdString(), frames[f]);
#endif
    }
    decoder->decodeFrames(up[i], vp[i], mask[i], shading[i]);
#if 0
            cvtools::writeMat(shading[i], QString("shading[%1].mat").arg(i).toLocal8Bit());
            cvtools::writeMat(up[i], QString("up[%1].mat").arg(i).toLocal8Bit());
            cvtools::writeMat(vp[i], QString("vp[%1].mat").arg(i).toLocal8Bit());
#endif
  }

  unsigned int frameWidth = frameSeqs[0][0].cols;
  unsigned int frameHeight = frameSeqs[0][0].rows;

  // Generate local calibration object coordinates [mm]
  vector<cv::Point3f> Qi;
  for (int h = 0; h < patternSize.height; h++)
    for (int w = 0; w < patternSize.width; w++)
      Qi.push_back(cv::Point3f(checkerSize * w, checkerSize * h, 0.0));

  // Find calibration point coordinates for camera and projector
  std::vector<int> sequence_mapping =
      {};  // Record down how Q indices map to Frame number in scroll table
  vector<vector<cv::Point2f> > qc, qp;
  vector<vector<cv::Point3f> > Q;
  for (unsigned int i = 0; i < nFrameSeq; i++) {
    // std::cout << i << " 1" << std::endl;
    vector<cv::Point2f> qci;
    // Aid checkerboard extraction by slight blur
    // cv::GaussianBlur(shading[i], shading[i], cv::Size(5,5), 2, 2);
    // Extract checker corners
    // std::cout << i << " findChessboardCorners" << std::endl;
    bool success = cv::findChessboardCorners(shading[i], patternSize, qci,
                                             cv::CALIB_CB_ADAPTIVE_THRESH);
    if (!success)
      std::cout << "Calibrator: could not extract chess board corners on frame "
                   "seqence "
                << i << std::endl
                << std::flush;
    else {
      // Refine corner locations
      cv::cornerSubPix(
          shading[i], qci, cv::Size(5, 5), cv::Size(1, 1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                           20, 0.01));
    }
    // Draw colored chessboard
    cv::Mat shadingColor;
    cv::cvtColor(shading[i], shadingColor, cv::COLOR_GRAY2RGB);
    cv::drawChessboardCorners(shadingColor, patternSize, qci, success);
#if 0
    cv::imwrite("shadingColor.png", shadingColor);
#endif
    // Emit chessboard results
    std::cout << i << " Seq Chessboard Extraction Results: "
              << (success ? "Passed" : "Failed") << std::endl;
    emit newSequenceResult(shadingColor, i, success);

    if (success) {
      // Vectors of accepted points for current view
      vector<cv::Point2f> qpi_a;
      vector<cv::Point2f> qci_a;
      vector<cv::Point3f> Qi_a;

      // Loop through checkerboard corners
      for (unsigned int j = 0; j < qci.size(); j++) {
        const cv::Point2f &qcij = qci[j];

        // Collect neighbor points
        const unsigned int WINDOW_SIZE = 10;
        std::vector<cv::Point2f> N_qcij, N_qpij;

        // avoid going out of bounds
        unsigned int starth = max(int(qcij.y + 0.5) - WINDOW_SIZE, 0u);
        unsigned int stoph =
            min(int(qcij.y + 0.5) + WINDOW_SIZE, frameHeight - 1);
        unsigned int startw = max(int(qcij.x + 0.5) - WINDOW_SIZE, 0u);
        unsigned int stopw =
            min(int(qcij.x + 0.5) + WINDOW_SIZE, frameWidth - 1);

        for (unsigned int h = starth; h <= stoph; h++) {
          for (unsigned int w = startw; w <= stopw; w++) {
            // stay within mask
            if (mask[i].at<bool>(h, w)) {
              N_qcij.push_back(cv::Point2f(w, h));

              float upijwh = up[i].at<float>(h, w);
              float vpijwh = vp[i].at<float>(h, w);
              N_qpij.push_back(cv::Point2f(upijwh, vpijwh));
            }
          }
        }
        // std::cout << i << " findHomography " << N_qcij.size() << " " <<
        // N_qpij.size() << std::endl;
        // if enough valid points to build homography
        if (N_qpij.size() >= 50) {
          //                    std::cout << i << " findHomography" <<
          //                    std::endl;
          // translate qcij into qpij using local homography
          cv::Mat H = cv::findHomography(N_qcij, N_qpij, cv::LMEDS);
          if (!H.empty()) {
            cv::Point3d Q = cv::Point3d(
                cv::Mat(H * cv::Mat(cv::Point3d(qcij.x, qcij.y, 1.0))));
            cv::Point2f qpij = cv::Point2f(Q.x / Q.z, Q.y / Q.z);

            qpi_a.push_back(qpij);
            qci_a.push_back(qci[j]);
            Qi_a.push_back(Qi[j]);
          }
        }
      }

      if (!Qi_a.empty()) {
        // Store projector corner coordinates
        qp.push_back(qpi_a);

        // Store camera corner coordinates
        qc.push_back(qci_a);

        // Store world corner coordinates
        Q.push_back(Qi_a);

        // Store frame number for this image
        sequence_mapping.push_back(i);
      }
    }
  }

  if (Q.size() < 1) {
    std::cerr << "Error: not enough calibration sequences!" << std::endl;
    CalibrationData nanData;
    return nanData;
  }

  // calibrate the camera
  cv::Mat Kc, kc;
  std::vector<cv::Mat> cam_rvecs, cam_tvecs;
  cv::Size frameSize(frameWidth, frameHeight);

  double cam_error = cv::calibrateCamera(
      Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));

  /**
  double cam_error = cv::calibrateCamera(
      Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT +
          cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
  **/

  /**
  double cam_error = cv::calibrateCamera(
      Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
  **/

  /**
  double cam_error = cv::calibrateCamera(
      Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT +
          cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
  **/

  // Fix Camera Instrinsics using pre-calibrated values

  // double cam_error = 0.0f;

  /**
  Kc = cv::Mat::zeros(3, 3, cv::DataType<double>::type);

  Kc.at<double>(0, 0) = 1.72170386e+03;
  Kc.at<double>(0, 1) = 0.0f;
  Kc.at<double>(0, 2) = 512;
  Kc.at<double>(1, 0) = 0.0f;
  Kc.at<double>(1, 1) = 1.72170386e+03;
  Kc.at<double>(1, 2) = 384;
  Kc.at<double>(2, 0) = 0.0f;
  Kc.at<double>(2, 1) = 0.0f;
  Kc.at<double>(2, 2) = 1.0f;

  kc = cv::Mat::zeros(5, 1, cv::DataType<double>::type);

  kc.at<double>(0, 0) = -3.80073786e-01;
  kc.at<double>(1, 0) = -6.65011778e-02;
  kc.at<double>(2, 0) = 2.51256861e-04;
  kc.at<double>(3, 0) = -5.38413355e-04;
  kc.at<double>(4, 0) = 5.30454107e-02;
  **/

  // calibrate the projector
  cv::Mat Kp, kp;
  std::vector<cv::Mat> proj_rvecs, proj_tvecs;
  cv::Size screenSize(screenCols, screenRows);

  std::cout << "Calibration num cols: " << screenCols
            << ", num rows: " << screenRows << std::endl;

  Kp = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
  Kp.at<double>(0, 0) = 2200;  // Focal length [mm] guess based on
                               // existing calibration attempts
  Kp.at<double>(1, 1) = 2200;
  Kp.at<double>(0, 2) = 912;       // SLStudio convention for diamond pixels, so
                                   // there are double the number of cols
  Kp.at<double>(1, 2) = 1140 - 1;  // 100% Offset for lightcrafter 4500 so its
                                   // the full height not the standard height/2,
                                   // must minus 1 or else opencv complains
  Kp.at<double>(2, 2) = 1;

  std::cout << "Calibration matrix guess: " << std::endl;
  std::cout << Kp << std::endl;

  double proj_error = cv::calibrateCamera(
      Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs,
      cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_ASPECT_RATIO,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));

  // Fix Projector Instrinsics using pre-calibrated values

  /**
  double proj_error = 0.0f;

  Kp = cv::Mat::zeros(3, 3, cv::DataType<double>::type);

  Kp.at<double>(0, 0) = 2.19183569e+03;
  Kp.at<double>(0, 1) = 0.0f;
  Kp.at<double>(0, 2) = 8.81184509e+02;
  Kp.at<double>(1, 0) = 0.0f;
  Kp.at<double>(1, 1) = 2.19183569e+03;
  Kp.at<double>(1, 2) = 1.16716187e+03;
  Kp.at<double>(2, 0) = 0.0f;
  Kp.at<double>(2, 1) = 0.0f;
  Kp.at<double>(2, 2) = 1.0f;

  kp = cv::Mat::zeros(5, 1, cv::DataType<double>::type);

  kp.at<double>(0, 0) = 4.54267673e-02;
  kp.at<double>(1, 0) = -1.87380165e-01;
  kp.at<double>(2, 0) = -3.58262099e-03;
  kp.at<double>(3, 0) = -4.46699938e-04;
  kp.at<double>(4, 0) = 1.07268170e-01;
  **/

  /**
  double proj_error = cv::calibrateCamera(
      Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs,
      cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_ASPECT_RATIO,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
  **/

  // + cv::CALIB_FIX_PRINCIPAL_POINT

  /**
  double proj_error = cv::calibrateCamera(
      Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
          cv::CALIB_ZERO_TANGENT_DIST,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
  **/

  /**
  double proj_error = cv::calibrateCamera(
      Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs,
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 +
          cv::CALIB_ZERO_TANGENT_DIST,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));
**/

  // stereo calibration
  cv::Mat Rp, Tp, E, F;
#if CV_MAJOR_VERSION < 3
  double stereo_error = cv::stereoCalibrate(
      Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                       DBL_EPSILON),
      cv::CALIB_FIX_INTRINSIC);
#else

  double stereo_error = cv::stereoCalibrate(
      Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F,
      cv::CALIB_FIX_INTRINSIC,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                       DBL_EPSILON));

#endif

  CalibrationData calData(Kc, kc, cam_error, Kp, kp, proj_error, Rp, Tp,
                          stereo_error);

  calData.print(std::cout);

  // Determine per-view reprojection errors:
  std::vector<float> cam_error_per_view(Q.size());
  cam_error_per_view.resize(Q.size());
  std::vector<float> proj_error_per_view(Q.size());
  proj_error_per_view.resize(Q.size());

  for (unsigned int i = 0; i < (unsigned int)Q.size(); ++i) {
    int n = (int)Q[i].size();

    // These lines are required if you are using a pre-calibrated camera
    // intrinsics
    /**
    cam_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
    cam_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
    cv::solvePnP(cv::Mat(Q[i]), qc[i], Kc, kc, cam_rvecs[i], cam_tvecs[i]);
    **/

    vector<cv::Point2f> qc_proj;
    cv::projectPoints(cv::Mat(Q[i]), cam_rvecs[i], cam_tvecs[i], Kc, kc,
                      qc_proj);

    float err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++) {
      cv::Point2f d = qc[i][j] - qc_proj[j];
      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
    cam_error_per_view[i] = (float)err / n;

    // These lines are required if you are using a pre-calibrated projector
    // intrinsics
    /**
    proj_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
    proj_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
    cv::solvePnP(cv::Mat(Q[i]), qp[i], Kp, kp, proj_rvecs[i], proj_tvecs[i]);
    **/

    vector<cv::Point2f> qp_proj;
    cv::projectPoints(cv::Mat(Q[i]), proj_rvecs[i], proj_tvecs[i], Kp, kp,
                      qp_proj);
    err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++) {
      cv::Point2f d = qp[i][j] - qp_proj[j];
      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
    proj_error_per_view[i] = (float)err / n;

    std::cout << "Error " << i + 1 << ") Sequence " << sequence_mapping[i] + 1
              << "):\n\tcam:" << cam_error_per_view[i]
              << " proj:" << proj_error_per_view[i] << std::endl;
  }

  // Print Stereo Error
  std::cout << "Stereo Error: " << stereo_error << std::endl;

  return calData;
}
