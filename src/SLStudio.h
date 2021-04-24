/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLSTUDIO_H
#define SLSTUDIO_H

#include <QMainWindow>
#include <QPointer>
#include <QSettings>
#include <QTimer>

#include "SLPointCloudWidget.h"
#include "SLScanWorker.h"
#include "SLTrackerDialog.h"
#include "SLTrackerWorker.h"
#include "SLVideoDialog.h"

#include "Camera.h"
#include "Codec.h"
#include "Projector.h"
#include "ProjectorLC4500Versavis.h"

#include <memory>

namespace Ui {
class SLStudio;
}

class SLStudio : public QMainWindow {
  Q_OBJECT

 public:
  explicit SLStudio(QWidget *parent = 0);
  void closeEvent(QCloseEvent *event);
  ~SLStudio();

 private slots:
  void onActionStart();
  void onActionStop();
  void onScanWorkerFinished();

  void onActionCalibration();
  void onActionLoadCalibration();
  void onActionPreferences();
  void onActionExportCalibration();

  void updateDisplayRate();
  void receiveNewPointCloud(PointCloudConstPtr pointCloud);

  void imshow(const char *windowName, cv::Mat im, unsigned int x,
              unsigned int y);
  void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);

  void onShowHistogram(cv::Mat im);
  void onShowShading(cv::Mat im);
  void onShowCameraFrames(std::vector<cv::Mat> frameSeq);
  void onShowDecoderUp(cv::Mat im);
  void onShowDecoderVp(cv::Mat im);
  void onActionAbout();

  void on_linearityTest_clicked();
  void on_generatePatterns_clicked();
  void on_startProjector_clicked();
  void on_stopProjector_clicked();

 signals:
  void newPointCloud(PointCloudConstPtr pointCloud);

 private:
  Ui::SLStudio *ui;
  std::vector<unsigned int> displayIntervals;

  SLScanWorker *scanWorker;
  QThread *scanWorkerThread;

  SLDecoderWorker *decoderWorker;
  QThread *decoderThread;

  SLTriangulatorWorker *triangulatorWorker;
  QThread *triangulatorThread;

  QTime *time;
  QSettings *settings;

  SLVideoDialog *histogramDialog, *shadingDialog, *cameraFramesDialog,
      *decoderUpDialog, *decoderVpDialog;
  SLTrackerDialog *trackerDialog;

  std::unique_ptr<ProjectorLC4500Versavis> projector_ptr;

 public:
};

#endif
