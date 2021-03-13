#include "SLCalibrationDialog.h"
#include "ui_SLCalibrationDialog.h"

#include <QSettings>
#include <QtTest/QTest>

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "ProjectorLC3000.h"
#include "ProjectorLC4500.h"
#include "ProjectorLC4500_versavis.h"
#include "ProjectorOpenGL.h"
#include "SLProjectorVirtual.h"

#include "CalibratorLocHom.h"
#include "CalibratorRBF.h"

#include <unordered_set>
#include "cvtools.h"

SLCalibrationDialog::SLCalibrationDialog(SLStudio *parent)
    : QDialog(parent),
      ui(new Ui::SLCalibrationDialog),
      reviewMode(false),
      timerInterval(100),
      delay(100) {
  ui->setupUi(this);
  setSizeGripEnabled(false);

  // Release this dialog on close
  this->setAttribute(Qt::WA_DeleteOnClose);

  QSettings settings("SLStudio");

  // Checkerboard parameters
  unsigned int checkerSize =
      settings.value("calibration/checkerSize", 8).toInt();
  ui->checkerSizeBox->setValue(checkerSize);
  unsigned int checkerRows =
      settings.value("calibration/checkerRows", 8).toInt();
  ui->checkerRowsBox->setValue(checkerRows);
  unsigned int checkerCols =
      settings.value("calibration/checkerCols", 8).toInt();
  ui->checkerColsBox->setValue(checkerCols);

  // Instatiate camera with software trigger
  int iNum = settings.value("camera/interfaceNumber", 0).toInt();
  int cNum = settings.value("camera/cameraNumber", 0).toInt();
  camera = Camera::NewCamera(iNum, cNum, triggerModeSoftware);

  delay = settings.value("trigger/delay", "100").toInt();

  // Set camera settings
  CameraSettings camSettings;
  camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
  camSettings.gain = 0.0;
  camera->setCameraSettings(camSettings);
  camera->startCapture();

  // Initialize projector
  int screenNum = settings.value("projector/screenNumber", -1).toInt();
  if (screenNum >= 0)
    projector = new ProjectorOpenGL(screenNum);
  else if (screenNum == -1)
    projector = new SLProjectorVirtual(screenNum);
  else if (screenNum == -2)
    projector = new ProjectorLC3000(0);
  else if (screenNum == -3)
    projector = new ProjectorLC4500(0);
  else if (screenNum == -4)
    projector = new ProjectorLC4500_versavis(0);
  else
    std::cerr << "SLCalibrationDialog: invalid projector id " << screenNum
              << std::endl;

  auto is_hardware_triggered = std::make_shared<bool>(false);
  auto void_is_hardware_triggered =
      std::static_pointer_cast<void>(is_hardware_triggered);
  projector->load_param("is_hardware_triggered", void_is_hardware_triggered);

  auto is_in_calibration_mode = std::make_shared<bool>(true);
  auto void_is_in_calibration_mode =
      std::static_pointer_cast<void>(is_in_calibration_mode);
  projector->load_param("is_in_calibration_mode", void_is_in_calibration_mode);

  projector->init();

  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);

  diamondPattern = settings.value("projector/diamondPattern", false).toBool();

  // Unique number of rows and columns
  if (diamondPattern) {
    screenCols = 2 * screenResX;
    screenRows = screenResY;
  } else {
    screenCols = screenResX;
    screenRows = screenResY;
  }

  // Create calibrator
  std::cout << "Calibration num cols: " << screenCols
            << ", num rows: " << screenRows << std::endl;
  calibrator = new CalibratorLocHom(screenCols, screenRows);
  // calibrator = new CalibratorRBF(screenCols, screenRows);

  connect(calibrator, SIGNAL(newSequenceResult(cv::Mat, unsigned int, bool)),
          this, SLOT(onNewSequenceResult(cv::Mat, uint, bool)));

  // Create recalibrator
  recalibrator = new CalibratorLocHom(screenCols, screenRows);
  // recalibrator = new CalibratorRBF(screenCols, screenRows);

  // Upload patterns to projector/GPU
  for (unsigned int i = 0; i < calibrator->getNPatterns(); i++) {
    cv::Mat pattern = calibrator->getCalibrationPattern(i);

    // general repmat
    pattern = cv::repeat(pattern, screenRows / pattern.rows + 1,
                         screenCols / pattern.cols + 1);
    pattern = pattern(cv::Range(0, screenRows), cv::Range(0, screenCols));

    if (diamondPattern) pattern = cvtools::diamondDownsample(pattern);

    projector->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);

    // Uncomment if you want to save calibration images
    // cv::imwrite(cv::format("pat_%d.bmp", i), pattern);
  }

  // Start live view
  timerInterval = delay + camSettings.shutter;
  liveViewTimer = startTimer(timerInterval);
}

void SLCalibrationDialog::timerEvent(QTimerEvent *event) {
  if (event->timerId() != liveViewTimer) {
    std::cerr << "Something fishy..." << std::endl << std::flush;
    return;
  }

  QApplication::processEvents();
  CameraFrame frame = camera->getFrame();

  cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
  frameCV = frameCV.clone();
  //    cv::resize(frameCV, frameCV, cv::Size(0, 0), 0.5, 0,5);

  ui->videoWidget->showFrameCV(frameCV);

  QApplication::processEvents();
}

SLCalibrationDialog::~SLCalibrationDialog() {
  delete camera;
  delete projector;
  delete calibrator;
  delete recalibrator;
  delete ui;
}

void SLCalibrationDialog::on_snapButton_clicked() {
  m_counter++;

  // If in review mode
  if (reviewMode) {
    reviewMode = false;
    ui->listWidget->clearSelection();
    liveViewTimer = startTimer(timerInterval);
    ui->snapButton->setText("Snap");
    return;
  }

  ui->snapButton->setEnabled(false);

  // Stop live view
  killTimer(liveViewTimer);

  vector<cv::Mat> frameSeq;

  for (unsigned int i = 0; i < calibrator->getNPatterns(); i++) {
    // Project pattern
    projector->displayPattern(i);
    QTest::qSleep(delay);

    // Effectuate sleep (necessary with some camera implementations)
    QApplication::processEvents();

    // Aquire frame
    CameraFrame frame = camera->getFrame();
    cv::Mat frameCV(frame.height, frame.width, CV_8U, frame.memory);
    frameCV = frameCV.clone();
    //        cv::resize(frameCV, frameCV, cv::Size(0, 0), 0.5, 0,5);

    // Show frame
    ui->videoWidget->showFrameCV(frameCV);

    // Save frame
    frameSeq.push_back(frameCV);
  }

  // Store frame sequence
  frameSeqs.push_back(frameSeq);

  // Add identifier to list
  QListWidgetItem *item = new QListWidgetItem(
      QString("Sequence %1").arg(frameSeqs.size()), ui->listWidget);
  item->setFlags(item->flags() |
                 Qt::ItemIsUserCheckable);  // set checkable flag
  item->setCheckState(Qt::Checked);         // AND initialize check state

  //    // Allow calibration if enough frame pairs
  //    if(ui->listWidget->count() >= 3)
  ui->calibrateButton->setEnabled(true);

  // Display white
  projector->displayWhite();

  /**
  #if 1
    // Write frame seq to disk
    for (unsigned int i = 0; i < frameSeq.size(); i++) {
      // QString filename =
      //    QString("calib_frameSeq__%1.bmp").arg(i, 2, 10, QChar('0'));
      std::string filename = "calib_frame_seq_" + std::to_string(i) + "_" +
                             std::to_string(m_counter) + ".bmp";
      cv::imwrite(filename, frameSeq[i]);
    }
  #endif
  **/

  // Restart live view
  liveViewTimer = startTimer(timerInterval);

  ui->snapButton->setEnabled(true);
}

void SLCalibrationDialog::
    on_calibrateButton_clicked() {  // We save the checkerboard settings
                                    // displayed in the calibration GUI prior to
  // the start of calibration
  // Checkerboard parameters
  QSettings settings("SLStudio");
  unsigned int checkerSize = ui->checkerSizeBox->value();
  settings.setValue("calibration/checkerSize", checkerSize);

  unsigned int checkerRows = ui->checkerRowsBox->value();
  settings.setValue("calibration/checkerRows", checkerRows);

  unsigned int checkerCols = ui->checkerColsBox->value();
  settings.setValue("calibration/checkerCols", checkerCols);

  std::cout << "Saving checkerboard settings before calibration starts ..."
            << std::endl;
  std::cout << "Checker grid size [mm]: " << checkerSize << std::endl;
  std::cout << "Number of grid intersection rows: " << checkerRows << std::endl;
  std::cout << "Number of grid intersection cols: " << checkerCols << std::endl;
  std::cout << "Done!" << std::endl;

  // Disable interface elements
  ui->calibrateButton->setEnabled(false);
  ui->listWidget->setEnabled(false);

  // Stop live view
  killTimer(liveViewTimer);
  reviewMode = true;
  ui->snapButton->setText("Live View");

  calibrator->reset();

  // Note which frame sequences are used
  activeFrameSeqs.clear();

  for (int i = 0; i < ui->listWidget->count(); i++) {
    if (ui->listWidget->item(i)->checkState() == Qt::Checked) {
      vector<cv::Mat> frameSeq(
          frameSeqs[i].begin(),
          frameSeqs[i].begin() + calibrator->getNPatterns());
      calibrator->addFrameSequence(frameSeq);
      activeFrameSeqs.push_back(i);

#if 1
      // Write frame seq to disk
      for (unsigned int j = 0; j < frameSeq.size(); j++) {
        // QString filename =
        //    QString("calib_frameSeq__%1.bmp").arg(i, 2, 10, QChar('0'));
        std::string filename = "calib_frame_seq_" + std::to_string(j) + "_" +
                               std::to_string(i) + ".bmp";
        cv::imwrite(filename, frameSeq[j]);
      }
#endif
    }
  }

  // Perform calibration
  calib = calibrator->calibrate();

  // Re-enable interface elements
  ui->calibrateButton->setEnabled(true);
  ui->listWidget->setEnabled(true);
  ui->saveButton->setEnabled(true);
}

void SLCalibrationDialog::on_listWidget_itemSelectionChanged() {
  // If selection was cleared
  if (ui->listWidget->selectedItems().isEmpty()) return;

  // Stop live view
  killTimer(liveViewTimer);
  reviewMode = true;
  ui->snapButton->setText("Live View");

  int currentRow = ui->listWidget->currentRow();
  ui->videoWidget->showFrameCV(frameSeqs[currentRow].back());
}

void SLCalibrationDialog::on_saveButton_clicked() {
  calib.frameWidth = camera->getFrameWidth();
  calib.frameHeight = camera->getFrameHeight();
  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);
  calib.screenResX = screenResX;
  calib.screenResY = screenResY;
  calib.calibrationDateTime = QDateTime::currentDateTime()
                                  .toString("DD.MM.YYYY HH:MM:SS")
                                  .toStdString();

  calib.save("calibration.xml");
  this->close();
}

void SLCalibrationDialog::onNewSequenceResult(cv::Mat img, unsigned int idx,
                                              bool success) {
  // Skip non-active frame sequences
  int idxListView = activeFrameSeqs[idx];

  // Append calibration result to frame sequence
  unsigned int N = calibrator->getNPatterns();
  if (frameSeqs[idxListView].size() == N)
    frameSeqs[idxListView].push_back(img);
  else
    frameSeqs[idxListView][N] = img;

  if (!success)  // uncheck
    ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);

  // Highlight
  ui->listWidget->setCurrentRow(idxListView);
  ui->listWidget->setFocus();

  QApplication::processEvents();
}

void SLCalibrationDialog::closeEvent(QCloseEvent *) {
  std::cout << "Deleting camera" << std::endl;
  delete camera;
  std::cout << "Deleting projector" << std::endl;
  delete projector;
  std::cout << "Deleting calibrator" << std::endl;
  delete calibrator;
  std::cout << "SLCalibrationDialog set to be deleted later" << std::endl;
  this->deleteLater();

  // Save calibration settings
  QSettings settings("SLStudio");
  unsigned int checkerSize = ui->checkerSizeBox->value();
  settings.setValue("calibration/checkerSize", checkerSize);
  unsigned int checkerRows = ui->checkerRowsBox->value();
  settings.setValue("calibration/checkerRows", checkerRows);
  unsigned int checkerCols = ui->checkerColsBox->value();
  settings.setValue("calibration/checkerCols", checkerCols);
}

void SLCalibrationDialog::on_pushButton_clicked() {
  std::cout << "Recalibration starting" << std::endl;

  recalibrator->reset();

  std::string directory = "/home/ltf/Desktop/horz5 cal pics/";
  int number_patterns = 12;
  int starting_indice = 0;

  int number_sequences = 50;
  std::unordered_set<int> seq_to_exclude = {26};

  for (int s = starting_indice; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "calib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          // return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }

  /**
std::string directory = "/home/ltf/Desktop/horz4_2_cal_pics/";
int number_patterns = 12;
int starting_indice = 1;
// int number_sequences = 5;
// int number_sequences = 21;
// int number_sequences = 69;
// int number_sequences = 142;
int number_sequences = 59;
// std::unordered_set<int> seq_to_exclude = {24, 40, 41, 42, 43, 44, 45, 46,
// 47};
std::unordered_set<int> seq_to_exclude = {24};

for (int s = starting_indice; s <= number_sequences; s++) {
  if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
    std::vector<cv::Mat> frame_sequence = {};
    for (int p = 0; p < number_patterns; p++) {
      std::string image_path = directory + "calib_frame_seq_" +
                               std::to_string(p) + "_" + std::to_string(s) +
                               ".bmp";
      // std::cout << "Trying to read " << image_path << std::endl;
      cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

      if (img.empty()) {
        std::cout << "Could not read the image: " << image_path << std::endl;
        // return;
      } else {
        // std::cout << "Successfully read image " << image_path << std::endl;
        frame_sequence.push_back(img);
      }
    }
    if (frame_sequence.size() == number_patterns) {
      recalibrator->addFrameSequence(frame_sequence);
    }
  } else {
    std::cout << "Excluded frame seq no: " << s << std::endl;
  }
}
**/

  /**
 std::string directory = "/home/ltf/horz2_redo_cal_pics/";
 int number_patterns = 12;
 int starting_indice = 1;
 // int number_sequences = 5;
 // int number_sequences = 21;
 // int number_sequences = 69;
 // int number_sequences = 142;
 int number_sequences = 74;
 std::unordered_set<int> seq_to_exclude = {2, 15, 22, 24, 30, 41, 51};

 for (int s = starting_indice; s <= number_sequences; s++) {
   if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
     std::vector<cv::Mat> frame_sequence = {};
     for (int p = 0; p < number_patterns; p++) {
       std::string image_path = directory + "calib_frame_seq_" +
                                std::to_string(p) + "_" + std::to_string(s) +
                                ".bmp";
       // std::cout << "Trying to read " << image_path << std::endl;
       cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

       if (img.empty()) {
         std::cout << "Could not read the image: " << image_path << std::endl;
         // return;
       } else {
         // std::cout << "Successfully read image " << image_path << std::endl;
         frame_sequence.push_back(img);
       }
     }
     if (frame_sequence.size() == number_patterns) {
       recalibrator->addFrameSequence(frame_sequence);
     }
   } else {
     std::cout << "Excluded frame seq no: " << s << std::endl;
   }
 }
 **/

  /**
  std::string directory = "/home/ltf/horz_redo_cal_pics/";
  int number_patterns = 12;
  int starting_indice = 1;
  // int number_sequences = 5;
  // int number_sequences = 21;
  // int number_sequences = 69;
  // int number_sequences = 142;
  int number_sequences = 177;
  std::unordered_set<int> seq_to_exclude = {
      15, 20, 22, 24,  38,  40,  41,  43,  45,  47,  50,  51,  53, 55,
      63, 85, 93, 100, 113, 148, 151, 153, 155, 157, 160, 163, 169};

  for (int s = starting_indice; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "calib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }
  **/

  /**
  std::string directory = "/home/ltf/horiz_pat_cal_pics/";
  int number_patterns = 12;
  // int number_sequences = 5;
  int number_sequences = 151;
  std::unordered_set<int> seq_to_exclude = {13, 39, 56, 75, 92};

  for (int s = 1; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "calib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }
  **/

  /**
  std::string directory = "/home/ltf/horiz_cali_pics/";
  int number_patterns = 12;
  int number_sequences = 82;
  std::unordered_set<int> seq_to_exclude = {27, 33, 44, 49, 62};

  for (int s = 1; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "calib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }
  **/

  /**
  std::string directory = "/home/ltf/cali_pics/";
  int number_patterns = 12;
  int number_sequences = 90;
  std::unordered_set<int> seq_to_exclude = {10, 78};

  for (int s = 1; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "calib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }

  number_patterns = 12;
  number_sequences = 33;
  seq_to_exclude = {};

  for (int s = 1; s <= number_sequences; s++) {
    if (seq_to_exclude.find(s) == seq_to_exclude.end()) {
      std::vector<cv::Mat> frame_sequence = {};
      for (int p = 0; p < number_patterns; p++) {
        std::string image_path = directory + "balib_frame_seq_" +
                                 std::to_string(p) + "_" + std::to_string(s) +
                                 ".bmp";
        // std::cout << "Trying to read " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
          std::cout << "Could not read the image: " << image_path << std::endl;
          return;
        } else {
          // std::cout << "Successfully read image " << image_path << std::endl;
          frame_sequence.push_back(img);
        }
      }
      if (frame_sequence.size() == number_patterns) {
        recalibrator->addFrameSequence(frame_sequence);
      }
    } else {
      std::cout << "Excluded frame seq no: " << s << std::endl;
    }
  }
  **/

  auto recali_results = recalibrator->calibrate();

  // Save results
  recali_results.frameWidth = camera->getFrameWidth();
  recali_results.frameHeight = camera->getFrameHeight();
  unsigned int screenResX, screenResY;
  projector->getScreenRes(&screenResX, &screenResY);
  recali_results.screenResX = screenResX;
  recali_results.screenResY = screenResY;
  recali_results.calibrationDateTime = QDateTime::currentDateTime()
                                           .toString("DD.MM.YYYY HH:MM:SS")
                                           .toStdString();

  recali_results.save("recalibration.xml");
}
