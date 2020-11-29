/********************************************************************************
** Form generated from reading UI file 'SLStudio.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLSTUDIO_H
#define UI_SLSTUDIO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "SLPointCloudWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLStudio
{
public:
    QAction *actionQuit;
    QAction *actionStart;
    QAction *actionStop;
    QAction *actionCalibration;
    QAction *actionLoadCalibration;
    QAction *actionExportCalibration;
    QAction *actionPreferences;
    QAction *actionSavePointCloud;
    QAction *actionSaveScreenshot;
    QAction *actionTracking;
    QAction *actionAbout;
    QAction *actionTracker;
    QAction *actionUpdatePointClouds;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    SLPointCloudWidget *pointCloudWidget;
    QPushButton *pushButton;
    QSpinBox *pattern_num_spin;
    QSpinBox *frame_num_spin;
    QMenuBar *menuBar;
    QMenu *menuCalibration;
    QMenu *menuSLStudio;
    QMenu *menuScan;
    QMenu *menuView;
    QToolBar *toolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SLStudio)
    {
        if (SLStudio->objectName().isEmpty())
            SLStudio->setObjectName(QStringLiteral("SLStudio"));
        SLStudio->resize(828, 544);
        SLStudio->setAutoFillBackground(false);
        actionQuit = new QAction(SLStudio);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        actionQuit->setMenuRole(QAction::QuitRole);
        actionStart = new QAction(SLStudio);
        actionStart->setObjectName(QStringLiteral("actionStart"));
        actionStop = new QAction(SLStudio);
        actionStop->setObjectName(QStringLiteral("actionStop"));
        actionStop->setEnabled(false);
        actionCalibration = new QAction(SLStudio);
        actionCalibration->setObjectName(QStringLiteral("actionCalibration"));
        actionLoadCalibration = new QAction(SLStudio);
        actionLoadCalibration->setObjectName(QStringLiteral("actionLoadCalibration"));
        actionExportCalibration = new QAction(SLStudio);
        actionExportCalibration->setObjectName(QStringLiteral("actionExportCalibration"));
        actionPreferences = new QAction(SLStudio);
        actionPreferences->setObjectName(QStringLiteral("actionPreferences"));
        actionSavePointCloud = new QAction(SLStudio);
        actionSavePointCloud->setObjectName(QStringLiteral("actionSavePointCloud"));
        actionSavePointCloud->setEnabled(false);
        actionSaveScreenshot = new QAction(SLStudio);
        actionSaveScreenshot->setObjectName(QStringLiteral("actionSaveScreenshot"));
        actionSaveScreenshot->setEnabled(false);
        actionTracking = new QAction(SLStudio);
        actionTracking->setObjectName(QStringLiteral("actionTracking"));
        actionTracking->setEnabled(false);
        actionAbout = new QAction(SLStudio);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout->setMenuRole(QAction::AboutRole);
        actionTracker = new QAction(SLStudio);
        actionTracker->setObjectName(QStringLiteral("actionTracker"));
        actionTracker->setCheckable(true);
        actionTracker->setChecked(true);
        actionUpdatePointClouds = new QAction(SLStudio);
        actionUpdatePointClouds->setObjectName(QStringLiteral("actionUpdatePointClouds"));
        actionUpdatePointClouds->setCheckable(true);
        actionUpdatePointClouds->setChecked(true);
        centralWidget = new QWidget(SLStudio);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setAcceptDrops(false);
        centralWidget->setLayoutDirection(Qt::LeftToRight);
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        pointCloudWidget = new SLPointCloudWidget(centralWidget);
        pointCloudWidget->setObjectName(QStringLiteral("pointCloudWidget"));
        pointCloudWidget->setEnabled(true);
        pointCloudWidget->setLayoutDirection(Qt::LeftToRight);
        pointCloudWidget->setAutoFillBackground(false);
        pushButton = new QPushButton(pointCloudWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(0, 420, 89, 25));
        pattern_num_spin = new QSpinBox(pointCloudWidget);
        pattern_num_spin->setObjectName(QStringLiteral("pattern_num_spin"));
        pattern_num_spin->setGeometry(QRect(90, 420, 48, 26));
        pattern_num_spin->setMaximum(1);
        frame_num_spin = new QSpinBox(pointCloudWidget);
        frame_num_spin->setObjectName(QStringLiteral("frame_num_spin"));
        frame_num_spin->setGeometry(QRect(140, 420, 48, 26));
        frame_num_spin->setMaximum(7);

        gridLayout->addWidget(pointCloudWidget, 0, 1, 1, 1);

        SLStudio->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SLStudio);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 828, 22));
        menuCalibration = new QMenu(menuBar);
        menuCalibration->setObjectName(QStringLiteral("menuCalibration"));
        menuSLStudio = new QMenu(menuBar);
        menuSLStudio->setObjectName(QStringLiteral("menuSLStudio"));
        menuScan = new QMenu(menuBar);
        menuScan->setObjectName(QStringLiteral("menuScan"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QStringLiteral("menuView"));
        SLStudio->setMenuBar(menuBar);
        toolBar = new QToolBar(SLStudio);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        toolBar->setEnabled(true);
        QFont font;
        font.setItalic(false);
        font.setStrikeOut(false);
        toolBar->setFont(font);
        toolBar->setMovable(false);
        toolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        SLStudio->addToolBar(Qt::TopToolBarArea, toolBar);
        statusBar = new QStatusBar(SLStudio);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        SLStudio->setStatusBar(statusBar);

        menuBar->addAction(menuSLStudio->menuAction());
        menuBar->addAction(menuScan->menuAction());
        menuBar->addAction(menuCalibration->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuCalibration->addAction(actionLoadCalibration);
        menuCalibration->addAction(actionExportCalibration);
        menuCalibration->addAction(actionCalibration);
        menuSLStudio->addAction(actionPreferences);
        menuSLStudio->addAction(actionQuit);
        menuSLStudio->addAction(actionAbout);
        menuScan->addAction(actionStart);
        menuScan->addAction(actionStop);
        menuScan->addAction(actionSavePointCloud);
        menuScan->addAction(actionSaveScreenshot);
        menuView->addAction(actionUpdatePointClouds);
        toolBar->addAction(actionStart);
        toolBar->addAction(actionStop);
        toolBar->addSeparator();
        toolBar->addAction(actionSavePointCloud);
        toolBar->addAction(actionSaveScreenshot);
        toolBar->addAction(actionCalibration);
        toolBar->addSeparator();

        retranslateUi(SLStudio);
        QObject::connect(actionStart, SIGNAL(triggered()), SLStudio, SLOT(onActionStart()));
        QObject::connect(actionStop, SIGNAL(triggered()), SLStudio, SLOT(onActionStop()));
        QObject::connect(actionQuit, SIGNAL(triggered()), SLStudio, SLOT(close()));
        QObject::connect(actionCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionCalibration()));
        QObject::connect(actionPreferences, SIGNAL(triggered()), SLStudio, SLOT(onActionPreferences()));
        QObject::connect(actionExportCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionExportCalibration()));
        QObject::connect(actionSaveScreenshot, SIGNAL(triggered()), pointCloudWidget, SLOT(saveScreenShot()));
        QObject::connect(actionSavePointCloud, SIGNAL(triggered()), pointCloudWidget, SLOT(savePointCloud()));
        QObject::connect(actionLoadCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionLoadCalibration()));
        QObject::connect(actionAbout, SIGNAL(triggered()), SLStudio, SLOT(onActionAbout()));

        QMetaObject::connectSlotsByName(SLStudio);
    } // setupUi

    void retranslateUi(QMainWindow *SLStudio)
    {
        SLStudio->setWindowTitle(QApplication::translate("SLStudio", "SLStudio", Q_NULLPTR));
        actionQuit->setText(QApplication::translate("SLStudio", "&Quit", Q_NULLPTR));
        actionStart->setText(QApplication::translate("SLStudio", "Start Scan", Q_NULLPTR));
        actionStop->setText(QApplication::translate("SLStudio", "Stop Scan", Q_NULLPTR));
        actionCalibration->setText(QApplication::translate("SLStudio", "Perform Calibration", Q_NULLPTR));
        actionLoadCalibration->setText(QApplication::translate("SLStudio", "Load Calibration", Q_NULLPTR));
        actionExportCalibration->setText(QApplication::translate("SLStudio", "Export Calibration", Q_NULLPTR));
        actionPreferences->setText(QApplication::translate("SLStudio", "&Preferences", Q_NULLPTR));
        actionSavePointCloud->setText(QApplication::translate("SLStudio", "Save Point Cloud", Q_NULLPTR));
        actionSaveScreenshot->setText(QApplication::translate("SLStudio", "Save Screenshot", Q_NULLPTR));
        actionTracking->setText(QApplication::translate("SLStudio", "Start Tracking", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("SLStudio", "About", Q_NULLPTR));
        actionTracker->setText(QApplication::translate("SLStudio", "Tracker", Q_NULLPTR));
        actionUpdatePointClouds->setText(QApplication::translate("SLStudio", "Update Point Clouds", Q_NULLPTR));
        pushButton->setText(QApplication::translate("SLStudio", "Grab Image", Q_NULLPTR));
        menuCalibration->setTitle(QApplication::translate("SLStudio", "Calibration", Q_NULLPTR));
        menuSLStudio->setTitle(QApplication::translate("SLStudio", "SLStudio", Q_NULLPTR));
        menuScan->setTitle(QApplication::translate("SLStudio", "Scan", Q_NULLPTR));
        menuView->setTitle(QApplication::translate("SLStudio", "View", Q_NULLPTR));
        toolBar->setWindowTitle(QApplication::translate("SLStudio", "toolBar", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SLStudio: public Ui_SLStudio {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLSTUDIO_H
