/********************************************************************************
** Form generated from reading UI file 'SLTrackerDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLTRACKERDIALOG_H
#define UI_SLTRACKERDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include "SLPoseWidget.h"
#include "SLTraceWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLTrackerDialog
{
public:
    QAction *actionStart_Tracking;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *poseTab;
    QGridLayout *gridLayout_2;
    SLPoseWidget *poseWidget;
    QWidget *traceTab;
    QGridLayout *gridLayout_3;
    SLTraceWidget *translationTraceWidget;
    SLTraceWidget *rotationTraceWidget;
    QPushButton *startStopPushButton;

    void setupUi(QWidget *SLTrackerDialog)
    {
        if (SLTrackerDialog->objectName().isEmpty())
            SLTrackerDialog->setObjectName(QStringLiteral("SLTrackerDialog"));
        SLTrackerDialog->resize(742, 567);
        actionStart_Tracking = new QAction(SLTrackerDialog);
        actionStart_Tracking->setObjectName(QStringLiteral("actionStart_Tracking"));
        gridLayout = new QGridLayout(SLTrackerDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        tabWidget = new QTabWidget(SLTrackerDialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        poseTab = new QWidget();
        poseTab->setObjectName(QStringLiteral("poseTab"));
        gridLayout_2 = new QGridLayout(poseTab);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        poseWidget = new SLPoseWidget(poseTab);
        poseWidget->setObjectName(QStringLiteral("poseWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(poseWidget->sizePolicy().hasHeightForWidth());
        poseWidget->setSizePolicy(sizePolicy);
        poseWidget->setAutoFillBackground(true);

        gridLayout_2->addWidget(poseWidget, 0, 0, 1, 1);

        tabWidget->addTab(poseTab, QString());
        traceTab = new QWidget();
        traceTab->setObjectName(QStringLiteral("traceTab"));
        gridLayout_3 = new QGridLayout(traceTab);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        translationTraceWidget = new SLTraceWidget(traceTab);
        translationTraceWidget->setObjectName(QStringLiteral("translationTraceWidget"));
        sizePolicy.setHeightForWidth(translationTraceWidget->sizePolicy().hasHeightForWidth());
        translationTraceWidget->setSizePolicy(sizePolicy);
        translationTraceWidget->setScaledContents(true);

        gridLayout_3->addWidget(translationTraceWidget, 0, 0, 1, 1);

        rotationTraceWidget = new SLTraceWidget(traceTab);
        rotationTraceWidget->setObjectName(QStringLiteral("rotationTraceWidget"));
        sizePolicy.setHeightForWidth(rotationTraceWidget->sizePolicy().hasHeightForWidth());
        rotationTraceWidget->setSizePolicy(sizePolicy);
        rotationTraceWidget->setScaledContents(true);

        gridLayout_3->addWidget(rotationTraceWidget, 1, 0, 1, 1);

        tabWidget->addTab(traceTab, QString());

        gridLayout->addWidget(tabWidget, 1, 0, 1, 1);

        startStopPushButton = new QPushButton(SLTrackerDialog);
        startStopPushButton->setObjectName(QStringLiteral("startStopPushButton"));

        gridLayout->addWidget(startStopPushButton, 2, 0, 1, 1);


        retranslateUi(SLTrackerDialog);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(SLTrackerDialog);
    } // setupUi

    void retranslateUi(QWidget *SLTrackerDialog)
    {
        SLTrackerDialog->setWindowTitle(QApplication::translate("SLTrackerDialog", "Tracker", Q_NULLPTR));
        actionStart_Tracking->setText(QApplication::translate("SLTrackerDialog", "Start Tracking", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(poseTab), QApplication::translate("SLTrackerDialog", "Pose", Q_NULLPTR));
        translationTraceWidget->setText(QString());
        rotationTraceWidget->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(traceTab), QApplication::translate("SLTrackerDialog", "Trace", Q_NULLPTR));
        startStopPushButton->setText(QApplication::translate("SLTrackerDialog", "Start Tracking", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SLTrackerDialog: public Ui_SLTrackerDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLTRACKERDIALOG_H
