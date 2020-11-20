/********************************************************************************
** Form generated from reading UI file 'SLCalibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLCALIBRATIONDIALOG_H
#define UI_SLCALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>
#include "SLVideoWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLCalibrationDialog
{
public:
    SLVideoWidget *videoWidget;
    QPushButton *snapButton;
    QListWidget *listWidget;
    QPushButton *calibrateButton;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *checkerSizeBox;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QSpinBox *checkerRowsBox;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QSpinBox *checkerColsBox;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *cancelButton;
    QPushButton *saveButton;
    QPushButton *pushButton;

    void setupUi(QDialog *SLCalibrationDialog)
    {
        if (SLCalibrationDialog->objectName().isEmpty())
            SLCalibrationDialog->setObjectName(QStringLiteral("SLCalibrationDialog"));
        SLCalibrationDialog->resize(890, 540);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SLCalibrationDialog->sizePolicy().hasHeightForWidth());
        SLCalibrationDialog->setSizePolicy(sizePolicy);
        SLCalibrationDialog->setMinimumSize(QSize(890, 540));
        SLCalibrationDialog->setMaximumSize(QSize(890, 540));
        SLCalibrationDialog->setSizeGripEnabled(false);
        videoWidget = new SLVideoWidget(SLCalibrationDialog);
        videoWidget->setObjectName(QStringLiteral("videoWidget"));
        videoWidget->setEnabled(true);
        videoWidget->setGeometry(QRect(10, 10, 651, 471));
        snapButton = new QPushButton(SLCalibrationDialog);
        snapButton->setObjectName(QStringLiteral("snapButton"));
        snapButton->setGeometry(QRect(205, 500, 241, 27));
        listWidget = new QListWidget(SLCalibrationDialog);
        listWidget->setObjectName(QStringLiteral("listWidget"));
        listWidget->setGeometry(QRect(680, 10, 181, 281));
        calibrateButton = new QPushButton(SLCalibrationDialog);
        calibrateButton->setObjectName(QStringLiteral("calibrateButton"));
        calibrateButton->setEnabled(false);
        calibrateButton->setGeometry(QRect(718, 430, 111, 27));
        layoutWidget = new QWidget(SLCalibrationDialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(680, 330, 181, 29));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        checkerSizeBox = new QSpinBox(layoutWidget);
        checkerSizeBox->setObjectName(QStringLiteral("checkerSizeBox"));

        horizontalLayout->addWidget(checkerSizeBox);

        layoutWidget1 = new QWidget(SLCalibrationDialog);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(680, 360, 181, 29));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget1);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_2->addWidget(label_3);

        checkerRowsBox = new QSpinBox(layoutWidget1);
        checkerRowsBox->setObjectName(QStringLiteral("checkerRowsBox"));

        horizontalLayout_2->addWidget(checkerRowsBox);

        layoutWidget2 = new QWidget(SLCalibrationDialog);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(680, 390, 181, 29));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget2);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_3->addWidget(label_4);

        checkerColsBox = new QSpinBox(layoutWidget2);
        checkerColsBox->setObjectName(QStringLiteral("checkerColsBox"));

        horizontalLayout_3->addWidget(checkerColsBox);

        layoutWidget3 = new QWidget(SLCalibrationDialog);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(690, 480, 181, 29));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget3);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        cancelButton = new QPushButton(layoutWidget3);
        cancelButton->setObjectName(QStringLiteral("cancelButton"));

        horizontalLayout_4->addWidget(cancelButton);

        saveButton = new QPushButton(layoutWidget3);
        saveButton->setObjectName(QStringLiteral("saveButton"));
        saveButton->setEnabled(false);

        horizontalLayout_4->addWidget(saveButton);

        pushButton = new QPushButton(SLCalibrationDialog);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(690, 510, 89, 25));

        retranslateUi(SLCalibrationDialog);
        QObject::connect(cancelButton, SIGNAL(clicked()), SLCalibrationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(SLCalibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *SLCalibrationDialog)
    {
        SLCalibrationDialog->setWindowTitle(QApplication::translate("SLCalibrationDialog", "SL Calibration", Q_NULLPTR));
        snapButton->setText(QApplication::translate("SLCalibrationDialog", "Snap", Q_NULLPTR));
        calibrateButton->setText(QApplication::translate("SLCalibrationDialog", "Calibrate", Q_NULLPTR));
        label->setText(QApplication::translate("SLCalibrationDialog", "Size (mm):", Q_NULLPTR));
        label_3->setText(QApplication::translate("SLCalibrationDialog", "Rows:", Q_NULLPTR));
        label_4->setText(QApplication::translate("SLCalibrationDialog", "Cols:", Q_NULLPTR));
        cancelButton->setText(QApplication::translate("SLCalibrationDialog", "&Cancel", Q_NULLPTR));
        saveButton->setText(QApplication::translate("SLCalibrationDialog", "&Save", Q_NULLPTR));
        pushButton->setText(QApplication::translate("SLCalibrationDialog", "Re-Calibrate", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SLCalibrationDialog: public Ui_SLCalibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLCALIBRATIONDIALOG_H
