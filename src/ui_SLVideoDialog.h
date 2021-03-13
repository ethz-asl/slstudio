/********************************************************************************
** Form generated from reading UI file 'SLVideoDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLVIDEODIALOG_H
#define UI_SLVIDEODIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include "SLVideoWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLVideoDialog
{
public:
    QGridLayout *gridLayout;
    SLVideoWidget *videoWidget;

    void setupUi(QDialog *SLVideoDialog)
    {
        if (SLVideoDialog->objectName().isEmpty())
            SLVideoDialog->setObjectName(QStringLiteral("SLVideoDialog"));
        SLVideoDialog->resize(310, 238);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SLVideoDialog->sizePolicy().hasHeightForWidth());
        SLVideoDialog->setSizePolicy(sizePolicy);
        SLVideoDialog->setContextMenuPolicy(Qt::DefaultContextMenu);
        SLVideoDialog->setAutoFillBackground(false);
        SLVideoDialog->setSizeGripEnabled(true);
        SLVideoDialog->setModal(false);
        gridLayout = new QGridLayout(SLVideoDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        videoWidget = new SLVideoWidget(SLVideoDialog);
        videoWidget->setObjectName(QStringLiteral("videoWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(videoWidget->sizePolicy().hasHeightForWidth());
        videoWidget->setSizePolicy(sizePolicy1);
        videoWidget->setMinimumSize(QSize(100, 100));
        videoWidget->setAutoFillBackground(false);
        videoWidget->setFrameShape(QFrame::NoFrame);
        videoWidget->setFrameShadow(QFrame::Plain);
        videoWidget->setScaledContents(false);
        videoWidget->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(videoWidget, 0, 0, 1, 1);


        retranslateUi(SLVideoDialog);

        QMetaObject::connectSlotsByName(SLVideoDialog);
    } // setupUi

    void retranslateUi(QDialog *SLVideoDialog)
    {
        SLVideoDialog->setWindowTitle(QApplication::translate("SLVideoDialog", "SLVideoDialog", Q_NULLPTR));
        videoWidget->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class SLVideoDialog: public Ui_SLVideoDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLVIDEODIALOG_H
