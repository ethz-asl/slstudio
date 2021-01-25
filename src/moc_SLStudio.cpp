/****************************************************************************
** Meta object code from reading C++ file 'SLStudio.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLStudio.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLStudio.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLStudio_t {
    QByteArrayData data[33];
    char stringdata0[476];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLStudio_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLStudio_t qt_meta_stringdata_SLStudio = {
    {
QT_MOC_LITERAL(0, 0, 8), // "SLStudio"
QT_MOC_LITERAL(1, 9, 13), // "newPointCloud"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 18), // "PointCloudConstPtr"
QT_MOC_LITERAL(4, 43, 10), // "pointCloud"
QT_MOC_LITERAL(5, 54, 13), // "onActionStart"
QT_MOC_LITERAL(6, 68, 12), // "onActionStop"
QT_MOC_LITERAL(7, 81, 20), // "onScanWorkerFinished"
QT_MOC_LITERAL(8, 102, 19), // "onActionCalibration"
QT_MOC_LITERAL(9, 122, 23), // "onActionLoadCalibration"
QT_MOC_LITERAL(10, 146, 19), // "onActionPreferences"
QT_MOC_LITERAL(11, 166, 25), // "onActionExportCalibration"
QT_MOC_LITERAL(12, 192, 17), // "updateDisplayRate"
QT_MOC_LITERAL(13, 210, 20), // "receiveNewPointCloud"
QT_MOC_LITERAL(14, 231, 6), // "imshow"
QT_MOC_LITERAL(15, 238, 11), // "const char*"
QT_MOC_LITERAL(16, 250, 10), // "windowName"
QT_MOC_LITERAL(17, 261, 7), // "cv::Mat"
QT_MOC_LITERAL(18, 269, 2), // "im"
QT_MOC_LITERAL(19, 272, 1), // "x"
QT_MOC_LITERAL(20, 274, 1), // "y"
QT_MOC_LITERAL(21, 276, 4), // "hist"
QT_MOC_LITERAL(22, 281, 15), // "onShowHistogram"
QT_MOC_LITERAL(23, 297, 13), // "onShowShading"
QT_MOC_LITERAL(24, 311, 18), // "onShowCameraFrames"
QT_MOC_LITERAL(25, 330, 20), // "std::vector<cv::Mat>"
QT_MOC_LITERAL(26, 351, 8), // "frameSeq"
QT_MOC_LITERAL(27, 360, 15), // "onShowDecoderUp"
QT_MOC_LITERAL(28, 376, 15), // "onShowDecoderVp"
QT_MOC_LITERAL(29, 392, 13), // "onActionAbout"
QT_MOC_LITERAL(30, 406, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(31, 428, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(32, 452, 23) // "on_pushButton_3_clicked"

    },
    "SLStudio\0newPointCloud\0\0PointCloudConstPtr\0"
    "pointCloud\0onActionStart\0onActionStop\0"
    "onScanWorkerFinished\0onActionCalibration\0"
    "onActionLoadCalibration\0onActionPreferences\0"
    "onActionExportCalibration\0updateDisplayRate\0"
    "receiveNewPointCloud\0imshow\0const char*\0"
    "windowName\0cv::Mat\0im\0x\0y\0hist\0"
    "onShowHistogram\0onShowShading\0"
    "onShowCameraFrames\0std::vector<cv::Mat>\0"
    "frameSeq\0onShowDecoderUp\0onShowDecoderVp\0"
    "onActionAbout\0on_pushButton_clicked\0"
    "on_pushButton_2_clicked\0on_pushButton_3_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLStudio[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  119,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,  122,    2, 0x08 /* Private */,
       6,    0,  123,    2, 0x08 /* Private */,
       7,    0,  124,    2, 0x08 /* Private */,
       8,    0,  125,    2, 0x08 /* Private */,
       9,    0,  126,    2, 0x08 /* Private */,
      10,    0,  127,    2, 0x08 /* Private */,
      11,    0,  128,    2, 0x08 /* Private */,
      12,    0,  129,    2, 0x08 /* Private */,
      13,    1,  130,    2, 0x08 /* Private */,
      14,    4,  133,    2, 0x08 /* Private */,
      21,    4,  142,    2, 0x08 /* Private */,
      22,    1,  151,    2, 0x08 /* Private */,
      23,    1,  154,    2, 0x08 /* Private */,
      24,    1,  157,    2, 0x08 /* Private */,
      27,    1,  160,    2, 0x08 /* Private */,
      28,    1,  163,    2, 0x08 /* Private */,
      29,    0,  166,    2, 0x08 /* Private */,
      30,    0,  167,    2, 0x08 /* Private */,
      31,    0,  168,    2, 0x08 /* Private */,
      32,    0,  169,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 17, QMetaType::UInt, QMetaType::UInt,   16,   18,   19,   20,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 17, QMetaType::UInt, QMetaType::UInt,   16,   18,   19,   20,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 25,   26,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SLStudio::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLStudio *_t = static_cast<SLStudio *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 1: _t->onActionStart(); break;
        case 2: _t->onActionStop(); break;
        case 3: _t->onScanWorkerFinished(); break;
        case 4: _t->onActionCalibration(); break;
        case 5: _t->onActionLoadCalibration(); break;
        case 6: _t->onActionPreferences(); break;
        case 7: _t->onActionExportCalibration(); break;
        case 8: _t->updateDisplayRate(); break;
        case 9: _t->receiveNewPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 10: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 11: _t->hist((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 12: _t->onShowHistogram((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 13: _t->onShowShading((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 14: _t->onShowCameraFrames((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        case 15: _t->onShowDecoderUp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 16: _t->onShowDecoderVp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 17: _t->onActionAbout(); break;
        case 18: _t->on_pushButton_clicked(); break;
        case 19: _t->on_pushButton_2_clicked(); break;
        case 20: _t->on_pushButton_3_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLStudio::*_t)(PointCloudConstPtr );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLStudio::newPointCloud)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject SLStudio::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_SLStudio.data,
      qt_meta_data_SLStudio,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLStudio::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLStudio::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLStudio.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int SLStudio::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void SLStudio::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
