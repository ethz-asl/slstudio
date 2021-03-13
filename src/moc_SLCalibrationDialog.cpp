/****************************************************************************
** Meta object code from reading C++ file 'SLCalibrationDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLCalibrationDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLCalibrationDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLCalibrationDialog_t {
    QByteArrayData data[15];
    char stringdata0[236];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLCalibrationDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLCalibrationDialog_t qt_meta_stringdata_SLCalibrationDialog = {
    {
QT_MOC_LITERAL(0, 0, 19), // "SLCalibrationDialog"
QT_MOC_LITERAL(1, 20, 19), // "newCalibrationSaved"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 15), // "CalibrationData"
QT_MOC_LITERAL(4, 57, 6), // "_calib"
QT_MOC_LITERAL(5, 64, 21), // "on_snapButton_clicked"
QT_MOC_LITERAL(6, 86, 26), // "on_calibrateButton_clicked"
QT_MOC_LITERAL(7, 113, 34), // "on_listWidget_itemSelectionCh..."
QT_MOC_LITERAL(8, 148, 21), // "on_saveButton_clicked"
QT_MOC_LITERAL(9, 170, 19), // "onNewSequenceResult"
QT_MOC_LITERAL(10, 190, 7), // "cv::Mat"
QT_MOC_LITERAL(11, 198, 3), // "img"
QT_MOC_LITERAL(12, 202, 3), // "idx"
QT_MOC_LITERAL(13, 206, 7), // "success"
QT_MOC_LITERAL(14, 214, 21) // "on_pushButton_clicked"

    },
    "SLCalibrationDialog\0newCalibrationSaved\0"
    "\0CalibrationData\0_calib\0on_snapButton_clicked\0"
    "on_calibrateButton_clicked\0"
    "on_listWidget_itemSelectionChanged\0"
    "on_saveButton_clicked\0onNewSequenceResult\0"
    "cv::Mat\0img\0idx\0success\0on_pushButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLCalibrationDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   52,    2, 0x08 /* Private */,
       6,    0,   53,    2, 0x08 /* Private */,
       7,    0,   54,    2, 0x08 /* Private */,
       8,    0,   55,    2, 0x08 /* Private */,
       9,    3,   56,    2, 0x08 /* Private */,
      14,    0,   63,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10, QMetaType::UInt, QMetaType::Bool,   11,   12,   13,
    QMetaType::Void,

       0        // eod
};

void SLCalibrationDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLCalibrationDialog *_t = static_cast<SLCalibrationDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newCalibrationSaved((*reinterpret_cast< CalibrationData(*)>(_a[1]))); break;
        case 1: _t->on_snapButton_clicked(); break;
        case 2: _t->on_calibrateButton_clicked(); break;
        case 3: _t->on_listWidget_itemSelectionChanged(); break;
        case 4: _t->on_saveButton_clicked(); break;
        case 5: _t->onNewSequenceResult((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 6: _t->on_pushButton_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLCalibrationDialog::*_t)(CalibrationData );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLCalibrationDialog::newCalibrationSaved)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject SLCalibrationDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLCalibrationDialog.data,
      qt_meta_data_SLCalibrationDialog,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLCalibrationDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLCalibrationDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLCalibrationDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int SLCalibrationDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void SLCalibrationDialog::newCalibrationSaved(CalibrationData _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
