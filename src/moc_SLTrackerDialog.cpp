/****************************************************************************
** Meta object code from reading C++ file 'SLTrackerDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLTrackerDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTrackerDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLTrackerDialog_t {
    QByteArrayData data[10];
    char stringdata0[148];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLTrackerDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLTrackerDialog_t qt_meta_stringdata_SLTrackerDialog = {
    {
QT_MOC_LITERAL(0, 0, 15), // "SLTrackerDialog"
QT_MOC_LITERAL(1, 16, 13), // "newPointCloud"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 18), // "PointCloudConstPtr"
QT_MOC_LITERAL(4, 50, 10), // "pointCloud"
QT_MOC_LITERAL(5, 61, 20), // "receiveNewPointCloud"
QT_MOC_LITERAL(6, 82, 16), // "showPoseEstimate"
QT_MOC_LITERAL(7, 99, 15), // "Eigen::Affine3f"
QT_MOC_LITERAL(8, 115, 1), // "T"
QT_MOC_LITERAL(9, 117, 30) // "on_startStopPushButton_clicked"

    },
    "SLTrackerDialog\0newPointCloud\0\0"
    "PointCloudConstPtr\0pointCloud\0"
    "receiveNewPointCloud\0showPoseEstimate\0"
    "Eigen::Affine3f\0T\0on_startStopPushButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLTrackerDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   37,    2, 0x0a /* Public */,
       6,    1,   40,    2, 0x0a /* Public */,
       9,    0,   43,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void,

       0        // eod
};

void SLTrackerDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLTrackerDialog *_t = static_cast<SLTrackerDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 1: _t->receiveNewPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->showPoseEstimate((*reinterpret_cast< Eigen::Affine3f(*)>(_a[1]))); break;
        case 3: _t->on_startStopPushButton_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLTrackerDialog::*_t)(PointCloudConstPtr );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTrackerDialog::newPointCloud)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject SLTrackerDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLTrackerDialog.data,
      qt_meta_data_SLTrackerDialog,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLTrackerDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLTrackerDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLTrackerDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int SLTrackerDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SLTrackerDialog::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
