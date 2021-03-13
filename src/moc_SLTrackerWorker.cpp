/****************************************************************************
** Meta object code from reading C++ file 'SLTrackerWorker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLTrackerWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTrackerWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLTrackerWorker_t {
    QByteArrayData data[13];
    char stringdata0[146];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLTrackerWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLTrackerWorker_t qt_meta_stringdata_SLTrackerWorker = {
    {
QT_MOC_LITERAL(0, 0, 15), // "SLTrackerWorker"
QT_MOC_LITERAL(1, 16, 15), // "newPoseEstimate"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 15), // "Eigen::Affine3f"
QT_MOC_LITERAL(4, 49, 1), // "T"
QT_MOC_LITERAL(5, 51, 5), // "error"
QT_MOC_LITERAL(6, 57, 3), // "err"
QT_MOC_LITERAL(7, 61, 5), // "setup"
QT_MOC_LITERAL(8, 67, 15), // "trackPointCloud"
QT_MOC_LITERAL(9, 83, 18), // "PointCloudConstPtr"
QT_MOC_LITERAL(10, 102, 10), // "pointCloud"
QT_MOC_LITERAL(11, 113, 12), // "setReference"
QT_MOC_LITERAL(12, 126, 19) // "referencePointCloud"

    },
    "SLTrackerWorker\0newPoseEstimate\0\0"
    "Eigen::Affine3f\0T\0error\0err\0setup\0"
    "trackPointCloud\0PointCloudConstPtr\0"
    "pointCloud\0setReference\0referencePointCloud"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLTrackerWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   45,    2, 0x0a /* Public */,
       8,    1,   46,    2, 0x0a /* Public */,
      11,    1,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 9,   12,

       0        // eod
};

void SLTrackerWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLTrackerWorker *_t = static_cast<SLTrackerWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newPoseEstimate((*reinterpret_cast< Eigen::Affine3f(*)>(_a[1]))); break;
        case 1: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->setup(); break;
        case 3: _t->trackPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 4: _t->setReference((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLTrackerWorker::*_t)(Eigen::Affine3f );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTrackerWorker::newPoseEstimate)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SLTrackerWorker::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTrackerWorker::error)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject SLTrackerWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLTrackerWorker.data,
      qt_meta_data_SLTrackerWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLTrackerWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLTrackerWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLTrackerWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SLTrackerWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void SLTrackerWorker::newPoseEstimate(Eigen::Affine3f _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLTrackerWorker::error(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
