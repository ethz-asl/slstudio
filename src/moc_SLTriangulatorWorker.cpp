/****************************************************************************
** Meta object code from reading C++ file 'SLTriangulatorWorker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLTriangulatorWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTriangulatorWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLTriangulatorWorker_t {
    QByteArrayData data[20];
    char stringdata0[169];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLTriangulatorWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLTriangulatorWorker_t qt_meta_stringdata_SLTriangulatorWorker = {
    {
QT_MOC_LITERAL(0, 0, 20), // "SLTriangulatorWorker"
QT_MOC_LITERAL(1, 21, 6), // "imshow"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 11), // "const char*"
QT_MOC_LITERAL(4, 41, 10), // "windowName"
QT_MOC_LITERAL(5, 52, 7), // "cv::Mat"
QT_MOC_LITERAL(6, 60, 3), // "mat"
QT_MOC_LITERAL(7, 64, 1), // "x"
QT_MOC_LITERAL(8, 66, 1), // "y"
QT_MOC_LITERAL(9, 68, 13), // "newPointCloud"
QT_MOC_LITERAL(10, 82, 18), // "PointCloudConstPtr"
QT_MOC_LITERAL(11, 101, 10), // "pointCloud"
QT_MOC_LITERAL(12, 112, 5), // "error"
QT_MOC_LITERAL(13, 118, 3), // "err"
QT_MOC_LITERAL(14, 122, 5), // "setup"
QT_MOC_LITERAL(15, 128, 21), // "triangulatePointCloud"
QT_MOC_LITERAL(16, 150, 2), // "up"
QT_MOC_LITERAL(17, 153, 2), // "vp"
QT_MOC_LITERAL(18, 156, 4), // "mask"
QT_MOC_LITERAL(19, 161, 7) // "shading"

    },
    "SLTriangulatorWorker\0imshow\0\0const char*\0"
    "windowName\0cv::Mat\0mat\0x\0y\0newPointCloud\0"
    "PointCloudConstPtr\0pointCloud\0error\0"
    "err\0setup\0triangulatePointCloud\0up\0"
    "vp\0mask\0shading"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLTriangulatorWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   39,    2, 0x06 /* Public */,
       9,    1,   48,    2, 0x06 /* Public */,
      12,    1,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    0,   54,    2, 0x0a /* Public */,
      15,    4,   55,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, QMetaType::UInt, QMetaType::UInt,    4,    6,    7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::QString,   13,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5,   16,   17,   18,   19,

       0        // eod
};

void SLTriangulatorWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLTriangulatorWorker *_t = static_cast<SLTriangulatorWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 1: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->setup(); break;
        case 4: _t->triangulatePointCloud((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLTriangulatorWorker::*_t)(const char * , cv::Mat , unsigned int , unsigned int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTriangulatorWorker::imshow)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SLTriangulatorWorker::*_t)(PointCloudConstPtr );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTriangulatorWorker::newPointCloud)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (SLTriangulatorWorker::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLTriangulatorWorker::error)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject SLTriangulatorWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLTriangulatorWorker.data,
      qt_meta_data_SLTriangulatorWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLTriangulatorWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLTriangulatorWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLTriangulatorWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SLTriangulatorWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void SLTriangulatorWorker::imshow(const char * _t1, cv::Mat _t2, unsigned int _t3, unsigned int _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLTriangulatorWorker::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLTriangulatorWorker::error(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
