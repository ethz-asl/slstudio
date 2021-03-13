/****************************************************************************
** Meta object code from reading C++ file 'SLDecoderWorker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLDecoderWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLDecoderWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLDecoderWorker_t {
    QByteArrayData data[23];
    char stringdata0[191];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLDecoderWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLDecoderWorker_t qt_meta_stringdata_SLDecoderWorker = {
    {
QT_MOC_LITERAL(0, 0, 15), // "SLDecoderWorker"
QT_MOC_LITERAL(1, 16, 6), // "imshow"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 11), // "const char*"
QT_MOC_LITERAL(4, 36, 10), // "windowName"
QT_MOC_LITERAL(5, 47, 7), // "cv::Mat"
QT_MOC_LITERAL(6, 55, 3), // "mat"
QT_MOC_LITERAL(7, 59, 1), // "x"
QT_MOC_LITERAL(8, 61, 1), // "y"
QT_MOC_LITERAL(9, 63, 11), // "showShading"
QT_MOC_LITERAL(10, 75, 13), // "showDecoderUp"
QT_MOC_LITERAL(11, 89, 13), // "showDecoderVp"
QT_MOC_LITERAL(12, 103, 7), // "newUpVp"
QT_MOC_LITERAL(13, 111, 2), // "up"
QT_MOC_LITERAL(14, 114, 2), // "vp"
QT_MOC_LITERAL(15, 117, 4), // "mask"
QT_MOC_LITERAL(16, 122, 7), // "shading"
QT_MOC_LITERAL(17, 130, 5), // "error"
QT_MOC_LITERAL(18, 136, 3), // "err"
QT_MOC_LITERAL(19, 140, 5), // "setup"
QT_MOC_LITERAL(20, 146, 14), // "decodeSequence"
QT_MOC_LITERAL(21, 161, 20), // "std::vector<cv::Mat>"
QT_MOC_LITERAL(22, 182, 8) // "frameSeq"

    },
    "SLDecoderWorker\0imshow\0\0const char*\0"
    "windowName\0cv::Mat\0mat\0x\0y\0showShading\0"
    "showDecoderUp\0showDecoderVp\0newUpVp\0"
    "up\0vp\0mask\0shading\0error\0err\0setup\0"
    "decodeSequence\0std::vector<cv::Mat>\0"
    "frameSeq"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLDecoderWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   54,    2, 0x06 /* Public */,
       9,    1,   63,    2, 0x06 /* Public */,
      10,    1,   66,    2, 0x06 /* Public */,
      11,    1,   69,    2, 0x06 /* Public */,
      12,    4,   72,    2, 0x06 /* Public */,
      17,    1,   81,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      19,    0,   84,    2, 0x0a /* Public */,
      20,    1,   85,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, QMetaType::UInt, QMetaType::UInt,    4,    6,    7,    8,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5,   13,   14,   15,   16,
    QMetaType::Void, QMetaType::QString,   18,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 21,   22,

       0        // eod
};

void SLDecoderWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLDecoderWorker *_t = static_cast<SLDecoderWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 1: _t->showShading((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->showDecoderUp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 3: _t->showDecoderVp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 4: _t->newUpVp((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        case 5: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->setup(); break;
        case 7: _t->decodeSequence((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLDecoderWorker::*_t)(const char * , cv::Mat , unsigned int , unsigned int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::imshow)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SLDecoderWorker::*_t)(cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::showShading)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (SLDecoderWorker::*_t)(cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::showDecoderUp)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (SLDecoderWorker::*_t)(cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::showDecoderVp)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (SLDecoderWorker::*_t)(cv::Mat , cv::Mat , cv::Mat , cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::newUpVp)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (SLDecoderWorker::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLDecoderWorker::error)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject SLDecoderWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLDecoderWorker.data,
      qt_meta_data_SLDecoderWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLDecoderWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLDecoderWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLDecoderWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SLDecoderWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void SLDecoderWorker::imshow(const char * _t1, cv::Mat _t2, unsigned int _t3, unsigned int _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLDecoderWorker::showShading(cv::Mat _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLDecoderWorker::showDecoderUp(cv::Mat _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SLDecoderWorker::showDecoderVp(cv::Mat _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SLDecoderWorker::newUpVp(cv::Mat _t1, cv::Mat _t2, cv::Mat _t3, cv::Mat _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void SLDecoderWorker::error(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
