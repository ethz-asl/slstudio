/****************************************************************************
** Meta object code from reading C++ file 'SLScanWorker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLScanWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLScanWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLScanWorker_t {
    QByteArrayData data[16];
    char stringdata0[140];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLScanWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLScanWorker_t qt_meta_stringdata_SLScanWorker = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SLScanWorker"
QT_MOC_LITERAL(1, 13, 13), // "showHistogram"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 7), // "cv::Mat"
QT_MOC_LITERAL(4, 36, 2), // "im"
QT_MOC_LITERAL(5, 39, 8), // "newFrame"
QT_MOC_LITERAL(6, 48, 5), // "frame"
QT_MOC_LITERAL(7, 54, 11), // "newFrameSeq"
QT_MOC_LITERAL(8, 66, 20), // "std::vector<cv::Mat>"
QT_MOC_LITERAL(9, 87, 8), // "frameSeq"
QT_MOC_LITERAL(10, 96, 5), // "error"
QT_MOC_LITERAL(11, 102, 3), // "err"
QT_MOC_LITERAL(12, 106, 8), // "finished"
QT_MOC_LITERAL(13, 115, 5), // "setup"
QT_MOC_LITERAL(14, 121, 6), // "doWork"
QT_MOC_LITERAL(15, 128, 11) // "stopWorking"

    },
    "SLScanWorker\0showHistogram\0\0cv::Mat\0"
    "im\0newFrame\0frame\0newFrameSeq\0"
    "std::vector<cv::Mat>\0frameSeq\0error\0"
    "err\0finished\0setup\0doWork\0stopWorking"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLScanWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       5,    1,   57,    2, 0x06 /* Public */,
       7,    1,   60,    2, 0x06 /* Public */,
      10,    1,   63,    2, 0x06 /* Public */,
      12,    0,   66,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    0,   67,    2, 0x0a /* Public */,
      14,    0,   68,    2, 0x0a /* Public */,
      15,    0,   69,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, QMetaType::QString,   11,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SLScanWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLScanWorker *_t = static_cast<SLScanWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->showHistogram((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 1: _t->newFrame((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->newFrameSeq((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        case 3: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->finished(); break;
        case 5: _t->setup(); break;
        case 6: _t->doWork(); break;
        case 7: _t->stopWorking(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLScanWorker::*_t)(cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLScanWorker::showHistogram)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SLScanWorker::*_t)(cv::Mat );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLScanWorker::newFrame)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (SLScanWorker::*_t)(std::vector<cv::Mat> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLScanWorker::newFrameSeq)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (SLScanWorker::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLScanWorker::error)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (SLScanWorker::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLScanWorker::finished)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject SLScanWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLScanWorker.data,
      qt_meta_data_SLScanWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLScanWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLScanWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLScanWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SLScanWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void SLScanWorker::showHistogram(cv::Mat _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLScanWorker::newFrame(cv::Mat _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLScanWorker::newFrameSeq(std::vector<cv::Mat> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SLScanWorker::error(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SLScanWorker::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
