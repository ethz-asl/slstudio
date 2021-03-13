/****************************************************************************
** Meta object code from reading C++ file 'Calibrator.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "calibrator/Calibrator.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Calibrator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Calibrator_t {
    QByteArrayData data[7];
    char stringdata0[54];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Calibrator_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Calibrator_t qt_meta_stringdata_Calibrator = {
    {
QT_MOC_LITERAL(0, 0, 10), // "Calibrator"
QT_MOC_LITERAL(1, 11, 17), // "newSequenceResult"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7), // "cv::Mat"
QT_MOC_LITERAL(4, 38, 3), // "img"
QT_MOC_LITERAL(5, 42, 3), // "idx"
QT_MOC_LITERAL(6, 46, 7) // "success"

    },
    "Calibrator\0newSequenceResult\0\0cv::Mat\0"
    "img\0idx\0success"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Calibrator[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::UInt, QMetaType::Bool,    4,    5,    6,

       0        // eod
};

void Calibrator::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Calibrator *_t = static_cast<Calibrator *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newSequenceResult((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (Calibrator::*_t)(cv::Mat , unsigned int , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Calibrator::newSequenceResult)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject Calibrator::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Calibrator.data,
      qt_meta_data_Calibrator,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Calibrator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Calibrator::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Calibrator.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Calibrator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void Calibrator::newSequenceResult(cv::Mat _t1, unsigned int _t2, bool _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
