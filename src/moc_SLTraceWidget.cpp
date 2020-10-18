/****************************************************************************
** Meta object code from reading C++ file 'SLTraceWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLTraceWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTraceWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLTraceWidget_t {
    QByteArrayData data[15];
    char stringdata0[125];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLTraceWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLTraceWidget_t qt_meta_stringdata_SLTraceWidget = {
    {
QT_MOC_LITERAL(0, 0, 13), // "SLTraceWidget"
QT_MOC_LITERAL(1, 14, 8), // "addTrace"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 4), // "name"
QT_MOC_LITERAL(4, 29, 14), // "addMeasurement"
QT_MOC_LITERAL(5, 44, 2), // "id"
QT_MOC_LITERAL(6, 47, 3), // "val"
QT_MOC_LITERAL(7, 51, 15), // "addMeasurements"
QT_MOC_LITERAL(8, 67, 18), // "std::vector<float>"
QT_MOC_LITERAL(9, 86, 4), // "vals"
QT_MOC_LITERAL(10, 91, 9), // "setBounds"
QT_MOC_LITERAL(11, 101, 6), // "_tSpan"
QT_MOC_LITERAL(12, 108, 5), // "_yMin"
QT_MOC_LITERAL(13, 114, 5), // "_yMax"
QT_MOC_LITERAL(14, 120, 4) // "draw"

    },
    "SLTraceWidget\0addTrace\0\0name\0"
    "addMeasurement\0id\0val\0addMeasurements\0"
    "std::vector<float>\0vals\0setBounds\0"
    "_tSpan\0_yMin\0_yMax\0draw"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLTraceWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x0a /* Public */,
       4,    2,   47,    2, 0x0a /* Public */,
       4,    2,   52,    2, 0x0a /* Public */,
       7,    1,   57,    2, 0x0a /* Public */,
      10,    3,   60,    2, 0x0a /* Public */,
      14,    0,   67,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::UInt, QMetaType::Float,    5,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::Float,    3,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,   11,   12,   13,
    QMetaType::Void,

       0        // eod
};

void SLTraceWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLTraceWidget *_t = static_cast<SLTraceWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->addTrace((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->addMeasurement((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->addMeasurement((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 3: _t->addMeasurements((*reinterpret_cast< std::vector<float>(*)>(_a[1]))); break;
        case 4: _t->setBounds((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 5: _t->draw(); break;
        default: ;
        }
    }
}

const QMetaObject SLTraceWidget::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_SLTraceWidget.data,
      qt_meta_data_SLTraceWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLTraceWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLTraceWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLTraceWidget.stringdata0))
        return static_cast<void*>(this);
    return QLabel::qt_metacast(_clname);
}

int SLTraceWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
