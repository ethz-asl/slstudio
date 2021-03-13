/****************************************************************************
** Meta object code from reading C++ file 'SLVideoWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLVideoWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLVideoWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLVideoWidget_t {
    QByteArrayData data[10];
    char stringdata0[95];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLVideoWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLVideoWidget_t qt_meta_stringdata_SLVideoWidget = {
    {
QT_MOC_LITERAL(0, 0, 13), // "SLVideoWidget"
QT_MOC_LITERAL(1, 14, 9), // "showFrame"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 11), // "CameraFrame"
QT_MOC_LITERAL(4, 37, 5), // "frame"
QT_MOC_LITERAL(5, 43, 11), // "showFrameCV"
QT_MOC_LITERAL(6, 55, 7), // "cv::Mat"
QT_MOC_LITERAL(7, 63, 11), // "resizeEvent"
QT_MOC_LITERAL(8, 75, 13), // "QResizeEvent*"
QT_MOC_LITERAL(9, 89, 5) // "event"

    },
    "SLVideoWidget\0showFrame\0\0CameraFrame\0"
    "frame\0showFrameCV\0cv::Mat\0resizeEvent\0"
    "QResizeEvent*\0event"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLVideoWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x0a /* Public */,
       5,    1,   32,    2, 0x0a /* Public */,
       7,    1,   35,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    4,
    QMetaType::Void, 0x80000000 | 8,    9,

       0        // eod
};

void SLVideoWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLVideoWidget *_t = static_cast<SLVideoWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->showFrame((*reinterpret_cast< CameraFrame(*)>(_a[1]))); break;
        case 1: _t->showFrameCV((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->resizeEvent((*reinterpret_cast< QResizeEvent*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject SLVideoWidget::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_SLVideoWidget.data,
      qt_meta_data_SLVideoWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLVideoWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLVideoWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLVideoWidget.stringdata0))
        return static_cast<void*>(this);
    return QLabel::qt_metacast(_clname);
}

int SLVideoWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
