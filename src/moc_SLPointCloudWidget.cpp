/****************************************************************************
** Meta object code from reading C++ file 'SLPointCloudWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLPointCloudWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLPointCloudWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLPointCloudWidget_t {
    QByteArrayData data[9];
    char stringdata0[142];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLPointCloudWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLPointCloudWidget_t qt_meta_stringdata_SLPointCloudWidget = {
    {
QT_MOC_LITERAL(0, 0, 18), // "SLPointCloudWidget"
QT_MOC_LITERAL(1, 19, 22), // "newPointCloudDisplayed"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 16), // "updatePointCloud"
QT_MOC_LITERAL(4, 60, 18), // "PointCloudConstPtr"
QT_MOC_LITERAL(5, 79, 14), // "_pointCloudPCL"
QT_MOC_LITERAL(6, 94, 14), // "savePointCloud"
QT_MOC_LITERAL(7, 109, 14), // "saveScreenShot"
QT_MOC_LITERAL(8, 124, 17) // "updateCalibration"

    },
    "SLPointCloudWidget\0newPointCloudDisplayed\0"
    "\0updatePointCloud\0PointCloudConstPtr\0"
    "_pointCloudPCL\0savePointCloud\0"
    "saveScreenShot\0updateCalibration"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLPointCloudWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    1,   40,    2, 0x0a /* Public */,
       6,    0,   43,    2, 0x0a /* Public */,
       7,    0,   44,    2, 0x0a /* Public */,
       8,    0,   45,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SLPointCloudWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLPointCloudWidget *_t = static_cast<SLPointCloudWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newPointCloudDisplayed(); break;
        case 1: _t->updatePointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->savePointCloud(); break;
        case 3: _t->saveScreenShot(); break;
        case 4: _t->updateCalibration(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SLPointCloudWidget::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SLPointCloudWidget::newPointCloudDisplayed)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject SLPointCloudWidget::staticMetaObject = {
    { &QVTKWidget::staticMetaObject, qt_meta_stringdata_SLPointCloudWidget.data,
      qt_meta_data_SLPointCloudWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLPointCloudWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLPointCloudWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLPointCloudWidget.stringdata0))
        return static_cast<void*>(this);
    return QVTKWidget::qt_metacast(_clname);
}

int SLPointCloudWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QVTKWidget::qt_metacall(_c, _id, _a);
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
void SLPointCloudWidget::newPointCloudDisplayed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
