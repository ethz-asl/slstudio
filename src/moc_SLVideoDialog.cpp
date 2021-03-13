/****************************************************************************
** Meta object code from reading C++ file 'SLVideoDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLVideoDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLVideoDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLVideoDialog_t {
    QByteArrayData data[8];
    char stringdata0[86];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLVideoDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLVideoDialog_t qt_meta_stringdata_SLVideoDialog = {
    {
QT_MOC_LITERAL(0, 0, 13), // "SLVideoDialog"
QT_MOC_LITERAL(1, 14, 11), // "showImageCV"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 7), // "cv::Mat"
QT_MOC_LITERAL(4, 35, 5), // "image"
QT_MOC_LITERAL(5, 41, 14), // "showImageSeqCV"
QT_MOC_LITERAL(6, 56, 20), // "std::vector<cv::Mat>"
QT_MOC_LITERAL(7, 77, 8) // "imageSeq"

    },
    "SLVideoDialog\0showImageCV\0\0cv::Mat\0"
    "image\0showImageSeqCV\0std::vector<cv::Mat>\0"
    "imageSeq"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLVideoDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x0a /* Public */,
       5,    1,   27,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,

       0        // eod
};

void SLVideoDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLVideoDialog *_t = static_cast<SLVideoDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->showImageCV((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 1: _t->showImageSeqCV((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject SLVideoDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLVideoDialog.data,
      qt_meta_data_SLVideoDialog,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLVideoDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLVideoDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLVideoDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int SLVideoDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
