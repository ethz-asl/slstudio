/****************************************************************************
** Meta object code from reading C++ file 'SLPreferenceDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SLPreferenceDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLPreferenceDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SLPreferenceDialog_t {
    QByteArrayData data[9];
    char stringdata0[233];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SLPreferenceDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SLPreferenceDialog_t qt_meta_stringdata_SLPreferenceDialog = {
    {
QT_MOC_LITERAL(0, 0, 18), // "SLPreferenceDialog"
QT_MOC_LITERAL(1, 19, 21), // "on_buttonBox_accepted"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 37), // "on_triggerHardwareRadioButton..."
QT_MOC_LITERAL(4, 80, 37), // "on_triggerSoftwareRadioButton..."
QT_MOC_LITERAL(5, 118, 37), // "on_cameraComboBox_currentInde..."
QT_MOC_LITERAL(6, 156, 4), // "arg1"
QT_MOC_LITERAL(7, 161, 36), // "on_patternHorizontalCheckBox_..."
QT_MOC_LITERAL(8, 198, 34) // "on_patternVerticalCheckBox_cl..."

    },
    "SLPreferenceDialog\0on_buttonBox_accepted\0"
    "\0on_triggerHardwareRadioButton_clicked\0"
    "on_triggerSoftwareRadioButton_clicked\0"
    "on_cameraComboBox_currentIndexChanged\0"
    "arg1\0on_patternHorizontalCheckBox_clicked\0"
    "on_patternVerticalCheckBox_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SLPreferenceDialog[] = {

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
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    1,   47,    2, 0x08 /* Private */,
       7,    0,   50,    2, 0x08 /* Private */,
       8,    0,   51,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SLPreferenceDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SLPreferenceDialog *_t = static_cast<SLPreferenceDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_buttonBox_accepted(); break;
        case 1: _t->on_triggerHardwareRadioButton_clicked(); break;
        case 2: _t->on_triggerSoftwareRadioButton_clicked(); break;
        case 3: _t->on_cameraComboBox_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->on_patternHorizontalCheckBox_clicked(); break;
        case 5: _t->on_patternVerticalCheckBox_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject SLPreferenceDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLPreferenceDialog.data,
      qt_meta_data_SLPreferenceDialog,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SLPreferenceDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SLPreferenceDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SLPreferenceDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int SLPreferenceDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
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
