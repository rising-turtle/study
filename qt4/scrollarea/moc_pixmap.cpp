/****************************************************************************
** Meta object code from reading C++ file 'pixmap.h'
**
** Created: Wed Oct 31 00:57:03 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "pixmap.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pixmap.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PixmapWidget[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      39,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PixmapWidget[] = {
    "PixmapWidget\0\0zoomFactorChanged(float)\0"
    "setZoomFactor(float)\0"
};

const QMetaObject PixmapWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PixmapWidget,
      qt_meta_data_PixmapWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PixmapWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PixmapWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PixmapWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PixmapWidget))
        return static_cast<void*>(const_cast< PixmapWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PixmapWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: zoomFactorChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: setZoomFactor((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void PixmapWidget::zoomFactorChanged(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
