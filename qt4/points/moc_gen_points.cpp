/****************************************************************************
** Meta object code from reading C++ file 'gen_points.h'
**
** Created: Wed Sep 26 04:42:11 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "gen_points.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gen_points.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CGenPoints[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      23,   12,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   11,   11,   11, 0x0a,
      77,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CGenPoints[] = {
    "CGenPoints\0\0px,py,pz,n\0"
    "sendpoints(float*,float*,float*,int)\0"
    "generatePoints()\0start()\0"
};

const QMetaObject CGenPoints::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CGenPoints,
      qt_meta_data_CGenPoints, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CGenPoints::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CGenPoints::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CGenPoints::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CGenPoints))
        return static_cast<void*>(const_cast< CGenPoints*>(this));
    return QWidget::qt_metacast(_clname);
}

int CGenPoints::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: sendpoints((*reinterpret_cast< float*(*)>(_a[1])),(*reinterpret_cast< float*(*)>(_a[2])),(*reinterpret_cast< float*(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 1: generatePoints(); break;
        case 2: start(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void CGenPoints::sendpoints(float * _t1, float * _t2, float * _t3, int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
