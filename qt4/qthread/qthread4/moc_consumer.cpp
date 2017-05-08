/****************************************************************************
** Meta object code from reading C++ file 'consumer.h'
**
** Created: Wed Dec 12 22:32:40 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "consumer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'consumer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CConsumer[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x05,
      22,   10,   10,   10, 0x05,
      32,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      43,   10,   10,   10, 0x0a,
      56,   10,   10,   10, 0x0a,
      66,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CConsumer[] = {
    "CConsumer\0\0consumed()\0back(int)\0"
    "finished()\0consume(int)\0stopCon()\0"
    "record()\0"
};

const QMetaObject CConsumer::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_CConsumer,
      qt_meta_data_CConsumer, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CConsumer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CConsumer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CConsumer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CConsumer))
        return static_cast<void*>(const_cast< CConsumer*>(this));
    return QObject::qt_metacast(_clname);
}

int CConsumer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: consumed(); break;
        case 1: back((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: finished(); break;
        case 3: consume((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: stopCon(); break;
        case 5: record(); break;
        default: ;
        }
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void CConsumer::consumed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void CConsumer::back(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CConsumer::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
