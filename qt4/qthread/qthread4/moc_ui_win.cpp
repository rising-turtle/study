/****************************************************************************
** Meta object code from reading C++ file 'ui_win.h'
**
** Created: Wed Dec 12 22:34:49 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ui_win.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ui_win.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CUIWin[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x05,
      24,    7,    7,    7, 0x05,
      40,    7,    7,    7, 0x05,
      50,    7,    7,    7, 0x05,

 // slots: signature, parameters, type, tag, flags
      67,    7,    7,    7, 0x0a,
      81,    7,    7,    7, 0x0a,
      94,    7,    7,    7, 0x0a,
     105,    7,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CUIWin[] = {
    "CUIWin\0\0stopThreadPro()\0stopThreadCon()\0"
    "stopApp()\0stopThreadTest()\0startSystem()\0"
    "stopSystem()\0stopTest()\0stopAll()\0"
};

const QMetaObject CUIWin::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CUIWin,
      qt_meta_data_CUIWin, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CUIWin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CUIWin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CUIWin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CUIWin))
        return static_cast<void*>(const_cast< CUIWin*>(this));
    return QWidget::qt_metacast(_clname);
}

int CUIWin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: stopThreadPro(); break;
        case 1: stopThreadCon(); break;
        case 2: stopApp(); break;
        case 3: stopThreadTest(); break;
        case 4: startSystem(); break;
        case 5: stopSystem(); break;
        case 6: stopTest(); break;
        case 7: stopAll(); break;
        default: ;
        }
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void CUIWin::stopThreadPro()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void CUIWin::stopThreadCon()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void CUIWin::stopApp()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void CUIWin::stopThreadTest()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}
QT_END_MOC_NAMESPACE
