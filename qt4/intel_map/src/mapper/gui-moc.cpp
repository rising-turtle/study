/****************************************************************************
** MapGUI meta object code from reading C++ file 'gui.h'
**
** Created: Fri Oct 26 00:28:31 2012
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "gui.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MapGUI::className() const
{
    return "MapGUI";
}

QMetaObject *MapGUI::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MapGUI( "MapGUI", &MapGUI::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MapGUI::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapGUI", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MapGUI::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapGUI", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MapGUI::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUMethod slot_0 = {"UpdateMapSlot", 0, 0 };
    static const QUMethod slot_1 = {"CenterRobotSlot", 0, 0 };
    static const QUMethod slot_2 = {"ClearMapSlot", 0, 0 };
    static const QUMethod slot_3 = {"MenuNewSlot", 0, 0 };
    static const QUMethod slot_4 = {"MenuOpenSlot", 0, 0 };
    static const QUMethod slot_5 = {"MenuPrintSlot", 0, 0 };
    static const QUMethod slot_6 = {"MenuExitSlot", 0, 0 };
    static const QUMethod slot_7 = {"GlobalMapTypeSlot", 0, 0 };
    static const QUMethod slot_8 = {"LocalMapTypeSlot", 0, 0 };
    static const QUMethod slot_9 = {"ShowBeamsTypeSlot", 0, 0 };
    static const QUMethod slot_10 = {"ViewPathSlot", 0, 0 };
    static const QUMethod slot_11 = {"ChangeMapSlot", 0, 0 };
    static const QUParameter param_slot_12[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_12 = {"ClearMapSlot", 1, param_slot_12 };
    static const QUMethod slot_13 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "UpdateMapSlot()", &slot_0, QMetaData::Public },
	{ "CenterRobotSlot()", &slot_1, QMetaData::Public },
	{ "ClearMapSlot()", &slot_2, QMetaData::Public },
	{ "MenuNewSlot()", &slot_3, QMetaData::Public },
	{ "MenuOpenSlot()", &slot_4, QMetaData::Public },
	{ "MenuPrintSlot()", &slot_5, QMetaData::Public },
	{ "MenuExitSlot()", &slot_6, QMetaData::Public },
	{ "GlobalMapTypeSlot()", &slot_7, QMetaData::Public },
	{ "LocalMapTypeSlot()", &slot_8, QMetaData::Public },
	{ "ShowBeamsTypeSlot()", &slot_9, QMetaData::Public },
	{ "ViewPathSlot()", &slot_10, QMetaData::Public },
	{ "ChangeMapSlot()", &slot_11, QMetaData::Public },
	{ "ClearMapSlot(bool)", &slot_12, QMetaData::Public },
	{ "languageChange()", &slot_13, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"MapGUI", parentObject,
	slot_tbl, 14,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MapGUI.setMetaObject( metaObj );
    return metaObj;
}

void* MapGUI::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MapGUI" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool MapGUI::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: UpdateMapSlot(); break;
    case 1: CenterRobotSlot(); break;
    case 2: ClearMapSlot(); break;
    case 3: MenuNewSlot(); break;
    case 4: MenuOpenSlot(); break;
    case 5: MenuPrintSlot(); break;
    case 6: MenuExitSlot(); break;
    case 7: GlobalMapTypeSlot(); break;
    case 8: LocalMapTypeSlot(); break;
    case 9: ShowBeamsTypeSlot(); break;
    case 10: ViewPathSlot(); break;
    case 11: ChangeMapSlot(); break;
    case 12: ClearMapSlot((bool)static_QUType_bool.get(_o+1)); break;
    case 13: languageChange(); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool MapGUI::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MapGUI::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool MapGUI::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
