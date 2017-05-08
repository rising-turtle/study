/****************************************************************************
** MapView meta object code from reading C++ file 'mapview.h'
**
** Created: Fri Oct 26 00:28:31 2012
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "mapview.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MapView::className() const
{
    return "MapView";
}

QMetaObject *MapView::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MapView( "MapView", &MapView::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MapView::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapView", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MapView::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapView", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MapView::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QScrollView::staticMetaObject();
    metaObj = QMetaObject::new_metaobject(
	"MapView", parentObject,
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MapView.setMetaObject( metaObj );
    return metaObj;
}

void* MapView::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MapView" ) )
	return this;
    return QScrollView::qt_cast( clname );
}

bool MapView::qt_invoke( int _id, QUObject* _o )
{
    return QScrollView::qt_invoke(_id,_o);
}

bool MapView::qt_emit( int _id, QUObject* _o )
{
    return QScrollView::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MapView::qt_property( int id, int f, QVariant* v)
{
    return QScrollView::qt_property( id, f, v);
}

bool MapView::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
