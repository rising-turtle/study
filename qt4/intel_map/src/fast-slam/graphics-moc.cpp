/****************************************************************************
** MapPainter meta object code from reading C++ file 'graphics.h'
**
** Created: Fri Oct 26 01:31:36 2012
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "graphics.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MapPainter::className() const
{
    return "MapPainter";
}

QMetaObject *MapPainter::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MapPainter( "MapPainter", &MapPainter::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MapPainter::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapPainter", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MapPainter::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MapPainter", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MapPainter::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QScrollView::staticMetaObject();
    metaObj = QMetaObject::new_metaobject(
	"MapPainter", parentObject,
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MapPainter.setMetaObject( metaObj );
    return metaObj;
}

void* MapPainter::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MapPainter" ) )
	return this;
    return QScrollView::qt_cast( clname );
}

bool MapPainter::qt_invoke( int _id, QUObject* _o )
{
    return QScrollView::qt_invoke(_id,_o);
}

bool MapPainter::qt_emit( int _id, QUObject* _o )
{
    return QScrollView::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MapPainter::qt_property( int id, int f, QVariant* v)
{
    return QScrollView::qt_property( id, f, v);
}

bool MapPainter::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
