/****************************************************************************
** Form implementation generated from reading ui file 'gui.ui'
**
** Created: Fri Oct 26 00:28:31 2012
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#include "gui.h"

#include <qvariant.h>
#include <mapview.h>
#include <qpushbutton.h>
#include <qgroupbox.h>
#include <qframe.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include <qimage.h>
#include <qpixmap.h>

#include "gui.ui.h"
/*
 *  Constructs a MapGUI as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 */
MapGUI::MapGUI( QWidget* parent, const char* name, WFlags fl )
    : QMainWindow( parent, name, fl )
{
    (void)statusBar();
    if ( !name )
	setName( "MapGUI" );
    setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, sizePolicy().hasHeightForWidth() ) );
    setMinimumSize( QSize( 540, 449 ) );
    setCentralWidget( new QWidget( this, "qt_central_widget" ) );
    MapGUILayout = new QHBoxLayout( centralWidget(), 0, 0, "MapGUILayout"); 

    Layout5 = new QHBoxLayout( 0, 0, 6, "Layout5"); 

    Map = new MapView( centralWidget(), "Map" );
    Map->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 1, 1, Map->sizePolicy().hasHeightForWidth() ) );
    Map->setMinimumSize( QSize( 400, 400 ) );
    Layout5->addWidget( Map );

    Actions = new QGroupBox( centralWidget(), "Actions" );
    Actions->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, Actions->sizePolicy().hasHeightForWidth() ) );
    Actions->setMinimumSize( QSize( 130, 400 ) );
    Actions->setMaximumSize( QSize( 130, 32767 ) );
    Actions->setLineWidth( 1 );
    Actions->setMargin( 0 );
    Actions->setMidLineWidth( 0 );

    QWidget* privateLayoutWidget = new QWidget( Actions, "Layout3" );
    privateLayoutWidget->setGeometry( QRect( 10, 21, 109, 288 ) );
    Layout3 = new QVBoxLayout( privateLayoutWidget, 0, 6, "Layout3"); 

    UpdateMap = new QPushButton( privateLayoutWidget, "UpdateMap" );
    Layout3->addWidget( UpdateMap );

    CenterRobot = new QPushButton( privateLayoutWidget, "CenterRobot" );
    Layout3->addWidget( CenterRobot );

    Frame3_2 = new QFrame( privateLayoutWidget, "Frame3_2" );
    Frame3_2->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)0, 0, 0, Frame3_2->sizePolicy().hasHeightForWidth() ) );
    Frame3_2->setMinimumSize( QSize( 0, 20 ) );
    Frame3_2->setFrameShape( QFrame::StyledPanel );
    Frame3_2->setFrameShadow( QFrame::Raised );
    Frame3_2->setLineWidth( 0 );
    Layout3->addWidget( Frame3_2 );

    GlobalMapType = new QPushButton( privateLayoutWidget, "GlobalMapType" );
    Layout3->addWidget( GlobalMapType );

    LocalMapType = new QPushButton( privateLayoutWidget, "LocalMapType" );
    Layout3->addWidget( LocalMapType );

    ShowBeamsType = new QPushButton( privateLayoutWidget, "ShowBeamsType" );
    Layout3->addWidget( ShowBeamsType );

    Frame3_2_2 = new QFrame( privateLayoutWidget, "Frame3_2_2" );
    Frame3_2_2->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)0, 0, 0, Frame3_2_2->sizePolicy().hasHeightForWidth() ) );
    Frame3_2_2->setMinimumSize( QSize( 0, 20 ) );
    Frame3_2_2->setFrameShape( QFrame::StyledPanel );
    Frame3_2_2->setFrameShadow( QFrame::Raised );
    Frame3_2_2->setLineWidth( 0 );
    Layout3->addWidget( Frame3_2_2 );

    ClearMap = new QPushButton( privateLayoutWidget, "ClearMap" );
    Layout3->addWidget( ClearMap );
    Layout5->addWidget( Actions );
    MapGUILayout->addLayout( Layout5 );

    // actions
    fileNewAction = new QAction( this, "fileNewAction" );
    fileNewAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );
    fileOpenAction = new QAction( this, "fileOpenAction" );
    fileOpenAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );
    fileSaveAction = new QAction( this, "fileSaveAction" );
    fileSaveAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );
    fileSaveAsAction = new QAction( this, "fileSaveAsAction" );
    filePrintAction = new QAction( this, "filePrintAction" );
    filePrintAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );
    fileExitAction = new QAction( this, "fileExitAction" );
    ChangeMapAction = new QAction( this, "ChangeMapAction" );
    ChangeMapAction->setToggleAction( TRUE );
    ChangeMapAction->setOn( TRUE );
    ChangeMapAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );
    ViewPathAction = new QAction( this, "ViewPathAction" );
    ViewPathAction->setToggleAction( TRUE );
    ViewPathAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "" ) ) );


    // toolbars


    // menubar
    menubar = new QMenuBar( this, "menubar" );


    fileMenu = new QPopupMenu( this );
    fileNewAction->addTo( fileMenu );
    fileOpenAction->addTo( fileMenu );
    fileMenu->insertSeparator();
    filePrintAction->addTo( fileMenu );
    fileMenu->insertSeparator();
    fileExitAction->addTo( fileMenu );
    menubar->insertItem( QString(""), fileMenu, 1 );

    settingsMenu = new QPopupMenu( this );
    ChangeMapAction->addTo( settingsMenu );
    ViewPathAction->addTo( settingsMenu );
    menubar->insertItem( QString(""), settingsMenu, 2 );

    languageChange();
    resize( QSize(557, 479).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( fileNewAction, SIGNAL( activated() ), this, SLOT( MenuNewSlot() ) );
    connect( fileOpenAction, SIGNAL( activated() ), this, SLOT( MenuOpenSlot() ) );
    connect( filePrintAction, SIGNAL( activated() ), this, SLOT( MenuPrintSlot() ) );
    connect( fileExitAction, SIGNAL( activated() ), this, SLOT( MenuExitSlot() ) );
    connect( UpdateMap, SIGNAL( clicked() ), this, SLOT( UpdateMapSlot() ) );
    connect( CenterRobot, SIGNAL( clicked() ), this, SLOT( CenterRobotSlot() ) );
    connect( GlobalMapType, SIGNAL( clicked() ), this, SLOT( GlobalMapTypeSlot() ) );
    connect( LocalMapType, SIGNAL( clicked() ), this, SLOT( LocalMapTypeSlot() ) );
    connect( ShowBeamsType, SIGNAL( clicked() ), this, SLOT( ShowBeamsTypeSlot() ) );
    connect( ClearMap, SIGNAL( clicked() ), this, SLOT( ClearMapSlot() ) );
    connect( ViewPathAction, SIGNAL( activated() ), this, SLOT( ViewPathSlot() ) );
    connect( ChangeMapAction, SIGNAL( activated() ), this, SLOT( ChangeMapSlot() ) );

    // tab order
    setTabOrder( UpdateMap, CenterRobot );
    setTabOrder( CenterRobot, GlobalMapType );
    setTabOrder( GlobalMapType, LocalMapType );
    setTabOrder( LocalMapType, ShowBeamsType );
    setTabOrder( ShowBeamsType, ClearMap );
}

/*
 *  Destroys the object and frees any allocated resources
 */
MapGUI::~MapGUI()
{
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void MapGUI::languageChange()
{
    setCaption( tr( "Map 2D" ) );
    Actions->setTitle( tr( "Actions" ) );
    UpdateMap->setText( tr( "Update Map" ) );
    CenterRobot->setText( tr( "Center Robot" ) );
    GlobalMapType->setText( tr( "Global Map" ) );
    LocalMapType->setText( tr( "Local Map" ) );
    ShowBeamsType->setText( tr( "Show Beams" ) );
    ClearMap->setText( tr( "Clear Map" ) );
    fileNewAction->setText( tr( "New" ) );
    fileNewAction->setMenuText( tr( "&New" ) );
    fileNewAction->setAccel( tr( "Ctrl+N" ) );
    fileOpenAction->setText( tr( "Open" ) );
    fileOpenAction->setMenuText( tr( "&Open..." ) );
    fileOpenAction->setAccel( tr( "Ctrl+O" ) );
    fileSaveAction->setText( tr( "Save" ) );
    fileSaveAction->setMenuText( tr( "&Save" ) );
    fileSaveAction->setAccel( tr( "Ctrl+S" ) );
    fileSaveAsAction->setText( tr( "Save As" ) );
    fileSaveAsAction->setMenuText( tr( "Save &As..." ) );
    fileSaveAsAction->setAccel( QString::null );
    filePrintAction->setText( tr( "Print" ) );
    filePrintAction->setMenuText( tr( "&Print..." ) );
    filePrintAction->setAccel( tr( "Ctrl+P" ) );
    fileExitAction->setText( tr( "Exit" ) );
    fileExitAction->setMenuText( tr( "E&xit" ) );
    fileExitAction->setStatusTip( tr( "Exit2" ) );
    fileExitAction->setAccel( QString::null );
    ChangeMapAction->setText( tr( "Open" ) );
    ChangeMapAction->setMenuText( tr( "Change Map" ) );
    ChangeMapAction->setAccel( QString::null );
    ViewPathAction->setText( tr( "View Path" ) );
    ViewPathAction->setMenuText( tr( "View Path" ) );
    ViewPathAction->setAccel( QString::null );
    if (menubar->findItem(1))
        menubar->findItem(1)->setText( tr( "&File" ) );
    if (menubar->findItem(2))
        menubar->findItem(2)->setText( tr( "Settings" ) );
}

