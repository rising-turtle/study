/****************************************************************************
** Form interface generated from reading ui file 'gui.ui'
**
** Created: Fri Oct 26 00:28:31 2012
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef MAPGUI_H
#define MAPGUI_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include "carmen/logtools.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
class MapView;
class QGroupBox;
class QPushButton;
class QFrame;

class MapGUI : public QMainWindow
{
    Q_OBJECT

public:
    MapGUI( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~MapGUI();

    MapView* Map;
    QGroupBox* Actions;
    QPushButton* UpdateMap;
    QPushButton* CenterRobot;
    QFrame* Frame3_2;
    QPushButton* GlobalMapType;
    QPushButton* LocalMapType;
    QPushButton* ShowBeamsType;
    QFrame* Frame3_2_2;
    QPushButton* ClearMap;
    QMenuBar *menubar;
    QPopupMenu *fileMenu;
    QPopupMenu *settingsMenu;
    QAction* fileNewAction;
    QAction* fileOpenAction;
    QAction* fileSaveAction;
    QAction* fileSaveAsAction;
    QAction* filePrintAction;
    QAction* fileExitAction;
    QAction* ChangeMapAction;
    QAction* ViewPathAction;

public slots:
    virtual void UpdateMapSlot();
    virtual void CenterRobotSlot();
    virtual void ClearMapSlot();
    virtual void MenuNewSlot();
    virtual void MenuOpenSlot();
    virtual void MenuPrintSlot();
    virtual void MenuExitSlot();
    virtual void GlobalMapTypeSlot();
    virtual void LocalMapTypeSlot();
    virtual void ShowBeamsTypeSlot();
    virtual void ViewPathSlot();
    virtual void ChangeMapSlot();
    virtual void ClearMapSlot( bool );

protected:
    QHBoxLayout* MapGUILayout;
    QHBoxLayout* Layout5;
    QVBoxLayout* Layout3;

protected slots:
    virtual void languageChange();

private:
    QPixmap image0;

};

#endif // MAPGUI_H
