PROJECT(UI)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
# with SET() command you can change variables or define new ones
# here we define UI_SRCS variable that contains a list of all .cpp files
# note that we don't need \ at the end of line
	SET( UI_SRCS
			gen_points.cpp
			points.cpp
			main.cpp
			mywindow.cpp
	   )

# another list, this time it includes all header files that should be treated with moc
	SET( UI_MOC_HDRS
			points.h
			gen_points.h     
			mywindow.h
	   )

# some .ui files
#	SET( UI_UIS
#	   )

# and finally an resource file
#	SET( UI_RCS
#			./src/rc/sample.qrc
#	   )

# enable warnings
ADD_DEFINITIONS( -DDEBUG )

# by default only QtCore and QtGui modules are enabled
# other modules must be enabled like this:
SET( QT_USE_QT3SUPPORT TRUE )   
SET( QT_USE_QTXML TRUE )

# this command finds Qt4 libraries and sets all required variables
# note that it's Qt4, not QT4 or qt4
FIND_PACKAGE( Qt4 REQUIRED )
SET(QT_USE_QTOPENGL TRUE)
SET(LINKS 
	glut
	GLU
	GL
)

# add some useful macros and variables
# (QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ) that contains a path to CMake script)
INCLUDE( ${QT_USE_FILE} )

# this command will generate rules that will run rcc on all files from UI_RCS
# in result UI_RC_SRCS variable will contain paths to files produced by rcc
QT4_ADD_RESOURCES( UI_RC_SRCS ${UI_RCS} )

# this will run uic on .ui files:
QT4_WRAP_UI( UI_UI_HDRS ${UI_UIS} )

# and finally this will run moc:
QT4_WRAP_CPP( UI_MOC_SRCS ${UI_MOC_HDRS} )

# we need this to be able to include headers produced by uic in our code
# (CMAKE_BINARY_DIR holds a path to the build directory, while INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )

# here we instruct CMake to build "sample" executable from all of the source files
ADD_EXECUTABLE( sample ${UI_SRCS} ${UI_MOC_SRCS} ${UI_RC_SRCS} ${UI_UI_HDRS} )

# last thing we have to do is to tell CMake what libraries our executable needs,
# luckily FIND_PACKAGE prepared QT_LIBRARIES variable for us:
TARGET_LINK_LIBRARIES( sample ${QT_LIBRARIES} )
