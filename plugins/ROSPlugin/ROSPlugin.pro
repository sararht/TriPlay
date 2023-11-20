#-------------------------------------------------
#
# Project created by QtCreator 2023-10-17T12:51:48
#
#-------------------------------------------------

QT       -= gui
QMAKE_LFLAGS += -rdynamic
QMAKE_POST_LINK += export ROS_MASTER_URI=http://localhost:11311
QMAKE_POST_LINK += export ROS_PACKAGE_PATH=/home/sara/robotarm_ws/src:/opt/ros/melodic/share

TEMPLATE = lib
CONFIG += plugin

INCLUDEPATH    += ../
INCLUDEPATH += /opt/ros/melodic/include
INCLUDEPATH += /usr/include/eigen3

DEFINES += ROSPLUGIN_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


LIBS += -L/opt/ros/melodic/lib -lroscpp -lroslib -lrostime -lmoveit_planning_interface -lmoveit_move_group_interface
LIBS += $$catkin_LIBRARIES

SOURCES += \
        rosplugin.cpp

HEADERS += \
        rosplugin.h \
        rosplugin_global.h 


TARGET          = $$qtLibraryTarget(rosplugin)
DESTDIR         = ../

unix {
    target.path = /usr/lib
    INSTALLS += target
}

DISTFILES += \
    rosplugin.json

