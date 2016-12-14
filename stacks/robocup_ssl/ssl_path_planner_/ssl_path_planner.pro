#-------------------------------------------------
#
# Project created by QtCreator 2011-06-23T19:51:32
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ssl_path_planner
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH +=/home/kadir/interconnection/workspace/packages/omplapp/ompl/src

SOURCES += \
    src/SSLPathPlanner.cpp \
    src/main.cpp

HEADERS += \
    include/ssl_path_planner/SSLPathPlanner.h

OTHER_FILES += \
    Makefile
