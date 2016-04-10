#-------------------------------------------------
#
# Project created by QtCreator 2016-03-04T14:38:58
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Visual-SLAM
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    framedata.cpp

CONFIG += c++11

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -std=c++0x

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3

LIBS += -L/usr/local/lib -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_flann -lopencv_imgcodecs -lopencv_video -lopencv_xobjdetect -lopencv_videoio -lopencv_tracking -lopencv_features2d -lopencv_calib3d

HEADERS += \
    framedata.h
