#----------------------------------------------#
#           N-D Visualization GUI              #
#                                              #
#        Copyright (c) ViGIR-Lab 2019          #
#          Written by Ali Shfiekhani           #
#----------------------------------------------#

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VisND
TEMPLATE = app

INCLUDEPATH += /usr/include/pcl-1.7 \
               /usr/include/vtk-6.2


SOURCES += main.cpp \
           planedialog.cpp\
           visualize_nd.cpp


DESTDIR = ../bin

CONFIG += staticlib
QTPLUGIN += qico

HEADERS  += instance_cloud.h \
            planedialog.h \
            visualize_nd.h


FORMS    += planedialog.ui \
            visualize_nd.ui


RESOURCES += resouce.qrc
