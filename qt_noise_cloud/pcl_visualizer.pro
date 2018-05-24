#-------------------------------------------------
#
# Project created by QtCreator 2014-11-11T14:00:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH +=       /usr/include/pcl-1.7  \
                    /usr/include/pcl-1.7/pcl  \
                    /usr/include/pcl-1.7/pcl/common  \
                    /usr/include/pcl-1.7/pcl/io  \
                    /usr/include/vtk-5.8    \

LIBS +=
            /usr/lib/libpcl_common.so.1.7 \
            /usr/lib/libpcl_io.so.1.7 \
            /usr/lib/libpcl_io_ply.so.1.7 \
            /usr/lib/libQVTK.so.5.8 \
            /usr/lib/libvtkViews.so \
            /usr/lib/libvtkRendering.so \
TARGET = colorize_cloud
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    pairalign.cpp

HEADERS  += pclviewer.h \
    pairalign.h

FORMS    += pclviewer.ui
