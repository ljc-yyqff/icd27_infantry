
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TEMPLATE = app

LIBS += -lgxiapi\

TARGET = GxViewer
TEMPLATE = app

unix:!mac:QMAKE_LFLAGS += -L/usr/lib -L./ -Wl,--rpath=.:/usr/lib

INCLUDEPATH += ../../inc/

SOURCES += main.cpp\
    GxViewer.cpp \
    ImageImprovement.cpp \
    AcquisitionThread.cpp \
    Fps.cpp \
    ViewerCommon.cpp \
    ../Common/Common.cpp

HEADERS += $$files(./include/*.h) \
    Fps.h

HEADERS  += \
    GxViewer.h \
    ImageImprovement.h \
    AcquisitionThread.h \
    ViewerCommon.h \
    ../Common/Common.h

FORMS    += \
    GxViewer.ui \
    ImageImprovement.ui

