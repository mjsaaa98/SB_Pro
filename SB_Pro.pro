QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG += C++11 #开启C++11新特性

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_highgui.so.3.3 \
        /usr/local/lib/libopencv_core.so.3.3    \
        /usr/local/lib/libopencv_imgproc.so.3.3 \
        /usr/local/lib/libopencv_imgcodecs.so.3.3\
        /usr/local/lib/libopencv_videoio.so.3.3\
        /usr/local/lib/libopencv_calib3d.so.3.3\
        /usr/local/lib/libopencv_ml.so\
        /usr/local/lib/libopencv_dnn.so.3.3
SOURCES += main.cpp \
    camera_calibration.cpp \
    solvepnp.cpp \
    stereo_vision.cpp \
    armorpredict.cpp \
    find_armour.cpp \
    serialport.cpp \
    CRC_Check.cpp \
    v4l2_set.cpp

HEADERS += \
    camera_calibration.h \
    Header.h \
    solvepnp.h \
    stereo_vision.h \
    armorpredict.h \
    find_armour.h \
    serialport.h \
    CRC_Check.h \
    v4l2_set.h
