TARGET = AA-SIPP
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
INCLUDEPATH += D:/boost_1_64_0/
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxmlparser.cpp \
    tinyxmlerror.cpp \
    tinyxml.cpp \
    tinystr.cpp \
    cXmlLogger.cpp \
    cMission.cpp \
    cMap.cpp \
    cLogger.cpp \
    cConfig.cpp \
    sipp.cpp \
    aa_sipp.cpp \
    constraints.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
    searchresult.h \
    gl_const.h \
    cXmlLogger.h \
    cSearch.h \
    cMission.h \
    cMap.h \
    cLogger.h \
    cConfig.h \
    sipp.h \
    aa_sipp.h \
    structs.h \
    constraints.h \
    StatesContainer.h
