QT += core
QT -= gui

CONFIG += c++11

TARGET = main
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    main.c \
    src/dynamixel_sdk/group_bulk_read.c \
    src/dynamixel_sdk/group_bulk_write.c \
    src/dynamixel_sdk/group_sync_read.c \
    src/dynamixel_sdk/group_sync_write.c \
    src/dynamixel_sdk/packet_handler.c \
    src/dynamixel_sdk/port_handler.c \
    src/dynamixel_sdk/protocol1_packet_handler.c \
    src/dynamixel_sdk/protocol2_packet_handler.c \
    src/dynamixel_sdk_linux/port_handler_linux.c \
    src/dynamixel_sdk/group_bulk_read.cpp \
    src/dynamixel_sdk/group_bulk_write.cpp \
    src/dynamixel_sdk/group_sync_read.cpp \
    src/dynamixel_sdk/group_sync_write.cpp \
    src/dynamixel_sdk/packet_handler.cpp \
    src/dynamixel_sdk/port_handler.cpp \
    src/dynamixel_sdk/protocol1_packet_handler.cpp \
    src/dynamixel_sdk/protocol2_packet_handler.cpp \
    src/dynamixel_sdk_linux/port_handler_linux.cpp \
    src/dynamixel_sdk_windows/port_handler_windows.cpp \
    src/command/command.cpp \
    command.cpp

DISTFILES += \
    main \
    main.pro.user \
    angmaior.txt \
    angsfim.txt \
    angulos_adj.txt \
    calibra.txt \
    passo2.txt \
    Makefile

HEADERS += \
    include/dynamixel_sdk.h \
    include/dynamixel_sdk/group_bulk_read.h \
    include/dynamixel_sdk/group_bulk_write.h \
    include/dynamixel_sdk/group_sync_read.h \
    include/dynamixel_sdk/group_sync_write.h \
    include/dynamixel_sdk/packet_handler.h \
    include/dynamixel_sdk/port_handler.h \
    include/dynamixel_sdk/protocol1_packet_handler.h \
    include/dynamixel_sdk/protocol2_packet_handler.h \
    include/dynamixel_sdk/robotis_def.h \
    include/dynamixel_sdk_linux/port_handler_linux.h \
    include/dynamixel_sdk_windows/port_handler_windows.h \
    include/command.h
