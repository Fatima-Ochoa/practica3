# Práctica 3 · Diseño de Interfaces (I7262) 2026B
# Control del robot de la Práctica 2 desde Android (Qt 6) por WebSocket.
# El mismo proyecto compila para escritorio (MinGW / GCC) y para Android (arm64-v8a):
# solo cambia el kit seleccionado en Qt Creator.

QT += core gui widgets websockets charts

CONFIG += c++17

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    mainwindow.h

# Android: Qt genera el AndroidManifest.xml; el permiso INTERNET lo agrega
# el módulo de red (websockets depende de network). Nombre visible de la app:
android {
    ANDROID_TARGET_SDK_VERSION = 35
    ANDROID_VERSION_NAME = 1.0
    ANDROID_VERSION_CODE = 1
}

TARGET = RobotP3
