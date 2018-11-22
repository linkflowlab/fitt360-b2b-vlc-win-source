#!/bin/sh

./configure PKG_CONFIG_PATH=/home/dragon/Qt/5.11.2/Src/qtbase/lib/pkgconfig:/home/dragon/Qt/5.11.2/Src/qtsvg/lib/pkgconfig:$PKG_CONFIG_PATH LIBS=-lX11 --enable-qt
