#!/bin/sh

export OPENCV_LIBS='/win64/opencv-3.4.4/build/modules/world/CMakeFiles/opencv_world.dir/objects.a /win64/opencv-3.4.4/build/3rdparty/lib/libzlib.a'

export OPENCV_CFLAGS='-I/win64/opencv-3.4.4/build/usr/local/include/ -I/win64/mingw-std-threads/'

export CFLAGS_vlc='-O3'

export CPPFLAGS_vlc='-O3'

./extras/package/win32/build.sh -a x86_64 -i r
