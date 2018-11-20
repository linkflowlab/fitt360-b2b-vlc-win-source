#!/bin/sh

gdb --args /usr/local/bin/vlc -I "qt" --video-filter stitching rtsp://1.214.28.210:8551/all
