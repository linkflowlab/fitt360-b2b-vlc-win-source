#!/bin/sh

gcc $(pkg-config --cflags libvlc) -c test.c -o test.o
#gcc -I../include -c test.c -o test.o
gcc test.o -o test $(pkg-config --libs libvlc)
#gcc test.o -o test -L../lib/.libs -lvlc
