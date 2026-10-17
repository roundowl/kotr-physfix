#!/bin/sh
# Builds dist/KoTR_PhysFix.asi with the MinGW-w64 32-bit cross compiler
# (Debian/Ubuntu: apt install gcc-mingw-w64-i686; on Windows use MSYS2's mingw32 gcc).
set -e
cd "$(dirname "$0")"
mkdir -p dist
${CC:-i686-w64-mingw32-gcc} -O2 -Wall -Wextra -Wno-unused-parameter -shared -static-libgcc -s \
    -o dist/KoTR_PhysFix.asi src/physfix.c
cp src/KoTR_PhysFix.ini dist/
echo "built dist/KoTR_PhysFix.asi"
