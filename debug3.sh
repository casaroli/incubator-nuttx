#!/bin/sh
elf="nuttx"
gdb -ex "target extended-remote localhost:3333" \
    "${elf}"
