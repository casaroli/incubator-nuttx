#!/bin/sh
elf="nuttx"
gdb -ex "target extended-remote localhost:3333" \
    -ex "monitor reset init" \
    -ex "load ${elf}" \
    -ex 'set $msp = _vectors[0]' \
    -ex 'set $sp = _vectors[0]' \
    "${elf}"
