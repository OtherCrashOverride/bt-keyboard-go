#!/bin/bash
rm Keyboard.fw
TITLE="Keyboard ($(date +%Y%m%d))"
./mkfw "$TITLE" tile.raw 0 16 1048576 keyboard build/keyboard-go.bin
mv firmware.fw Keyboard.fw
