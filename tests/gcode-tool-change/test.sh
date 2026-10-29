#!/bin/sh
# What gcode.parse does at an M6 for a program simulator, as task and
# iocontrol do on a machine: the private tool table's spindle slot, the move
# to the tool change position, where the controlled point is after offsets
# change, and which file each line comes from.
set -e
./test_tool_change.py >&2
echo ok
