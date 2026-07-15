#!/usr/bin/env python3
from control.api import Auv

with Auv() as auv:
    auv.submerge_to_depth(2.0)
    auv.move_forward(speed=1.0, duration=5)
    auv.turn(90, 3)
    auv.move_left(0.5, 3)
    auv.surface()
    auv.stop()

