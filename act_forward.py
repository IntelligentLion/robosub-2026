#!/usr/bin/env python3
"""Isolated action: move FORWARD (ramped) then pause holding depth.

  python3 act_forward.py --speed 0.4 --ramp-up 1.5 --duration 3 --pause 2

See field_common.py for the shared safety notes and tuning flags.
"""

import argparse, rclpy
from depth_hold_bar02_test import main


from field_common import RampedDriver, ThrusterController, HeadingMonitor
#---------------init everything-------------#
rclpy.init()
driver = RampedDriver()

thrusters = ThrusterController()
heading = HeadingMonitor()
driver.attach_heading(heading)     # enables closed-loop turn_left/right
driver.thrusters = thrusters       # e.g. dropper shares thrusters.master



#--------actual movement--------#
#depth_hold_bar02_test.main()

driver.move_down(speed=1.00, ramp=10)
#driver.idle()
driver.turn_right(None, 1, None)

#driver.move_forward(speed=0.7, ramp=1)
driver.stop_move()






