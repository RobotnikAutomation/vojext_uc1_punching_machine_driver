#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from punching_machine_driver.punching_driver import punchingMachineDriver


def main():

    rospy.init_node("punching_machine_driver_node")

    rc_node = punchingMachineDriver()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
