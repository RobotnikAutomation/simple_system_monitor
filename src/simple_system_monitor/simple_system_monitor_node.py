#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

try:
    from system_monitor import SimpleSystemMonitor
except ImportError:
    from simple_system_monitor.system_monitor import SimpleSystemMonitor


def main():

    rospy.init_node("simple_email_sender")

    rc_node = SimpleSystemMonitor()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
