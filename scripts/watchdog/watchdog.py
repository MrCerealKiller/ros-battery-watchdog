#! /usr/bin/env python

import os
import numpy
import rospy

from collections import deque
from std_msgs.msg import Float64


class BatteryWatchdog:
    def __init__(self):
        # Parameters for the monitoring
        self.sample_size = rospy.get_param('/battery_watchdog/sample_size', 25)
        self.warn_thresh = rospy.get_param('/battery_watchdog/warn_thresh', 14.3)
        self.warn_offset = rospy.get_param('/battery_watchdog/warn_offset', 20)
        self.crit_thresh = rospy.get_param('battery_watchdog/crit_thresh', 14.2)
        self.crit_offset = rospy.get_param('/battery_watchdog/crit_offset', 20)
        self.warn_rate = rospy.get_param('/battery_watchdog/warn_rate', 10)

        # Circular Deque to get rolling average of battery voltage
        self.samples = deque(maxlen=self.sample_size)
        self.voltage = 9999

        # Track Warning and Critical Data
        self.last_warn = rospy.Time.now()
        self.warn_count = 0
        self.crit_count = 0

    def cb(self, msg):
        # Update "Rolling" Mean
        self.samples.append(msg.data)
        self.voltage = numpy.mean(self.samples)

        # Check for critical status
        if (self.voltage < self.crit_thresh):
            self.crit_count += 1
            if (self.crit_count > self.crit_offset):
                self.shutdown()
        else:
            self.crit_count = 0

        # Check for warning status
        if (self.voltage < self.warn_thresh):
            self.warn_count += 1
            if (self.warn_count > self.warn_offset):
                self.warn()
        else:
            self.warn_count = 0

        rospy.loginfo('Battery Voltage is: {} V'.format(self.voltage))

    def warn(self):
        now = rospy.Time.now()
        if ((now - self.last_warn).to_sec() > self.warn_rate):
            os.system('echo \'Battery voltage low!\' | wall')
            self.last_warn = now

    def shutdown(self):
        rospy.logfatal('Shutting down due to low battery')
        os.system('shutdown -P now \'Shutting down due to low battery!\'')


if __name__ == '__main__':
    rospy.init_node('watchdog')

    wd = BatteryWatchdog()
    topic = rospy.get_param('battery_watchdog/topic',
                            'battery_watchdog/voltage')
    sub = rospy.Subscriber(topic, Float64, wd.cb)

    rospy.loginfo('Battery Watchdog started')
    rospy.spin()
