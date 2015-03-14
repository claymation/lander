#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import geometry_msgs.msg

from lander.lib.controller import Controller


# Maximum descent rate, in m/s
DEFAULT_MAX_DESCENT_RATE = 5.0


class DescendController(Controller):
    """
    Descend toward the landing target.

    The general strategy here is to control vehicle velocity using a simple PID
    controller, using target position error as input, and constraining vertical
    velocity to within sane limits. The rate of descent is proportional to error
    and inversely proportional to altitude.
    """
    def __init__(self, *args, **kwargs):
        super(DescendController, self).__init__(*args, **kwargs)

        self.max_descent_rate = rospy.get_param("max_descent_rate",
                DEFAULT_MAX_DESCENT_RATE)

        rospy.Subscriber("/tracker/error",
                geometry_msgs.msg.Point,
                self.handle_error_message)

    def handle_error_message(self, msg):
        self.error = msg

    def run(self):
        e = self.error

        kP = 0.25

        vx = -e.x * kP
        vy = -e.y * kP
        vz = -min(e.z * kP, self.max_descent_rate)

        setpoint = (vx, vy, vz, 0)
        print setpoint
        self.vehicle.set_velocity_setpoint(setpoint)
