#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy
import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState


# Maximum horizontal speed, in m/s
DEFAULT_MAX_SPEED_XY = 1.5

# Maximum horizontal acceleration, in m/s/s
DEFAULT_MAX_ACCEL_XY = 0.5


class ApproachController(Controller):
    """
    Approach the landing target while maintaining altitude.

    The general strategy here is to first minimize the velocity error,
    and then minimize position error.
    """
    def __init__(self, *args, **kwargs):
        super(ApproachController, self).__init__(*args, **kwargs)

        self.max_accel_xy = rospy.get_param("max_accel_xy", DEFAULT_MAX_ACCEL_XY)
        self.max_speed_xy = rospy.get_param("max_speed_xy", DEFAULT_MAX_SPEED_XY)

        self.vxs, self.vys = [], []

    def handle_track_message(self, msg):
        """
        Implement control logic to correct for velocity and position errors.
        """
        # Abort if we're no longer tracking the target
        if not msg.track.tracking.data:
            self.commander.relinquish_control()
            return

        # Position and velocity of vehicle
        veh_p = self.vehicle.position
        veh_v = self.vehicle.velocity

        # Position and velocity of target
        tgt_p = msg.track.position
        tgt_v = msg.track.velocity

        # Compute position error
        err_x = tgt_p.x - veh_p.x
        err_y = tgt_p.y - veh_p.y

        # Compute velocity setpoints
        # TODO: Implement I and D terms for full PID control
        Kp = 0.25
        set_vx = Kp * err_x
        set_vy = Kp * err_y

        # Enforce speed constraint
        speed = numpy.sqrt(set_vx**2 + set_vy**2)
        if speed > self.max_speed_xy:
            set_vx = set_vx * self.max_speed_xy / speed
            set_vy = set_vy * self.max_speed_xy / speed

        # TODO: Enforce acceleration constraints

        setpoint = (set_vx, set_vy, 0, 0)
        self.vehicle.set_velocity_setpoint(setpoint)

        print "error: (%6.2f, %6.2f)  setpoint: (%6.2f, %6.2f)  speed: %6.2f" % (
            err_x, err_y, set_vx, set_vy, speed)
