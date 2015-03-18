#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy
import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState
from lander.lib.timers import HolddownTimer


# Maximum horizontal speed, in m/s
DEFAULT_MAX_SPEED_XY = 0.25

# Maximum horizontal acceleration, in m/s/s
DEFAULT_MAX_ACCEL_XY = 0.1

DEFAULT_TRACKING_HOLDDOWN = 1

DEFAULT_DESCEND_RADIUS = 1.0
DEFAULT_DESCEND_MAX_SPEED_XY = 0.25
DEFAULT_DESCEND_HOLDDOWN = 1


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
        self.descend_radius = rospy.get_param("descend_radius", DEFAULT_DESCEND_RADIUS)
        self.descend_max_speed_xy = rospy.get_param("descend_max_speed_xy",
                DEFAULT_DESCEND_MAX_SPEED_XY)
        self.descend_holddown = rospy.get_param("descend_holddown",
                DEFAULT_DESCEND_HOLDDOWN)

        self.descend_holddown_timer = HolddownTimer(self.descend_holddown)

    def enter(self):
        self.descend_holddown_timer.reset()
        self.setpoint = None
        self.damping_factor = 0

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

        distance = numpy.sqrt(err_x**2 + err_y**2)

        # Compute velocity setpoints
        # TODO: Implement I and D terms for full PID control
        Kp = 0.10 * self.damping_factor
        set_vx = Kp * err_x
        set_vy = Kp * err_y
        self.damping_factor = min(self.damping_factor + 0.01, 1.00)

        # Enforce speed constraint
        speed = numpy.sqrt(set_vx**2 + set_vy**2)
        if speed > self.max_speed_xy:
            set_vx = set_vx * self.max_speed_xy / speed
            set_vy = set_vy * self.max_speed_xy / speed

        # TODO: Enforce acceleration constraints

        target_is_close   = distance < self.descend_radius
        vehicle_is_stable = speed < self.descend_max_speed_xy

        # Transition to DESCEND state if:
        #   - we're within the approach radius of the target
        #   - the vehicle speed is within the approach speed threshold
        #   - those conditions have been true for the holddown period
        if self.descend_holddown_timer.test(target_is_close and vehicle_is_stable):
            self.commander.transition_to_state(FlightState.DESCEND)
            return

        # Otherwise, control velocity to minimize position error
        self.setpoint = (set_vx, set_vy, 0, 0)

        rospy.logdebug("error: (%6.2f, %6.2f)  setpoint: (%6.2f, %6.2f)  " +
                       "speed: %6.2f  distance: %6.2f",
                        err_x, err_y, set_vx, set_vy, speed, distance)

    def run(self):
        """
        Set the velocity setpoint once per control loop iteration.
        """
        if self.setpoint is not None:
            self.vehicle.set_velocity_setpoint(self.setpoint)
