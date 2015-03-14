#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import geometry_msgs.msg

from lander.lib.controller import Controller
from lander.lib.state import FlightState


# By default, we assume the target is located at the origin,
# that is, where the vehicle initialized its interial nav
DEFAULT_TARGET_LOCAL_POSITION = (0, 0, 0)

# By default, we approach the target at an altitude of 15m
DEFAULT_TARGET_SEEK_ALTITUDE = 15


class SeekController(Controller):
    """
    Seek the landing target.

    The general strategy here is to leverage the autopilot's waypoint navigation
    functionality by setting a position setpoint and letting the autopilot take
    us there. The tracker node, which is constantly surveiling the ground for the
    landing target, should start to provide target error estimates once we're
    in proximity to the target. We'll use that as an indication that it's time
    to transition to the DESCEND state of the flight program.

    For now, we have a static landing target position, so setting a location
    setpoint at 10 Hz is like beating a dead horse like beating a dead horse
    like beating a dead horse like -- you get the idea. But eventually we'll
    be receiving dynamic updates of the target position, and then this will
    be more useful. Besides, PX4 requires a constant stream of position msgs
    for heartbeat to prevent an offboard failsafe.
    """

    def __init__(self, *args, **kwargs):
        super(SeekController, self).__init__(*args, **kwargs)

        self.target_local_position = rospy.get_param("target_local_position",
                DEFAULT_TARGET_LOCAL_POSITION)
        self.target_seek_altitude = rospy.get_param("target_seek_altitude",
                DEFAULT_TARGET_SEEK_ALTITUDE)

        rospy.Subscriber("/tracker/error",
                geometry_msgs.msg.Point,
                self.handle_error_message)

    def handle_error_message(self, msg):
        """
        Transition to DESCEND state when we start receiving target error messages.
        """
        # TODO: Find a better way to transition between states
        # TODO: Require a minimum certainty (covariance) before transitioning
        if self.commander.state == FlightState.SEEK:
            self.commander.transition_to_state(FlightState.DESCEND)

    def run(self):
        # Construct an (x, y, z, yaw) setpoint, in local coordinates
        # NB: yaw currently has no effect (with ArduCopter, at least)
        x, y, z = self.target_local_position
        z += self.target_seek_altitude
        setpoint = (x, y, z, 0)

        # Send the vehicle on its merry way
        self.vehicle.set_location_setpoint(setpoint)
