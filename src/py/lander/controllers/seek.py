#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy
import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState


# The landing target is located at the origin, i.e., where the vehicle's
# inertial navigation system was initialized (in meters from home)
DEFAULT_TARGET_LOCAL_POSITION = (0, 0, 0)

# The altitude at which to approach the target (in meters)
DEFAULT_TARGET_SEEK_ALTITUDE = 15

# The radius within which we'll transition to APPROACH state
# (assuming we can see the landing target) (in meters)
DEFAULT_APPROACH_RADIUS = 1.5


class SeekController(Controller):
    """
    Seek the landing target.

    The general strategy here is to leverage the autopilot's waypoint navigation
    functionality by setting a position setpoint and letting the autopilot take
    us there. The tracker node, which is constantly surveiling the ground for the
    landing target, should start to provide target tracking estimates once we're
    in proximity to the target. We'll use that as an indication that it's time
    to transition to the DESCEND state of the flight program.
    """

    def __init__(self, *args, **kwargs):
        super(SeekController, self).__init__(*args, **kwargs)

        self.target_local_position = rospy.get_param("target_local_position",
                DEFAULT_TARGET_LOCAL_POSITION)
        self.target_seek_altitude = rospy.get_param("target_seek_altitude",
                DEFAULT_TARGET_SEEK_ALTITUDE)
        self.approach_radius = rospy.get_param("approach_radius",
                DEFAULT_APPROACH_RADIUS)

        self.target_is_in_sight = False

    def handle_track_message(self, msg):
        """
        Determine whether we can see the target.
        """
        self.target_is_in_sight = msg.track.tracking.data

    def run(self):
        """
        Publish location setpoint once per control loop.

        For now, we have a static landing target position, so repeatedly setting
        the same location setpoint is like beating a dead horse like beating a
        dead horse like beating a dead horse like -- you get the idea.

        But eventually we'll receive dynamic updates of the target position, and
        then this will be more useful. Besides, PX4 requires a constant stream of
        heartbeat messages, so this is not for naught.
        """
        # Transition to APPROACH state when we're within sight of the target
        d = distance(self.target_local_position, self.vehicle.position)
        if self.target_is_in_sight and d < self.approach_radius:
            self.commander.transition_to_state(FlightState.APPROACH)

        # Construct an (x, y, z, yaw) setpoint, in local coordinates
        # NB: yaw currently has no effect (with ArduCopter, at least)
        x, y, z = self.target_local_position
        z += self.target_seek_altitude
        setpoint = (x, y, z, 0)

        # Send the vehicle on its merry way
        self.vehicle.set_location_setpoint(setpoint)


def distance(pt1, pt2):
    """
    Compute the Euclidean distance between two points.
    """
    try:
        x1, y1 = pt1.x, pt1.y
    except AttributeError:
        x1, y1, _ = pt1
    try:
        x2, y2 = pt2.x, pt2.y
    except AttributeError:
        x2, y2, _ = pt2
    return numpy.sqrt((x1-x2)**2 + (y1-y2)**2)
