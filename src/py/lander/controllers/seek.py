#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy
import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState
from lander.lib.timers import HolddownTimer


# The landing target is located at the origin, i.e., where the vehicle's
# inertial navigation system was initialized (in meters from home)
DEFAULT_TARGET_LOCAL_POSITION = (0, 0, 0)

# The altitude at which to approach the target (in meters)
DEFAULT_TARGET_SEEK_ALTITUDE = 15

# The period of time we need to have continuously seen the target
# before transitioning to APPROACH state (in seconds)
DEFAULT_APPROACH_HOLDDOWN = 2.5

# The speed at which we'll transition to APPROACH state
# (assuming we can see the landing target) (in m/s)
DEFAULT_APPROACH_SPEED = 1.0

# The radius within which we'll transition to APPROACH state
# (assuming we can see the landing target) (in meters)
DEFAULT_APPROACH_RADIUS = 3


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
        self.approach_holddown = rospy.get_param("approach_holddown",
                DEFAULT_APPROACH_HOLDDOWN)
        self.approach_speed = rospy.get_param("approach_speed",
                DEFAULT_APPROACH_SPEED)
        self.approach_radius = rospy.get_param("approach_radius",
                DEFAULT_APPROACH_RADIUS)

        self.approach_holddown_timer = HolddownTimer(self.approach_holddown)

    def enter(self):
        self.approach_holddown_timer.reset()

    def handle_track_message(self, msg):
        """
        Implement control logic to correct for velocity and position errors.
        """
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
        speed = numpy.sqrt(veh_v.x**2 + veh_v.y**2)

        target_is_stable  = self.approach_holddown_timer.test(msg.track.tracking.data)
        target_is_close   = distance < self.approach_radius
        vehicle_is_stable = speed < self.approach_speed

        # Transition to APPROACH state if:
        #   - we've continuously seen the target for the past several seconds
        #   - we're within the approach radius of the target
        #   - the vehicle speed is within the approach speed threshold
        if target_is_stable and target_is_close and vehicle_is_stable:
            self.commander.transition_to_state(FlightState.APPROACH)
            return  # NB: Explicit return to prevent fall-through if we add code later

        rospy.logdebug("distance to target: %6.2f", distance)

    def run(self):
        """
        Publish location setpoint once per control loop iteration.

        For now, we have a static landing target position, so repeatedly setting
        the same location setpoint is like beating a dead horse like beating a
        dead horse like beating a dead horse like -- you get the idea.

        But eventually we'll receive dynamic updates of the target position, and
        then this will be more useful. Besides, PX4 requires a constant stream of
        heartbeat messages, so this is not for naught.
        """

        # Construct an (x, y, z, yaw) setpoint, in local coordinates
        # NB: yaw currently has no effect (with ArduCopter, at least)
        x, y, z = self.target_local_position
        z += self.target_seek_altitude
        setpoint = (x, y, z, 0)

        # Send the vehicle on its merry way
        self.vehicle.set_location_setpoint(setpoint)
