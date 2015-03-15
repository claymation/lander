#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState


# Maximum horizontal velocity, in m/s
DEFAULT_MAX_VEL_XY = 3.0

# Maximum horizontal acceleration, in m/s/s
DEFAULT_MAX_ACCEL_XY = 1.5


class ApproachController(Controller):
    """
    Approach the landing target while maintaining altitude.

    The general strategy here is to first minimize the velocity error,
    and then minimize position error.
    """
    def __init__(self, *args, **kwargs):
        super(ApproachController, self).__init__(*args, **kwargs)

        self.max_accel_xy = rospy.get_param("max_accel_xy", DEFAULT_MAX_ACCEL_XY)
        self.max_vel_xy = rospy.get_param("max_vel_xy", DEFAULT_MAX_VEL_XY)

    def handle_track_message(self, msg):
        """
        Implement control logic to correct for velocity and position errors.
        """
        # Abort if we're no longer tracking the target
        if not msg.track.tracking.data:
            self.vehicle.set_velocity_setpoint((0, 0, 0, 0))
            self.commander.transition_to_state(FlightState.SEEK)
            return

        # Velocity of vehicle
        # NB: ArduCopter's velocities sort of suck; fake it for now
        veh_v = self.vehicle.velocity

        # Position and velocity of target, relative to the vehicle
        rel_p = msg.track.position
        rel_v = msg.track.velocity

        # First, compute velocity setpoints to minimize velocity error
        set_vx = veh_v.x + rel_v.x
        set_vy = veh_v.y + rel_v.y

        # Then, add in a little velocity to minimize position error
        kP = 0.1
        set_vx += kP * rel_p.x
        set_vy += kP * rel_p.y

        # Enforce velocity constraints
        max_vx = max_vy = self.max_vel_xy
        set_vx = min(set_vx, max_vx) if set_vx > 0 else max(set_vx, -max_vx)
        set_vy = min(set_vy, max_vy) if set_vy > 0 else max(set_vy, -max_vy)

        # Enforce acceleration constraints
        # NB: Dividing by 30 to convert to m/s/frame
        max_ax = max_ay = self.max_accel_xy / 30.0

        if set_vx > veh_v.x:
            set_vx = min(set_vx, veh_v.x + max_ax)
        else:
            set_vx = max(set_vx, veh_v.x - max_ax)

        if set_vy > veh_v.y:
            set_vy = min(set_vy, veh_v.y + max_ay)
        else:
            set_vy = max(set_vy, veh_v.y - max_ay)

        print "veh_vx: %8.4f  rel_vx: %8.4f  rel_px: %8.4f  set_vx: %8.4f" % (veh_v.x, rel_v.x, rel_p.x, set_vx)
        print "veh_vy: %8.4f  rel_vy: %8.4f  rel_py: %8.4f  set_vy: %8.4f" % (veh_v.y, rel_v.y, rel_p.y, set_vy)
        print

        setpoint = (set_vx, set_vy, 0, 0)
        self.vehicle.set_velocity_setpoint(setpoint)
