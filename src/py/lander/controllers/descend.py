#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy
import rospy

from lander.lib.controller import Controller
from lander.lib.state import FlightState


# Maximum horizontal speed, in m/s
DEFAULT_MAX_SPEED_XY = 0.1

# Maximum horizontal acceleration, in m/s/s
DEFAULT_MAX_ACCEL_XY = 0.05

# Maximum vertical speed, in m/s
DEFAULT_MAX_SPEED_Z = 0.33

# Maximum vertical acceleration, in m/s/s
DEFAULT_MAX_ACCEL_Z = 0.1

DEFAULT_MAX_DESCEND_RADIUS = 0.25

DEFAULT_LAND_MAX_SPEED_XY = 0.05
DEFAULT_LAND_ALTITUDE = 0.5


class DescendController(Controller):
    """
    Descend toward the landing target.

    The general strategy here is to control vehicle velocity using a simple PID
    controller, using target position error as input, and constraining vertical
    velocity to within sane limits.
    """
    def __init__(self, *args, **kwargs):
        super(DescendController, self).__init__(*args, **kwargs)

        self.max_speed_xy = rospy.get_param("max_speed_xy", DEFAULT_MAX_SPEED_XY)
        self.max_accel_xy = rospy.get_param("max_accel_xy", DEFAULT_MAX_ACCEL_XY)
        self.max_speed_z = rospy.get_param("max_speed_z", DEFAULT_MAX_SPEED_Z)
        self.max_accel_z = rospy.get_param("max_accel_z", DEFAULT_MAX_ACCEL_Z)
        self.max_descend_radius = rospy.get_param("max_descend_radius", DEFAULT_MAX_DESCEND_RADIUS)
        self.land_max_speed_xy = rospy.get_param("land_max_speed_xy", DEFAULT_LAND_MAX_SPEED_XY)
        self.land_altitude = rospy.get_param("land_altitude", DEFAULT_LAND_ALTITUDE)

    def enter(self):
        self.setpoint = None

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
        err_z = tgt_p.z - veh_p.z

        distance = numpy.sqrt(err_x**2 + err_y**2)

        # Abort if we've moved out of the target radius
        # TODO: climb and attempt to reacquire the target
        # if distance > self.max_descend_radius:
        #     self.commander.relinquish_control()

        # Compute velocity setpoints
        # TODO: Implement I and D terms for full PID control
        Kpxy = 0.05
        Kpz = 0.15
        set_vx = Kpxy * err_x
        set_vy = Kpxy * err_y
        set_vz = Kpz * err_z

        # Enforce speed constraints
        speed = numpy.sqrt(set_vx**2 + set_vy**2)
        if speed > self.max_speed_xy:
            set_vx = set_vx * self.max_speed_xy / speed
            set_vy = set_vy * self.max_speed_xy / speed

        set_vz = max(set_vz, -self.max_speed_z)

        # TODO: Enforce acceleration constraints

        vehicle_is_right_above_target = veh_p.z < self.land_altitude
        vehicle_is_stable = speed < self.land_max_speed_xy

        # Transition to LAND state if:
        #   - the vehicle speed is within the approach speed threshold
        #   - those conditions have been true for the holddown period
        if vehicle_is_right_above_target and vehicle_is_stable:
            self.commander.transition_to_state(FlightState.LAND)
            return

        # Otherwise, control velocity to minimize position error
        self.setpoint = (set_vx, set_vy, set_vz, 0)

        rospy.logdebug("error: (%6.2f, %6.2f, %6.2f) setpoint: (%6.2f, %6.2f, %6.2f) " +
                       "speed: %6.2f  distance: %6.2f",
                        err_x, err_y, err_z, set_vx, set_vy, set_vz, speed, distance)

    def run(self):
        """
        Set the velocity setpoint once per control loop iteration.
        """
        if self.setpoint is not None:
            self.vehicle.set_velocity_setpoint(self.setpoint)
