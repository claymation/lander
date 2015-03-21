#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

from lander.states.base import State


class LandState(State):
    """
    Land on the landing target.

    Landing is tricky. Ground effect causes random perturbations in
    vehicle velocity, and we can't correct for that very well because
    we're so close to the ground we don't have much room to maneuver
    or margin for error.

    So we just ask the FCU to land and then disarm the motors.
    """
    def enter(self):
        self.vehicle.set_mode("LAND")
