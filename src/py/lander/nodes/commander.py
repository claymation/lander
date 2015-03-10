#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import mavros.msg

from lander import controllers
from lander.lib.vehicle import Vehicle


# Guided modes (differ between ArduCopter and PX4 native)
GUIDED_MODES = ("GUIDED", "OFFBOARD")


class FlightState:
    INIT    = "INIT"
    PENDING = "PENDING"
    SEEK    = "SEEK"
    DESCEND = "DESCEND"
    LAND    = "LAND"


class CommanderNode(object):
    """
    The Commander node is responsible for monitoring vehicle status
    and driving state transitions.
    """

    def __init__(self, vehicle):
        """
        Initialize the state machine subscribe to mavros topics.
        """
        rospy.init_node("commander")

        self.vehicle = vehicle

        # Initialize flight controllers
        self.controllers = {
            FlightState.DESCEND : controllers.DescendController(self, vehicle),
            FlightState.LAND    : controllers.LandController(self, vehicle),
            FlightState.PENDING : controllers.PendingController(self, vehicle),
            FlightState.SEEK    : controllers.SeekController(self, vehicle),
        }

        # Initialize state machine
        self.state = FlightState.INIT
        self.transition_to_state(FlightState.PENDING)

        # Run the control loop at 10 Hz
        self.control_loop_rate = rospy.Rate(10)

        rospy.Subscriber("/mavros/state", mavros.msg.State, self.handle_state_message)

    def handle_state_message(self, msg):
        """
        Handle mavros/State messages, published by the FCU at 1 Hz.

        When the FCU enters a guided mode, we begin the landing program by
        transitioning to SEEK state.

        When the FCU exits a guided mode, we terminate the landing program by
        transitioning to PENDING state.
        """
        mode = msg.mode

        if mode in GUIDED_MODES and self.state == "PENDING":
            self.transition_to_state("SEEK")
        elif mode not in GUIDED_MODES and self.state != "PENDING":
            self.transition_to_state("PENDING")

    def transition_to_state(self, new_state):
        """
        Enter the new state.
        """
        rospy.loginfo("STATE TRANSITION: %s --> %s", self.state, new_state)
        self.state = new_state
        self.controller = self.controllers[self.state]

    def run(self):
        """
        Spin the lander control loop and ROS event loop.

        Runs at 10 Hz.
        """
        while not rospy.is_shutdown():
            self.controller.run()
            self.control_loop_rate.sleep()


if __name__ == "__main__":
    vehicle = Vehicle()
    node = CommanderNode(vehicle)
    node.run()
