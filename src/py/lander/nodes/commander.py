#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import mavros.msg

from lander import states
from lander.lib.state import FlightState
from lander.lib.vehicle import Vehicle
from lander.msg import TrackStamped


# Guided modes (differ between ArduCopter and PX4 native)
GUIDED_MODES = ("GUIDED", "OFFBOARD")

# Run control loop at 10 Hz by default
DEFAULT_CONTROL_LOOP_RATE = 10


class CommanderNode(object):
    """
    The Commander node is responsible for monitoring vehicle status
    and driving state transitions.
    """
    def __init__(self, vehicle):
        """
        Initialize the state machine and subscribe to mavros topics.
        """
        rospy.init_node("commander")

        self.vehicle = vehicle

        # Initialize the control loop
        control_loop_rate = rospy.get_param("~control_loop_rate", DEFAULT_CONTROL_LOOP_RATE)
        self.control_loop_rate = rospy.Rate(control_loop_rate)

        # Initialize flight states
        self.states = {
            FlightState.PENDING  : states.PendingState(self, vehicle),
            FlightState.SEEK     : states.SeekState(self, vehicle),
            FlightState.APPROACH : states.ApproachState(self, vehicle),
            FlightState.DESCEND  : states.DescendState(self, vehicle),
            FlightState.LAND     : states.LandState(self, vehicle),
        }

        # Initialize state machine
        self.controller = None
        self.state = FlightState.INIT
        self.transition_to_state(FlightState.PENDING)

        rospy.Subscriber("/mavros/state", mavros.msg.State, self.handle_state_message)
        rospy.Subscriber("/tracker/track", TrackStamped, self.handle_track_message)

    def handle_state_message(self, msg):
        """
        Handle mavros/State messages, published by the FCU at 1 Hz.

        When the FCU enters a guided mode, we begin the landing program by
        transitioning to SEEK state.

        When the FCU exits a guided mode, we terminate the landing program by
        transitioning to PENDING state.
        """
        mode = msg.mode

        if mode in GUIDED_MODES and self.state == FlightState.PENDING:
            self.transition_to_state(FlightState.SEEK)
        elif mode not in GUIDED_MODES and self.state != FlightState.PENDING:
            self.transition_to_state(FlightState.PENDING)

    def handle_track_message(self, msg):
        """
        Forward tracker/TrackStamped messages to states.
        """
        self.controller.handle_track_message(msg)

    def transition_to_state(self, new_state):
        """
        Enter the new state.
        """
        rospy.loginfo("STATE TRANSITION: %s --> %s", self.state, new_state)

        if self.controller is not None:
            self.controller.exit()

        self.state = new_state
        self.controller = self.states[new_state]
        self.controller.enter()

    def relinquish_control(self):
        """
        Relinquish control of the vehicle.
        """
        rospy.loginfo("RELINQUISHING CONTROL")

        # Full stop
        self.vehicle.set_velocity_setpoint((0, 0, 0, 0))

        # Ask FCU to hold current position
        # NB: This is ArduPilot-specific
        self.vehicle.set_mode("POSHOLD")

        # Wait for FCU to change flight mode
        while self.state != FlightState.PENDING and not rospy.is_shutdown():
            self.control_loop_rate.sleep()

    def run(self):
        """
        Spin the ROS event loop, running the controller on each iteration.
        """
        while not rospy.is_shutdown():
            self.controller.run()
            self.control_loop_rate.sleep()


if __name__ == "__main__":
    vehicle = Vehicle()
    node = CommanderNode(vehicle)
    node.run()
