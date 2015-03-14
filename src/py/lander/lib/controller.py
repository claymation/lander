#!/usr/bin/env python
# vim: set ts=4 sw=4 et:


class Controller(object):
    """
    Encapsulates the control logic for each stage of flight
    within its own controller class.
    """
    def __init__(self, commander, vehicle):
        self.commander = commander
        self.vehicle = vehicle

    def run(self):
        """
        Called by the commander once per control loop.
        """
        pass

    def handle_track_message(self, msg):
        """
        Called by the commander when it receives a TrackStamped message.
        """
        pass

