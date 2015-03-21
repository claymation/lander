#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

from lander.states.base import State


class PendingState(State):
    """
    Do nothing pending landing sequence starting.
    """
    def run(self):
        pass
