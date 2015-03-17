#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import time


class HolddownTimer(object):
    """
    Boolean wrapper that requires a condition to have been True
    for a pre-defined period before evaluating as True.

    Params:
        - period: holddown time (in seconds)
    """
    def __init__(self, period):
        self.period = period
        self.reset()

    def reset(self):
        self.state_changed_at = 0
        self.last_state = False
    
    def test(self, state):
        """
        Test whether the given state has been True for the holddown period.
        """
        now = time.time()

        if state and self.last_state:
            return now - self.state_changed_at >= self.period

        self.last_state = state
        self.state_changed_at = now

        return False

