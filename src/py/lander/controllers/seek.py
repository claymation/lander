from .base import Controller


class SeekController(Controller):
    """
    Seek the landing target.
    """
    def run(self):
        # Hover above the target
        setpoint = (0, 0, 15, 0)
        self.vehicle.set_location_setpoint(setpoint)
