class Controller(object):
    """
    Encapsulates the control logic for each stage of flight
    within its own controller class.
    """
    def __init__(self, commander, vehicle):
        self.commander = commander
        self.vehicle = vehicle

    def run(self):
        raise NotImplementedError
