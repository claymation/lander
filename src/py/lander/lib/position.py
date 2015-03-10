import rospy

import geometry_msgs.msg


class PositionMixin:
    def __init__(self):
        # TODO: Select a more suitable initial value
        self.position = None
        self.orientation = None

        # Would be nice to use /mavros/position/local here, but ArduCopter doesn't
        # transmit the required POSITION_LOCAL_NED messages. Instead, mavros converts
        # the global coordinates passed via the POSITION_GLOBAL_INT message into UTM
        # coordinates, which can be used for local navigation assuming the drone
        # doesn't cross zone boundaries.
        rospy.Subscriber("/mavros/global_position/local",
                geometry_msgs.msg.PoseWithCovarianceStamped,
                self.handle_position_message)

    def handle_position_message(self, msg):
        """
        Handle mavros/global_position/local messages, published by the FCU at
        around 1 Hz.
        """
	pose = msg.pose.pose
        self.position = pose.position
        self.orientation = pose.orientation
