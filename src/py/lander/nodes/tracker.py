#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import array

import cv2
import numpy
import rospy

import geometry_msgs.msg
import sensor_msgs.msg

from lander.drivers.camera import OpenCVCamera, SimulatedCamera
from lander.msg import TrackStamped


# Primary colors (BGR)
BLUE  = (255, 0, 0)
GREEN = (0, 255, 0)
RED   = (0, 0, 255)


class TrackerNode(object):
    """
    The Tracker node is responsible for vision-based tracking of the landing pad.
    """

    def __init__(self):
        """
        Initialize the camera and subscribe to mavros topics.
        """
        rospy.init_node("tracker")

        camera_matrix = numpy.matrix(rospy.get_param("~camera_matrix"))
        use_sim = rospy.get_param("~use_sim", False)

        if use_sim:
            self.camera = SimulatedCamera(camera_matrix)
            self.camera.set_target(
                    image_file=rospy.get_param("~target_image"),
                    position=rospy.get_param("~target_position"),
                    size_in_meters=rospy.get_param("~target_size"))
        else:
            self.camera = OpenCVCamera(camera_matrix)

        self.image_publisher = rospy.Publisher("tracker/image",
                sensor_msgs.msg.Image, queue_size=1)

        self.track_publisher = rospy.Publisher("tracker/track",
                TrackStamped, queue_size=1)

    def publish_track(self, tracking, position):
        """
        Publish a TrackStamped message containing relative position
        (and eventually velocity) of the tracked object.
        """
        msg = TrackStamped()
        msg.track.tracking.data = tracking

        if tracking:
            msg.track.position.x = position[0]
            msg.track.position.y = position[1]
            msg.track.position.z = position[2]

        self.track_publisher.publish(msg)

    def publish_image(self, image):
        """
        Construct and publish an Image message for the given image.
        """
        # TODO: Publish monochrome image + metadata to minimize telemetry bandwidth
        # TODO: Use ROS image_transport?
        msg = sensor_msgs.msg.Image()
        msg.height   = image.shape[0]
        msg.width    = image.shape[1]
        msg.encoding = "mono8" if image.ndim == 2 else "bgr8"
        msg.step     = msg.width if image.ndim == 2 else msg.width * 3
        msg.data     = image.reshape(-1).tolist()
        self.image_publisher.publish(msg)

    def process_frame(self, frame):
        """
        Find and track the landing pad.
        """
        position = self.detect_target(frame)

        # TODO: Filter position to determine velocity and uncertainty
        tracking = position is not None

        self.publish_track(tracking, position)

    def detect_target(self, frame):
        """
        Detect the landing pad target in the camera frame and compute its center
        in body-relative camera coordinates.

        Returns a (dx, dy, dz) tuple giving the body-relative location of the
        target center, in meters, or None if the target could not be found.
        """
        frame = cv2.GaussianBlur(frame, (5,5), 0)

        # NB: OpenCV's HoughCircles routine excludes concentric circles, so we
        #     can't do anything fancy with radii ratio to validate the signals.
        circles = cv2.HoughCircles(frame, cv2.cv.CV_HOUGH_GRADIENT, 1.5, 100)

        # Publish telemetry image
        self.publish_image(draw_circles(frame, circles))

        # Fail fast if we didn't detect any circles
        if circles is None or len(circles) == 0: return

        # Extract the pixel coordinates of the strongest circle center
        target_xy_pixels = circles[0,0,:2]

        # Back-project the pixel coordinates to body-relative camera coordinates
        target_xyz_camera = self.camera.back_project(*target_xy_pixels)

        return target_xyz_camera

    def wait_for_position(self):
        """
        Wait until we start receiving position messages.
        """
        while not rospy.is_shutdown() and self.camera.position is None:
            rospy.sleep(1)

    def run(self):
        """
        Capture frames from the camera and publish them.
        """
        self.wait_for_position()

        while not rospy.is_shutdown():
            frame = self.camera.get_frame()
            self.process_frame(frame)

        self.camera.release()


def draw_circles(image, circles):
    output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    if circles is not None:
        circles = numpy.round(circles[0, :]).astype("int")

        for x, y, r in circles:
            cv2.circle(output, (x, y), r, GREEN, 4)
            cv2.rectangle(output, (x - 3, y - 3), (x + 3, y + 3), RED, -1)

    return output


if __name__ == "__main__":
    node = TrackerNode()
    node.run()

