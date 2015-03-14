#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import time

import cv2
import numpy
import rospy

import geometry_msgs.msg
import sensor_msgs.msg

from lander.drivers.camera import OpenCVCamera, SimulatedCamera
from lander.lib.kalman import KalmanFilter
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

        self.tracking = False
        self.last_frame_time = 0
        self.last_seen_time = 0
        self.initialize_track_filter()

        self.image_publisher = rospy.Publisher("tracker/image",
                sensor_msgs.msg.Image, queue_size=1)

        self.track_publisher = rospy.Publisher("tracker/track",
                TrackStamped, queue_size=1)

    def initialize_track_filter(self, x=0, vx=0, y=0, vy=0, z=0, vz=0, p=10, q=1, r=1):
        """
        Construct a Kalman filter for a discretized continuous-time 3D kinematic model.

        From: Estimation with Applications to Tracking and Navigation (Bar-Shalom)
        """
        # Process model (position and velocity terms)
        F = lambda dt: numpy.matrix([
            [  1, dt,  0,  0,  0,  0  ],  # x' = x + vx*dt
            [  0,  1,  0,  0,  0,  0  ],  # vx' = vx
            [  0,  0,  1, dt,  0,  0  ],  # y' = y + vy*dt
            [  0,  0,  0,  1,  0,  0  ],  # vy' = vy
            [  0,  0,  0,  0,  1, dt  ],  # z' = z + vz*dt
            [  0,  0,  0,  0,  0,  1  ],  # vz' = vz
        ])

        # Control model (TODO)
        B = 0

        # Measurement model
        H = numpy.matrix([
            [  1, 0, 0, 0, 0, 0  ], # x
            [  0, 0, 1, 0, 0, 0  ], # y
            [  0, 0, 0, 0, 1, 0  ], # z
        ])

        # Initial state vector
        x0 = numpy.matrix([ x, vx, y, vy, z, vz ]).T

        # Initial state covariance
        P0 = numpy.matrix(numpy.eye(6)) * p

        # Process error covariance (continuous white noise acceleration model)
        Q = lambda dt: numpy.matrix([
            [  dt**3/3.0,  dt**2/2.0,          0,          0,          0,          0  ],
            [  dt**2/2.0,         dt,          0,          0,          0,          0  ],
            [          0,          0,  dt**3/3.0,  dt**2/2.0,          0,          0  ],
            [          0,          0,  dt**2/2.0,         dt,          0,          0  ],
            [          0,          0,          0,          0,  dt**3/3.0,  dt**2/2.0  ],
            [          0,          0,          0,          0,  dt**2/2.0,         dt  ],
        ]) * q

        # Measurement error covariance
        R = numpy.matrix(numpy.eye(3)) * r

        self.track_filter = KalmanFilter(F, B, H, x0, P0, Q, R)

    def publish_track(self, position, velocity):
        """
        Publish a TrackStamped message containing relative position
        (and eventually velocity) of the tracked object.
        """
        msg = TrackStamped()
        msg.track.tracking.data = self.tracking

        if self.tracking:
            msg.track.position.x = position[0]
            msg.track.position.y = position[1]
            msg.track.position.z = position[2]
            msg.track.velocity.x = velocity[0]
            msg.track.velocity.y = velocity[1]
            msg.track.velocity.z = velocity[2]

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
        now = time.time()

        # Compute the time step (dt) between frames
        dt = now - self.last_frame_time if self.last_frame_time else 0
        self.last_frame_time = now

        # Extract target position from the frame
        raw_position = self.detect_target(frame)

        # Determine whether we see the target now or whether we've seen it recently
        see_currently = raw_position is not None
        seen_recently = now - self.last_seen_time < 2

        if see_currently: self.last_seen_time = now

        # Reset filter if this is the first time we've seen the target recently
        if see_currently and not seen_recently and not self.tracking:
            self.initialize_track_filter()

        # Time update
        if self.tracking or (see_currently and seen_recently):
            self.track_filter.predict(dt=dt)

        # Measurement update
        if see_currently and (seen_recently or self.tracking):
            self.track_filter.update(raw_position)

        # Extract position and velocity from filter state vector
        position = self.track_filter.x[[0, 2, 4], 0].T.tolist()[0]
        velocity = self.track_filter.x[[1, 3, 5], 0].T.tolist()[0]

        # Determine whether we are certain about the target position
        certain = (self.track_filter.P < 2).all()

        if certain and not self.tracking:
            rospy.loginfo("Found target at (%6.4f, %6.4f, %6.4f)", *position)
        elif not certain and self.tracking:
            rospy.loginfo("Lost target")

        self.tracking = certain

        self.publish_track(position, velocity)

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

