#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import numpy


class KalmanFilter(object):
    """
    Discretized continuous-time Kalman filter.
    """
    def __init__(self, F, B, H, x, P, Q, R):
        self.F = F  # state transition matrix
        self.B = B  # control model matrix
        self.H = H  # measurement model matrix
        self.x = x  # state vector
        self.P = P  # state covariance matrix
        self.Q = Q  # process error covariance matrix
        self.R = R  # measurement error covariance matrix
        self.I = numpy.eye(self.P.shape[0])

    def predict(self, u=0, dt=1):
        """
        Time update: predict future state with control u.
        """
        # Update model and error matrices for current dt
        F = result(self.F, dt)
        B = result(self.B, dt)
        Q = result(self.Q, dt)

        # Compute state prediction
        self.x = F * self.x + B * u

        # Incorporate process error
        self.P = (F * self.P) * F.T + Q

    def update(self, z):
        """
        Measurement update: incorporate measurement z.
        """
        # Compute estimate error (residual)
        y = z - self.H * self.x

        # Project into measurement space
        S = self.H * self.P * self.H.T + self.R

        # Compute Kalman gain
        K = self.P * self.H.T * S.I

        # Compute maximum likelihood state estimate
        self.x = self.x + K * y

        # Incorporate measurement error
        I_KH = self.I - K * self.H
        self.P = (I_KH * self.P) * I_KH.T + (K * self.R) * K.T


def result(f, *args, **kwargs):
    if callable(f):
        return f(*args, **kwargs)
    return f

