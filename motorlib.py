# -*- coding: utf-8 -*-
#
# This file is part of the motorlib project
#
# Copyright (c) 2021 Tiago Coutinho
# Distributed under the GPLv3 license. See LICENSE for more info.

import math
import time

__version__ = "0.1.0"


def p_for(p0, v0, a, t):
    return p0 + v0 * t + 0.5 * a * t**2


def v_for(v0, a, t):
    return v0 + a * t


class JogTrajectory:
    """
    Trajectory representation for a jog motion

               v|
                |
            vel |....pa,ta______
                |        /
             vi |......./
                |_____________________> t
                      pi,ti
    """

    def __init__(self, pi, velocity, accel, vi=0, ti=None):
        if ti is None:
            ti = time.monotonic()
        self.ti = ti
        self.pi = pi = float(pi)
        self.vi = vi = float(vi)
        self.vel = velocity = float(velocity)
        self.positive = velocity >= 0
        accel = abs(float(accel))
        if velocity < vi:
            accel = -accel
        self.accel = accel

        accel_time = 0 if accel == 0 else (velocity - vi) / accel
        accel_dp = vi * accel_time + 0.5 * accel * accel_time ** 2

        self.accel_dp = accel_dp
        self.accel_time = accel_time
        self.ta = self.ti + self.accel_time
        self.pa = pi + self.accel_dp

    def finished(self, instant=None):
        """Tells if motion has finished at a given instant in time"""
        return False

    def velocity(self, instant=None):
        """Velocity at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant < self.ti:
            raise ValueError("instant cannot be less than start time")
        elif instant < self.ta:
            return self.vi + self.accel * (instant - self.ti)
        else:
            return self.vel

    def position(self, instant=None):
        """Position at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant < self.ti:
            raise ValueError("instant cannot be less than start time")
        dt = instant - self.ti
        if instant < self.ta:
            return self.pi + self.vi * dt + 0.5 * self.accel * dt ** 2

        return self.pi + self.accel_dp + self.vel * (dt - self.accel_time)


class LinearTrajectory(object):
    """
    Trajectory representation for a linear motion

               v|
                |
            vel |....pa,ta_________pb,tb
                |        /          \
                |_______/____________\_______> t
                      pi,ti         pf,tf
                        <--duration-->
    """

    def __init__(self, pi, pf, velocity, accel, ti=None):
        if ti is None:
            ti = time.monotonic()
        self.ti = ti
        self.pi = pi = float(pi)
        self.pf = pf = float(pf)
        self.vel = velocity = float(velocity)
        self.accel = accel = float(accel)
        self.p = pf - pi
        self.dp = abs(self.p)
        self.positive = pf > pi

        try:
            full_accel_time = velocity / accel
        except ZeroDivisionError:
            # piezo motors have 0 accel
            full_accel_time = 0
        full_accel_dp = 0.5 * accel * full_accel_time ** 2

        full_dp_non_const_vel = 2 * full_accel_dp
        self.reaches_top_vel = self.dp > full_dp_non_const_vel
        if self.reaches_top_vel:
            self.top_vel_dp = self.dp - full_dp_non_const_vel
            self.top_vel_time = self.top_vel_dp / velocity
            self.accel_dp = full_accel_dp
            self.accel_time = full_accel_time
            self.duration = self.top_vel_time + 2 * self.accel_time
            self.ta = self.ti + self.accel_time
            self.tb = self.ta + self.top_vel_time
            if self.positive:
                self.pa = pi + self.accel_dp
                self.pb = self.pa + self.top_vel_dp
            else:
                self.pa = pi - self.accel_dp
                self.pb = self.pa - self.top_vel_dp
        else:
            self.top_vel_dp = 0
            self.top_vel_time = 0
            self.accel_dp = self.dp / 2
            try:
                self.accel_time = math.sqrt(2 * self.accel_dp / accel)
            except ZeroDivisionError:
                self.accel_time = 0
            self.duration = 2 * self.accel_time
            self.vel = accel * self.accel_time
            self.ta = self.tb = self.ti + self.accel_time
            if self.positive:
                pa_pb = pi + self.accel_dp
            else:
                pa_pb = pi - self.accel_dp
            self.pa = self.pb = pa_pb
        self.tf = self.ti + self.duration

    def finished(self, instant=None):
        """Tells if motion has finished at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        return instant > self.tf

    def velocity(self, instant=None):
        """Velocity at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant < self.ti:
            raise ValueError("instant cannot be less than start time")
        elif instant > self.tf:
            return 0
        elif instant < self.ta:
            dt = instant - self.ta
            return self.accel * dt
        elif instant > self.tb:
            dt = instant - self.tb
            return -self.accel * dt + self.vel
        else:
            return self.vel

    def position(self, instant=None):
        """Position at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant < self.ti:
            raise ValueError("instant cannot be less than start time")
        if instant > self.tf:
            return self.pf
        dt = instant - self.ti
        p = self.pi
        f = 1 if self.positive else -1
        if instant < self.ta:
            accel_dp = 0.5 * self.accel * dt ** 2
            return p + f * accel_dp

        p += f * self.accel_dp

        # went through the initial accel
        if instant < self.tb:
            t_at_max = dt - self.accel_time
            dp_at_max = self.vel * t_at_max
            return p + f * dp_at_max
        else:
            dp_at_max = self.top_vel_dp
            decel_time = instant - self.tb
            decel_dp = 0.5 * self.accel * decel_time ** 2
            return p + f * dp_at_max + f * decel_dp

    def instant(self, position):
        """Instant when the trajectory passes at the given position"""
        d = position - self.pi
        dp = abs(d)
        if dp > self.dp:
            raise ValueError("position outside trajectory")

        dt = self.ti
        if dp > self.accel_dp:
            dt += self.accel_time
        else:
            return math.sqrt(2 * dp / self.accel) + dt

        top_vel_dp = dp - self.accel_dp
        if top_vel_dp > self.top_vel_dp:
            # starts deceleration
            dt += self.top_vel_time
            decel_dp = abs(position - self.pb)
            dt += math.sqrt(2 * decel_dp / self.accel)
        else:
            dt += top_vel_dp / self.vel
        return dt

    def __repr__(self):
        return "{0}({1.pi}, {1.pf}, {1.velocity}, {1.accel}, {1.ti})" \
            .format(type(self).__name__, self)


class StopTrajectory(object):
    """
       v|    vi
        |     |\
        |     | \
        |_____|__\___> t
          pi,ti   pf,tf
              <--->
             duration
    """
    def __init__(self, pi, vi, accel, ti=None):
        if ti is None:
            ti = time.monotonic()
        self.ti = ti
        self.pi = pi = float(pi)
        self.vi = vi = float(vi)
        self.positive = vi >= 0
        accel = abs(float(accel))
        if self.positive:
            accel = -accel
        self.accel = accel
        t = (0 - vi) / accel if accel != 0 else 0
        self.duration = t
        self.pf = pf = pi + vi * t + 0.5 * accel * t**2
        self.p = pf - pi
        self.dp = abs(self.p)
        self.tf = ti + t

    def finished(self, instant=None):
        """Tells if motion has finished at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        return instant > self.tf

    def velocity(self, instant=None):
        """Velocity at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant > self.tf:
            return 0
        dt = instant - self.ti
        return self.vi + self.accel * dt

    def position(self, instant=None):
        """Position at a given instant in time"""
        if instant is None:
            instant = time.monotonic()
        if instant < self.ti:
            raise ValueError("instant cannot be less than start time")
        if instant > self.tf:
            return self.pf
        dt = instant - self.ti
        return self.pi + self.vi * dt + 0.5 * self.accel * dt**2


class Motion(object):
    """Describe a single motion"""

    def __init__(self, pi, pf, velocity, accel, hard_limits, ti=None):

        # TODO: take hard limits into account (complicated).
        # For now just shorten the movement
        self.hard_limits = low_limit, high_limit = hard_limits
        if pf > high_limit:
            pf = high_limit
        if pf < low_limit:
            pf = low_limit
        self.trajectory = LinearTrajectory(pi, pf, velocity, accel, ti)

    def __getattr__(self, name):
        return getattr(self.trajectory, name)

    def stop(self):
        ti = time.monotonic()
        if self.finished(instant=ti):
            return
        pi = self.position(instant=ti)
        vi = self.velocity(instant=ti)
        if not self.positive:
            vi = -vi
        self.trajectory = StopTrajectory(pi, vi, self.accel, ti)


class Jog(object):
    """Describe a single motion"""

    def __init__(self, pi, velocity, accel, hard_limits, vi=0, ti=None):

        # TODO: take hard limits into account (complicated).
        self.hard_limits = low_limit, high_limit = hard_limits
        self.trajectory = JogTrajectory(pi, velocity, accel, vi, ti)

    def __getattr__(self, name):
        return getattr(self.trajectory, name)

    def stop(self):
        ti = time.monotonic()
        if self.finished(instant=ti):
            return
        pi = self.position(instant=ti)
        vi = self.velocity(instant=ti)
        if not self.positive:
            vi = -vi
        self.trajectory = StopTrajectory(pi, vi, self.accel, ti)
