import math
from utils import *
from enum import Enum

class Controller(Enum):
    SINE = 0
    PD = 1
    LQR = 2

def ctrl_sine(t, x):
    amplitude = 200  # deg/s
    frequency = 2  # Hz
    return amplitude * math.sin(2 * math.pi * frequency * t) * (t < 5)


def ctrl_pd(dt, x):
    # for x = [stepPos, encPos, encVel, stepVelLast]
    enc_pos = x[1]
    enc_vel = x[2]

    setpoint = 180
    kp = 16
    kd = 0.03

    err = setpoint - enc_pos
    if abs(err) < 15:
        vc = kp * err + kd * -enc_vel
    else:
        vc = 0
    return vc


def vlim(v, vp, vmax, amax, dt):
    # limit acceleration
    dv_max = amax * dt
    dv = min(max(v - vp, -dv_max), dv_max)

    # limit velocity
    vc = min(max(vp + dv, -vmax), vmax)
    return vc


def ctrl_lqr(dt, x):
    # tuned with MATLAB (pen_5_inverted_control_optimization.m, pen_5_inverted_control.m)
    # for x = [stepPos, encPos, encVel, stepVelLast]
    # K = [-1.0488, 122.9508, 10.1096, -60.619]
    K = [-1.63257, 90.547, 19.2718, -64.3262]

    x_sp = [0, 180, 0, 0]
    err = [
        x_sp[0] - x[0],
        x_sp[1] - x[1],
        x_sp[2] - x[2],
        x_sp[3] - x[3],
    ]
    if abs(err[1]) < 25:
        # raw command
        return (K[0] * err[0] + K[1] * err[1] + K[2] * err[2] + K[3] * err[3]) / 57.2958
    else:
        return 0.0

