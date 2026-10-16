import math
import time
import matplotlib.pyplot as plt
from utils import *


def main():
    # connect to pendulum
    port = openPort()
    writeCurrent(port, 1500)  # write current to 1500mA so the stepper does not move
    writeReset(port)  # reset the position of the encoder and stepper in firmware
    time.sleep(0.1)
    start = time.perf_counter()
    tlast = 0.0

    mode = 2  # 0 = sinusoidal, 1 = inverted (PD control), 2 = LQR
    if mode == 0:
        # open loop sine input
        amplitude = 200  # deg/s
        frequency = 2  # Hz
    elif mode == 1:
        setpoint = 180
        errcum = 0.0
        kp = 16
        ki = 0.0
        kd = 0.03
    elif mode == 2:
        # for x = [stepPos, encPos, encVel, stepVelLast]
        setpoint = 180
        # K = [
        #     -1.0488,
        #     122.9508,
        #     10.1096,
        #     -60.619,
        # ]
        K = [-1.63257, 90.547, 19.2718, -64.3262]
        vc_last = 0

    enc_pos = 0.0
    enc_vel = 0.0
    try:
        while time.perf_counter() - start < 30:
            state = readState(port)
            if state is None:
                continue

            t_device, enc_pos_new, enc_vel_reported, stepper_pos = state
            t = time.perf_counter() - start  # host-side line arrival time
            dt = t - tlast
            if dt > 0:
                enc_vel = (enc_pos_new - enc_pos) / dt
                # enc_vel = enc_vel_reported
            else:
                enc_vel = 0
                # enc_vel = enc_vel_reported

            enc_pos = enc_pos_new

            if mode == 0:
                # sine test
                vc = amplitude * math.sin(2 * math.pi * frequency * t) * (t < 5)

            elif mode == 1:
                # PD control (manual tuning required)
                dt_control = t - tlast
                err = setpoint - enc_pos
                errcum += err * dt_control
                if abs(err) < 15:
                    vc = kp * err + ki * errcum + kd * -enc_vel
                else:
                    vc = 0
                tlast = t

            elif mode == 2:
                # LQR control (tuned with MATLAB (pen_5_inverted_control_optimization.m, pen_5_inverted_control.m))
                dt_control = t - tlast
                x_sp = [0, 180, 0, 0]
                err = [
                    x_sp[0] - stepper_pos,
                    x_sp[1] - enc_pos,
                    x_sp[2] - enc_vel,
                    x_sp[3] - vc_last,
                ]
                vc = 0
                if abs(enc_pos - x_sp[1]) < 25:
                    # raw command
                    vcc = (
                        K[0] * err[0] + K[1] * err[1] + K[2] * err[2] + K[3] * err[3]
                    ) / 57.2958

                    # limit acceleration
                    dv_max = 2000 * dt_control
                    dv = min(max(vcc - vc_last, -dv_max), dv_max)

                    # limit velocity
                    v_max = 400
                    vc = min(max(vc_last + dv, -v_max), v_max)
                    # vc = vcc
                    vc_last = vc
                    tlast = t
                else:
                    vc = 0

            # write
            writeSpeed(port, vc)

            print(
                f"t: {t:.3f}\tPe: {enc_pos:.3f}\tVe: {enc_vel:.3f}\tVsc: {vc:.3f}\tPs: {stepper_pos:.3f}"
            )

    except KeyboardInterrupt:
        writeSpeed(port, 0)
        time.sleep(0.1)
        stepper_pos = -100
        is_swinging = False
        while abs(stepper_pos) > 0.5 or is_swinging:
            # TODO wrap theta state to [-180, 180] in firmware
            state = readState(port)
            if state is None:
                continue
            _, enc_pos, enc_vel, stepper_pos = state
            vc = -min(max(-80, stepper_pos * 16), 80)
            writeSpeed(port, vc)
            t = time.perf_counter() - start  # host-side line arrival time
            print(
                f"t: {t:.3f}\tPe: {enc_pos:.3f}\tVe: {enc_vel:.3f}\tVsc: {vc:.3f}\tPs: {stepper_pos:.3f}"
            )
        writeSpeed(port, 0)

    writeSpeed(port, 0)
    writeCurrent(port, 50)
    port.close()


if __name__ == "__main__":
    main()
