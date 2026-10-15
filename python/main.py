import math
import time
import matplotlib.pyplot as plt
from utils import *
import keyboard

def main():
    # connect to pendulum
    port = openPort()
    writeCurrent(port, 1500) # write current to 1500mA so the stepper does not move 
    writeReset(port) # reset the position of the encoder and stepper in firmware
    time.sleep(0.1)

    mode = 2  # 0 = sinusoidal, 1 = inverted (PD control), 2 = LQR
    if mode == 0:
        amplitude = 200  # deg/s
        frequency = 2     # Hz
    elif mode == 1:
        setpoint = 180
    elif mode == 2:
        setpoint = 180
        # K = [-1.0488, 122.9508, 10.1096, -60.619] # for x = [stepPos, encPos, encVel, stepVelLast]
        # K = [-2.67226,  47.3769,  2.38368, -58.4731]
        # K = [-1.632569,  101.0551,  19.27821, -67.99174]
        K = [-1.63257,   90.547,  19.2718, -64.3262]
        vc_last = 0

    start = time.perf_counter()
    tlast = 0.0
    enc_pos = 0.0
    enc_vel = 0.0
    errcum = 0.0
    t_read = []
    t_write = []

    while time.perf_counter() - start < 30:
        state = readState(port)
        if state is None:
            continue

        _, enc_pos_new, enc_vel_reported, stepper_pos = state
        t = time.perf_counter() - start  # host-side line arrival time
        if t_read:
            dt = t - t_read[-1]
            enc_vel = (enc_pos_new - enc_pos) / dt
        else:
            enc_vel = 0
            # enc_vel = enc_vel_reported

        enc_pos = enc_pos_new
        t_read.append(t)

        if mode == 0:
            vc = amplitude * math.sin(2 * math.pi * frequency * t) * (t < 5)
        elif mode == 1:
            dt_control = t - tlast
            err = setpoint - enc_pos
            errcum += err * dt_control
            if abs(err) < 15:
                kp = 16
                ki = 0.0
                kd = 0.03
                vc = kp * err + ki * errcum + kd * -enc_vel
            else:
                vc = 0
            tlast = t
            
        elif mode == 2:
            dt_control = t - tlast
            x_sp = [0, 180, 0, 0]
            err = [x_sp[0] - stepper_pos, x_sp[1] - enc_pos, x_sp[2] - enc_vel, x_sp[3] - vc_last]
            vc = 0
            if abs(enc_pos-x_sp[1]) < 25:
                # raw command
                vcc = (K[0] * err[0] + K[1] * err[1] + K[2] * err[2] + K[3] * err[3]) / 57.2958
                
                # limit acceleration
                dv_max = 8000 * dt_control
                dv = min(max(vcc - vc_last, -dv_max), dv_max)
                
                # limit velocity
                v_max = 600
                vc = min(max(vc_last + dv, -v_max), v_max)
            else:
                vc = 0
                
            vc_last = vc
            tlast = t
        # write
        writeSpeed(port, vc)
        t_write.append(t)
        print(f"t: {t:.3f}\tenc_pos: {enc_pos:.3f}\tenc_vel: {enc_vel:.3f}\tvc: {vc:.3f}\tstepper_pos: {stepper_pos:.3f}")
        
        # exit if 'q' is pressed
        if keyboard.is_pressed('q'):
            writeSpeed(port, 0)
            time.sleep(0.1)
            stepper_pos = -100
            is_swinging = True
            last_swing_time = time.perf_counter()
            while abs(stepper_pos) > 0.5 or is_swinging:
                #TODO wrap theta state to [-180, 180]
                state = readState(port)
                if state is None:
                    continue
                _, enc_pos, enc_vel, stepper_pos = state
                writeSpeed(port, -min(max(-80, stepper_pos*16), 80))
                t = time.perf_counter() - start  # host-side line arrival time
                print(f"t: {t:.3f}\tenc_pos: {enc_pos:.3f}\tenc_vel: {enc_vel:.3f}\tvc: {vc:.3f}\tstepper_pos: {stepper_pos:.3f}")
                if abs(enc_vel) > 50:
                    is_swinging = True
                    last_swing_time = time.perf_counter()
                elif abs(enc_vel) < 5 and time.perf_counter() - last_swing_time > 0.1:
                    is_swinging = False

            writeSpeed(port, 0)
            break

    writeSpeed(port, 0)
    writeCurrent(port, 250)
    port.close()

if __name__ == "__main__":
    main()
