import math
import time
import matplotlib.pyplot as plt
from utils import *

def main():
    # connect to pendulum
    port = openPort()
    writeCurrent(port, 1500) # write current to 1500mA so the stepper does not move 
    writeReset(port) # reset the position of the encoder and stepper in firmware
    time.sleep(0.1)

    mode = 1  # 0 = sinusoidal, 1 = inverted stabilization
    if mode == 0:
        amplitude = 200  # deg/s
        frequency = 2     # Hz
    else:
        setpoint = 180

    start = time.perf_counter()
    tlast = 0.0
    enc_pos = 0.0
    enc_vel = 0.0
    errcum = 0.0
    t_read = []
    t_write = []

    while time.perf_counter() - start < 10:
        state = readState(port)
        if state is None:
            continue

        _, enc_pos_new, enc_vel_reported, _ = state
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
        else:
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

        writeSpeed(port, vc)
        t_write.append(t)

    writeSpeed(port, 0)
    writeCurrent(port, 250)
    port.close()

    plt.subplot(2, 1, 1)
    plt.hist([b - a for a, b in zip(t_read, t_read[1:])], bins=301)
    plt.subplot(2, 1, 2)
    plt.hist([b - a for a, b in zip(t_write, t_write[1:])], bins=301)
    plt.show()


if __name__ == "__main__":
    main()
