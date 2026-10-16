import time
from utils import *
from control import *

# PARAMS
mode = Controller.LQR

def main():
    # connect to pendulum
    port = openPort()
    writeCurrent(port, 1500)  # write current to 1500mA so the stepper does not move
    writeReset(port)  # reset the position of the encoder and stepper in firmware
    time.sleep(0.1)  # wait for the reset to take effect

    start = time.perf_counter()
    tlast = 0.0
    enc_pos = 0.0
    enc_vel = 0.0
    vc = 0.0
    vc_last = 0.0
    try:
        while time.perf_counter() - start < 30:
            state = readState(port)
            if state is None:
                continue

            t_device, enc_pos_new, enc_vel_reported, stepper_pos = state
            t = time.perf_counter() - start  # host-side line arrival time
            dt = t - tlast

            if True:
                if dt > 0:
                    enc_vel = (enc_pos_new - enc_pos) / dt
                else:
                    enc_vel = 0
            else:
                enc_vel = enc_vel_reported

            enc_pos = enc_pos_new

            # control
            if mode == Controller.SINE:  # sine test
                vc = ctrl_sine(t)

            elif mode == Controller.PD:  # (manual tuning required)
                vc = ctrl_pd(t - tlast, [stepper_pos, enc_pos, enc_vel, vc_last])

            elif mode == Controller.LQR:  # LQR (tuned with MATLAB)
                dt_control = t - tlast
                vc = ctrl_lqr(dt_control, [stepper_pos, enc_pos, enc_vel, vc_last])

            # limit velocity and acceleration
            vc = vlim(vc, vc_last, 300, 3000, t - tlast)

            # update last values
            tlast = t
            vc_last = vc

            # write
            writeSpeed(port, vc)
            printState(t, enc_pos, enc_vel, vc, stepper_pos)

    except KeyboardInterrupt:
        # go home
        stepper_pos = 1.0
        while abs(stepper_pos) > 0.5:
            # TODO wrap theta state to [-180, 180] in firmware
            state = readState(port)
            if state is None:
                continue
            t_device, enc_pos, enc_vel, stepper_pos = state
            vc = -min(max(-80, stepper_pos * 16), 80)
            writeSpeed(port, vc)
            t = time.perf_counter() - start  # host-side line arrival time
            printState(t, enc_pos, enc_vel, vc, stepper_pos)
        writeSpeed(port, 0)

    writeSpeed(port, 0)
    writeCurrent(port, 50)
    port.close()


if __name__ == "__main__":
    main()
