from math import nan
import serial

def openPort(port_name="COM3", baud_rate=921600):
    return serial.Serial(port_name, baud_rate, timeout=5)

def writeSpeed(port, speed):
    port.write(f"s{speed}\n".encode())

def writeCurrent(port, current):
    port.write(f"c{current}\n".encode())

def writeReset(port):
    port.write(b"r\n")

def readState(port):
    line = port.readline().decode().strip()
    if not line:
        return None
    try:
        values = [float(value) for value in line.split(",")]
        if len(values) != 4:
            print(f"unknown line: {line}")
            return None

        return tuple(values)  # device time, encoder position, encoder velocity, stepper position
    except:
        return tuple([nan, nan, nan, nan])

def printState(t, enc_pos, enc_vel, vc, stepper_pos):
    print(
        f"t: {t:.3f}\tPe: {enc_pos:.3f}\tVe: {enc_vel:.3f}\tVsc: {vc:.3f}\tPs: {stepper_pos:.3f}"
    )

