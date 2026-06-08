import serial
import time
from pynput.keyboard import Key, Listener
import struct

def on_press(key):
    try:
        if key.char == 'l':
            msg = bytes([1, 0, 0, 0, 0, 0, 0, 0])
            ser.write(msg)
            print("Key l pressed, LED switched")
        elif key.char == 'w':
            msg = struct.pack('bbbbf', 127, 1, 1, 0, 0.8)
            ser.write(msg)
            print("robot one step forward, left step first")
        elif key.char == 'q':
            msg = struct.pack('bbbbf', 127, 1, -1, 0, 0.8)
            ser.write(msg)
            print("robot one step forward, right step first")
        elif key.char == 's':
            msg = struct.pack('bbbbf', 127, -1, 1, 0, 0.8)
            ser.write(msg)
            print("robot one step backward, left step first")
        elif key.char == 'e':
            msg = struct.pack('bbbbf', 127, -1, -1, 0, 0.8)
            ser.write(msg)
            print("robot one step backward, right step first")
        elif key.char == 'a':
            msg = struct.pack('bbbbf', 127, 2, 1, 0, 0.5)
            ser.write(msg)
            print("robot one step left")
        elif key.char == 'd':
            msg = struct.pack('bbbbf', 127, 3, 1, 0, 0.5)
            ser.write(msg)
            print("robot one step right")
        elif key.char <= '4' and key.char >= '1':
            channel = int(key.char) - int('0')
            msg = struct.pack('bbbbf', 2, channel, 0, 0, 0)
            ser.write(msg)
            print(msg)
            print(f"Solenoid valve {channel} switched")
        elif key.char.lower() == '7':  # Use '7' key to trigger all channels
            for channel in range(1, 5):
                msg = struct.pack('<bbbbf', 2, channel, 0, 0, 0.0)
                ser.write(msg)
            print(f"All solenoid valves (1-4) switched")
        elif key.char.lower() == '8':  # Use '8' key to trigger channels 1+2
            # Switch channel 1
            msg = struct.pack('<bbbbf', 2, 1, 0, 0, 0.0)
            ser.write(msg)
            # Switch channel 2
            msg = struct.pack('<bbbbf', 2, 2, 0, 0, 0.0)
            ser.write(msg)
            print(f"Solenoid valves 1 and 2 switched simultaneously")
        elif key.char.lower() == '9':  # Use '9' key to trigger channels 
            # Switch channel 1
            msg = struct.pack('<bbbbf', 2, 3, 0, 0, 0.0)
            ser.write(msg)
            # Switch channel 2
            msg = struct.pack('<bbbbf', 2, 4, 0, 0, 0.0)
            ser.write(msg)
            print(f"Solenoid valves 3 and 4 switched simultaneously")         
        elif key.char == '0':
            msg = struct.pack('bbbbf', 3, 0, 0, 0, 0)
            ser.write(msg)
            print(msg)
            print(f"Pump switched")
    except AttributeError:
        if key == Key.esc:
            return False


def main():
    global ser
    ser = serial.Serial('/dev/ttyAMA1', 115200)
    if ser.is_open == False:
        ser.open()

    with Listener(on_press=on_press) as listener:
        listener.join()


if __name__ == '__main__':
    main()