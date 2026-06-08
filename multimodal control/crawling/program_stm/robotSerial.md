# robotSerial.py

Purpose
- Keyboard teleoperation helper that sends packed binary commands over a serial port to robot hardware (solenoids, pump, step motions, LED).

Dependencies
- Python 3
- pyserial
- pynput

Install
```bash
pip install pyserial pynput
```

Default serial settings
- Device: `/dev/ttyAMA1`
- Baudrate: `115200`
- Message format: `struct.pack('bbbbf', cmd_type, cmd_id, param1, param2, param3)`

Run
```bash
python robotSerial.py
```
On Windows change the device string in the script to a COM port (for example `COM3`).

Keyboard mapping
- `l` — LED toggle (raw bytes)
- `w` — one step forward (left-first)
- `q` — one step forward (right-first)
- `s` — one step backward (left-first)
- `e` — one step backward (right-first)
- `a` — step left
- `d` — step right
- `1`–`4` — toggle solenoid valve channel 1..4
- `7` — trigger channels 1..4 (all)
- `8` — trigger channels 1 and 2
- `9` — trigger channels 3 and 4
- `0` — pump switch
- `Esc` — exit

Notes
- Some `struct.pack` calls use explicit little-endian format (`'<bbbbf'`) in the code — ensure the receiving firmware expects the same endianness.
- The script opens the serial port immediately; change the hardcoded string to parameterize the port for safer use.

Safety checklist
- Verify the correct serial device and baudrate before running.
- Test with hardware powered off or use a serial monitor to inspect bytes first.
- Have a physical emergency stop or power cutoff available.

Suggested improvements
- Add CLI args or environment variable to configure `serial_port` and `baudrate`.
- Add a `--dry-run` mode that prints packed bytes instead of writing to serial.
- Add logging and confirmation prompts for high-risk commands (pump/solenoids).

License
- This file follows the same licensing and distribution rules as the repository it belongs to.
