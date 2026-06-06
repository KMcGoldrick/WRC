# WRC (Water Resource Controller) Firmware

## Short description
Firmware for an ESP32-S3 based data-logger that reads a Lowell Instruments TCM sensor over USB CDC,
computes temperature, accelerometer, magnetometer, and roll/pitch/yaw, and streams output over RS485.

## Hardware variants

| Name          | Chip                | Flash | PSRAM        | Notes                        |
|---------------|---------------------|-------|--------------|------------------------------|
| Lonely Binary | ESP32-S3 QFN56 r0.2 | 16MB  | 8MB (AP_3v3) | No SPIRAM in config          |
| Tiny          | ESP32-S3 QFN56 r0.2 | 4MB   | 2MB (AP_3v3) | SPIRAM quad mode, 40MHz      |

## Features
- USB Host CDC connection to a TCM device (myUsb.c)
- Sensor acquisition, calibration and processing (myTcm.c)
- NVS logging utilities (myNvs.c)
- RGB LED status indicators (LED_MODEL_SK6812 via led_strip)
- RS485 binary and CSV text output with runtime-selectable dataset
- Runtime command interface over RS485 (see Commands section)

## Repository layout
- main/main.c         — application entry, LED handling, main loop
- myTcm/myTcm.c/.h   — TCM sensor interface, calculations, data output
- myUsb/myUsb.c/.h   — USB host, CDC handling, helper I/O
- myNvs/*            — persistent log (NVS) helpers
- Includes/WRCDefs.h — project-specific defines (pins, timings, TCM VID/PID, constants)

## Port configuration

Two ports are used. Set `usbPort` and `four85Port` in main.c to select the mode.

| usbPort  | four85Port | Behavior                                                                 |
|----------|------------|--------------------------------------------------------------------------|
| DEBUG    | any        | RS485 loopback. Bytes received on 485 are echoed back with loop counter. |
| TCM_COM  | TCM_DATA   | Operational mode. ESP32 reads TCM over USB, streams data over 485.       | Default
| TCM_COM  | DEBUG      | TCM algorithm debug. Human-readable sensor data sent over 485.           |

## RS485 runtime commands

Send commands from the PC over RS485 using the `[` prefix character.
Commands must be short (≤ 4 bytes). Binary frame data is automatically ignored.

| Command | Effect                          |
|---------|---------------------------------|
| `[t`    | Switch output to text (CSV)     | Default = binary Sets four85port to TCM_DATA
| `[b`    | Switch output to binary         | Default = binary Sets four85port to TCM_DATA 
| '[a'    | Toggle average mode on/off      | Default = off
| '[w'    | Enter 485 wrapback mode         | 
| '[d'    | Sets four85port to TCM_DEBUG    |
| `[0`–`[9` | Select dataset 0–9            |
| `[10`–`[11` | Select dataset 10–11        |

Average mode only averages datasets 1-5.
Average mode averages 20 samples before calculating values.
As a result, average mode only sends data every 20 iterations.

## Dataset reference

| ID | Content                              | Text columns                        | Binary bytes |
|----|--------------------------------------|-------------------------------------|--------------|
| 0  | Disabled                             | 0,0,0,0                             | 0            |
| 1  | Heading + velocity                   | heading, north, east                | 12           | Default
| 2  | Roll / Pitch / Yaw (radians)         | roll, pitch, yaw                    | 12           |
| 3  | Accelerometer raw + scaled           | rx,ry,rz, sx,sy,sz                  | 18           |
| 4  | Magnetometer raw + scaled            | rx,ry,rz, sx,sy,sz                  | 18           |
| 5  | Temperature + battery                | raw_t, temp_C, raw_b, batt_V        | 12           |
| 6  | Version + serial number              | version, serial                     | 24           |
| 7  | Temperature calibration (5 floats)   | TMO,TMR,TMA,TMB,TMC                 | 20           |
| 8  | Accel offsets + cubic (6 floats)     | ox,oy,oz, cx,cy,cz                  | 24           |
| 9  | Accel gain matrix (9 floats)         | 3x3 matrix row-major                | 36           |
| 10 | Mag soft + hard iron (12 floats)     | 3x3 soft-iron + 3 hard-iron         | 48           |
| 11 | Mag temp compensation (4 floats)     | tempRef, TMX, TMY, TMZ              | 16           |

Binary frame format: `0xAA | select | payload_len | payload bytes`

## Binary output — Python decoder example

```python
import serial, struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

while True:
    sof = ser.read(1)
    if sof != b'\xaa':
        continue
    select = ser.read(1)[0]
    length = ser.read(1)[0]
    payload = ser.read(length)

    if select == 1:
        heading, north, east = struct.unpack('<fff', payload)
        print(f"Heading: {heading:.2f}  N: {north:.3f}  E: {east:.3f}")
    elif select == 2:
        roll, pitch, yaw = struct.unpack('<fff', payload)
        print(f"Roll: {roll:.3f}  Pitch: {pitch:.3f}  Yaw: {yaw:.3f}")
    elif select == 5:
        raw_t, temp, raw_b, batt = struct.unpack('<HfHf', payload)
        print(f"Temp: {temp:.2f}C  Batt: {batt:.2f}V")
```

## LED behavior

- Startup sweep: red → green → blue (boot confirmation)
- Normal running: strip blinks green (heartbeat, TCM OK)
- TCM runtime error: strip solid red
- USB init failure: LED 0 flashes blue
- TCM init failure: LED 0 flashes red

Note: setPixelColor calls led_strip_refresh per pixel — not optimal for large strips.

## Timing

- Main Loop runs 1hz if in loopBack/debug mode
- When Main Loop is running TCM, loop rate is driven by TCM interaction time (it runs as fast as the TCM will respond)
- Average mode uses 20 samples (will run 20 times slower)
- Note: a "run" ends with outputting the data on RS485 

## Build & flash

Use standard ESP-IDF commands from the project root:
- Configure:  idf.py menuconfig
- Build:      idf.py build
- Flash:      idf.py flash
- Monitor:    idf.py monitor

## Configuration notes

- Port modes: set `usbPort` and `four85Port` in main.c
- Dataset and format: set `tcmDataSelect` (0–11) and `tcmDataAsText` (true/false) in main.c,
  or change them at runtime via RS485 commands
- Hardware constants and pin assignments: Includes/WRCDefs.h
- Logging level: set `logLevel` in main.c or via idf.py menuconfig

## Known limitations

- speedFromTilt() is not implemented — returns a placeholder value (MAGIC/10.0).
  All velocity (current.north, current.east) derived from speed is therefore invalid.
- setPixelColor calls led_strip_refresh on every pixel write; refactor to batch if
  strip length increases.
- Main loop is synchronous/blocking — large binary frames or slow RS485 can delay
  TCM polling. Consider a dedicated TX task if timing becomes critical.

## Troubleshooting

- LEDs always red: TCM runtime error or init failure. Check logs via idf.py monitor.
- Cannot switch from binary back to text: confirm command uses '[' prefix (e.g. "[t").
  Single bare bytes are unreliable when binary frames are in flight on the bus.
- USB / CDC errors: confirm TCM is powered, VID/PID in WRCDefs.h matches device.
  Expected: VID=0x2047 PID=0x08AE.
- No RS485 output: confirm usbPort and four85Port modes, check baud rate matches receiver.
- Junk at end of RS485 messages: ensure buffer is null-terminated after read and
  use rx_bytes count rather than %s formatting on binary buffers.

## Monitoring RS485 binary output

PuTTY will display binary frames as garbage characters. Use one of:
- Serial Studio (Win/Mac/Linux) — live hex + plot
- RealTerm (Windows) — raw hex display
- CoolTerm (Mac/Windows) — hex view mode
- Python decoder script (see above)
- PuTTY session logging to file, then inspect with a hex editor

PuTTY connection: COM3, baud per WRCDefs.h LOG_UART_BAUD. Use second red pot for level shifting.

## References
- ESP-IDF documentation for USB Host / CDC, led_strip, and logging backends
- Lowell Instruments TCM documentation for sensor commands and calibration format
- WRCDefs.h for all compile-time hardware settings and pinouts
