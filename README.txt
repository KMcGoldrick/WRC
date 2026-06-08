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
ESP-IDF 5.5 PowerShell
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

### Main loop and initialization loops

- Startup delay  
  - Single `vTaskDelay(pdMS_TO_TICKS(5000))` at startup to allow flashing/debug attach and for the TCM to power up.

- USB host / TCM init loops (blocking)  
  - `initUsbHostMode()` waits for the USB host to become ready (internal poll loop with timeout).  
  - `initTcm()` loops calling `connectDeviceUsb(...)` until the TCM is found or `TCM_CONNECT_TIMEOUT_MS` elapses; on failure the app enters a fatal error flash loop.

- Main runtime loop (`while (1)` in `app_main`) — per iteration:
  1. `runLED()` — update heartbeat / error LEDs (uses `millis()` / `esp_timer_get_time()` and refreshes pixels).  
  2. Read RS‑485 control bytes with `read_rs485_bytes(...)` (blocking up to its timeout).  
  3. Parse short commands that begin with `'['` and update runtime mode variables (`usbPort`, `four85Port`, `tcmDebug`, `tcmAverage`, `tcmDataSelect`, `tcmDataAsText`).  
  4. Configure logging routing: call `redirect_esp_log()` when `four85Port == DEBUG` (routes `ESP_LOG*` to RS‑485); otherwise set log levels with `esp_log_level_set`.  
  5. If `usbPort == TCM_COM` call `runTcm(...)`: read sensors via USB, optionally average, run computations (`calcTcm()`), and output via `tcmDataText()` or `tcmDataBinary()`. `runTcm()` returns a bool used to set `tcmProcessOk` (affects LEDs).  
  6. Else (loopback/debug) echo or log received RS‑485 data and sleep with `vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS))`.

- Important gotchas
  - The main loop performs blocking I/O (`read_rs485_bytes`, `getSensorsRawUsb` with retries) so timing is synchronous — heavy TX/RX can delay processing. Consider a dedicated RS‑485 TX task/queue for high throughput.  
  - When `usbPort == TCM_COM && four85Port == TCM_DATA` the code sets global logging to `ESP_LOG_NONE` to avoid contaminating RS‑485 output; `ESP_LOGE` messages are suppressed in that normal operational mode unless you enable `DEBUG` on the RS‑485 port.  
  - `uart_log_vprintf()` + `send_rs485_text()` may produce `\r\r\n` line ends (remove one terminator to clean output).

- Raw-value sanity check (recommended)  
  - Call `checkRawValuesNotStuck()` immediately after `getSensorsRawUsb(&tcmInfo.raw, SENSOR_READINGS_CMD)` inside `runTcm()` and treat a `false` return as a read error (skip output, log, retry).

- Quick improvement suggestions
  - Batch LED updates and call `led_strip_refresh()` once per toggle.  
  - Move RS‑485 TX to a dedicated task and use a queue to avoid blocking main logic.  
  - Expose a non‑blocking or callback driven USB path if responsiveness is required.

### Behavior of `while (tcmProcessOk)` inside the main loop

- Purpose
  - Runs the TCM processing continuously while the TCM subsystem is healthy; the loop keeps polling the TCM, performing calculations and streaming data over RS‑485.

- Typical actions per iteration
  - Update LED heartbeat (`runLED()`).
  - Read short RS‑485 control bytes and apply commands (mode, dataset, averaging, text/binary selection).
  - Call `runTcm(...)` which: reads raw sensors via USB (`getSensorsRawUsb`), optionally averages samples, runs `calcTcm()` computations, and outputs via `tcmDataText()` or `tcmDataBinary()`.
  - Use `runTcm()` return value to decide health: a `false` return should set `tcmProcessOk = false` and exit the inner loop.

- Timing
  - Loop rate is governed by blocking USB I/O and TCM response time; when in debug/loopback mode `MAIN_LOOP_RATE_MS` controls the delay.

- Error handling and recovery
  - On read/processing failure `runTcm()` returns `false`; the app currently sets `tcmProcessOk` accordingly and the LED indicates error.
  - Recommended improvements: add retry/backoff logic, an error counter, and an automatic reinitialization attempt (or escalate to a safe mode) rather than halting immediately.

- Sanity checks
  - Immediately after `getSensorsRawUsb(...)` call `checkRawValuesNotStuck()` to detect channels stuck at zero and treat failures as read errors.

- Performance recommendations
  - Move RS‑485 transmissions to a dedicated TX task with a queue to avoid blocking the TCM loop.
  - Batch LED updates and call `led_strip_refresh()` once per toggle to reduce timing impact.

- Summary
  - The `while (tcmProcessOk)` pattern keeps sensor acquisition and output running while healthy; ensure robust error detection, non‑blocking I/O for TX, and clear recovery strategies to improve reliability.

## Fallback and reinitialization strategy

When the TCM subsystem fails (for example `runTcm()` returns `false` repeatedly or USB/TCM initialization fails), the firmware currently uses an infinite error-flash loop (`while (1)`) to indicate a fatal state. For production devices it’s better to implement a controlled fallback and reinitialization strategy to increase uptime and recover from transient faults.

Recommended approach

- Detect vs. escalate
  - Treat a single `runTcm()` failure as transient: log it, increment a failure counter, and retry the read a few times before escalating.
  - After a configurable number of consecutive failures (e.g., 3–5), transition to a reinitialization state rather than staying in the normal running loop.

- Reinitialization sequence
  1. Switch the LED to a distinct reinit pattern (e.g., slow amber blink) so status is visible.
  2. Redirect logs to RS‑485 (or enable local logging) so reinit progress is observable: call `redirect_esp_log()` or `esp_log_level_set()` before reinit attempts.
  3. Gracefully close/reset interfaces used by the TCM driver where applicable (close CDC handle, flush UART buffers, set RS‑485 DE low).
  4. Attempt clean reinitialize of the USB host and TCM stack:
     - Use `usb_host_uninstall()` / `usb_host_install()` or library-specific reset/close APIs if available.
     - Re-run `initUsbHostMode()` and `initTcm()` with retries.
  5. Use exponential backoff between attempts (500 ms → 1 s → 2 s → 4 s) and cap total retry time (e.g., 30–60 s).
  6. If reinit succeeds, resume normal operation and clear failure counters. Optionally record recovery in NVS.

- Final escalation
  - If reinit repeatedly fails, choose one: call `esp_restart()` to reset the MCU; enter a low-power idle with occasional reinit attempts; or halt in an error loop for manual servicing.

- Implementation notes
  - Perform reinit in a separate FreeRTOS task or state machine so other subsystems (watchdog, RS‑485 parser, NVS logger) continue running.
  - Clean up driver resources before reinstall; if cleanup isn’t possible, trigger a watchdog reset for a full platform restart.
  - Keep logs enabled during reinit and remove extra CR/LF duplication for clean RS‑485 logs.
  - Use unique LED patterns for transient error, reinit in progress, and fatal/unrecoverable states.

- Observability
  - Log reinit attempts and outcomes to NVS and emit structured messages over RS‑485 so field diagnostics are possible.

Summary
- Replace permanent `while (1)` error loops with a structured reinit strategy (retry, backoff, cleanup, escalate) to improve reliability and recoverability in the field.
