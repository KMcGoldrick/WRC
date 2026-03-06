# WRC (Water Resource Controller) Firmware

I (4458) USB: Attempting to connect device VID=0x2047 PID=0x08AE
I (4458) USB: Found 1 USB device(s)
I (4458) USB: VID=0x2047 PID=0x08AE device found at address 1
V (4458) USB: bNumInterfaces = 3
V (4458) USB: Interface 0: Class=0x08 SubClass=0x06 Protocol=0x50 NumEP=2
V (4468) USB: Interface 1: Class=0x02 SubClass=0x02 Protocol=0x01 NumEP=1
I (4468) USB: Attempting to open CDC interface 1
D (4478) cdc_acm: Checking list of opened USB devices
D (4478) cdc_acm: Checking list of connected USB devices
D (4488) cdc_acm: Submitting poll for BULK IN transfer
D (4488) cdc_acm: Submitting poll for INTR IN transfer
I (4498) USB: Opened CDC interface successfully
I (4498) USB: Device connected and switched to CDC-ACM
V (4508) USB: Sending command: GFV
V (4508) cdc_acm: Submitting BULK OUT transfer chunk: 4 bytes (remaining: 4)
D (4518) cdc_acm: out/ctrl xfer cb
D (4518) cdc_acm: in xfer cb
V (4518) USB: Data received len = 16
V (4528) USB: 0x3fc9c744   0a 0d 47 46 56 20 30 36  31 2e 39 2e 30 31 0d 0a  |..GFV 061.9.01..|
D (4528) cdc_acm: Submitting poll for BULK IN transfer
V (4538) USB: Sent command: GFV
I (4538) USB: getStrUsb: sent command 'GFV' (try 1/3)
I (4548) USB: getStrUsb: parsed 'GFV' = '1.9.01'
I (4548) TCM: TCM Version: 1.9.01



Purple 19
Grey 20 

Lonely Binary
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
SERIAL FLASHER CONFIG CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
# CONFIG_SPIRAM is not set

Tiny
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded Flash 4MB (XMC), Embedded PSRAM 2MB (AP_3v3)
Crystal is 40MHz USB mode: USB-Serial/JTAG
SERIAL FLASHER CONFIG CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_SPIRAM=y
	CONFIG_SPIRAM_MODE_QUAD=y
	CONFIG_SPIRAM_TYPE_AUTO=y
	CONFIG_SPIRAM_CLK_IO=30
	CONFIG_SPIRAM_CS_IO=26
	CONFIG_SPIRAM_SPEED_40M=y
	CONFIG_SPIRAM_SPEED=40
	CONFIG_SPIRAM_BOOT_HW_INIT=y
	CONFIG_SPIRAM_BOOT_INIT=y
	CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=y
	CONFIG_SPIRAM_USE_MALLOC=y
	CONFIG_SPIRAM_MEMTEST=y
	CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
	CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=32768
	CONFIG_ESP32S3_SPIRAM_SUPPORT=y
	CONFIG_DEFAULT_PSRAM_CLK_IO=30
	CONFIG_DEFAULT_PSRAM_CS_IO=26
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y

Short description
- Firmware for an ESP32-S2 / ESP32-S3 based data-logger that reads a TCM sensor over USB (CDC), 
  computes temperature/accel/mag/RPY, logs info, and provides simple LED status/serial-plot output.
- TCM is a Lowell TCM device (e.g. TCM2, TCM3) connected via USB CDC.	
- S

Features
- USB Host CDC connection to a TCM device (myUsb.c)
- Sensor acquisition, calibration and processing (myTcm.c)
- NVS logging utilities (myNvs.c)
- RGB LED status indicators (LED_MODEL_SK6812 via led_strip)
- Simple serial plotting output (tcmPlot) and configurable logging via ESP-IDF logging API

Repository layout
- main/main.c         — application entry, LED handling, main loop
- myTcm/myTcm.c/.h    — TCM sensor interface, calculations, tcmPlot
- myUsb/myUsb.c/.h    — USB host, CDC handling, helper I/O
- myNvs/*             — persistent log (NVS) helpers
- Includes/WRCDefs.h  — project-specific defines (pins, timings, TCM VID/PID, constants)

Requirements
- ESP-IDF (compatible version for chosen target)
- Toolchain for ESP32-S2 or ESP32-S3
- USB host capable board (ESP32-S2/S3) and the TCM device or a USB-serial adapter for UART

Build & flash
- From project root use the standard ESP-IDF commands:
  - Configure: __idf.py menuconfig__
  - Build: __idf.py build__
  - Flash: __idf.py flash__
  - Monitor: __idf.py monitor__
- Use __idf.py menuconfig__ to change default console baud, logging backend and other platform settings.

Configuration notes
- Logging level: change `overall_log_level` in main/main.c or set default via __idf.py menuconfig__.
- Hardware constants and timings are in Includes/WRCDefs.h (NUM_LEDS, RGB_PIN, LED_TIME_MS, STARTUP_DELAY_MS, MAIN_LOOP_RATE_MS, TCM_VID, TCM_PID, etc.).
- Serial plotting: control which data are emitted by changing the global `serial_plot` (see myTcm.c:tcmPlot). `plotting_all_loops` controls whether tcmPlot runs on every loop or only after averaging.

LED behavior (what the user sees)
- Startup sweep: sequenceLED cycles pixels red → green → blue (visual boot confirmation).
- Normal running: whole strip blinks green (heartbeat) when TCM processing is OK.
- Runtime TCM error: whole strip shows steady red.
- Fatal init errors:
  - USB init failure: LED 0 flashes red repeatedly.
  - TCM init failure: LED 0 flashes yellow repeatedly.
Notes: setPixelColor currently calls led_strip_refresh per pixel (simple, but not optimal for large strips).

Serial / plotting
- tcmPlot() prints ASCII lines using printf — these go to the UART console (UART0) by default and are suitable for PC tools like SerialPlot when you connect the board UART to the PC via a USB-to-UART adapter.
- If you need the data on the same channel as `ESP_LOG*` output you can use the logging API (ESP_LOGI/ESP_LOGV), but ESP_LOG output includes prefixes (level, time, tag). For raw "v1 v2 v3" lines (preferred by SerialPlot) keep using printf or retarget stdout to a CDC device.
- If you expect to stream directly to a PC over USB (so printf appears as a COM/TTY on the PC), use the USB Device CDC driver (TinyUSB) and retarget stdout — this project currently uses USB Host (talks to a TCM peripheral), so sending to the PC via that same host link is not the default behavior.
- Use PuTTY

Troubleshooting
- LEDs always red: indicates TCM runtime not OK or initialization failed; inspect logs via serial console (use __idf.py monitor__).
- No serial plot data: ensure you are connected to the board UART (or retargeted USB CDC), confirm `serial_plot` ≠ 0 and `plotting_all_loops` settings.
- USB / CDC errors: confirm the TCM device is powered and VID/PID in WRCDefs.h match the device; check myUsb logs (TAG = "USB").
- High update rate causing timing issues: sending large amounts of blocking USB/UART data from the main loop can block processing — consider a dedicated send task or message queue.

Extending the project
- Optimize LED writes by batching pixel updates and calling led_strip_refresh once.
- Add a USB Device CDC (TinyUSB) configuration to present the ESP as a serial device to a host PC and retarget stdout.
- Implement non-blocking or queued serial sends to avoid blocking the main loop.
- Add a README section for calibration file formats, tilt-curve integration and RS485 routing if those features are added.

Contacts / references
- ESP-IDF documentation: refer to your installed ESP-IDF docs for USB Host / CDC, led_strip and logging backends.
- Inspect WRCDefs.h for compile-time settings and hardware pinouts.
