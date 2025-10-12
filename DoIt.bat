@echo off
REM Activate ESP-IDF environment
call C:\Espressif\frameworks\esp-idf-v5.5.1\export.bat

REM Navigate to your project folder
cd /d C:\Users\kevin\source\repos\WRC

REM Set target chip (only needed once, safe to repeat)
idf.py set-target esp32s2

REM Build the project
idf.py build

REM Flash to ESP32-S2
idf.py flash

REM Open serial monitor
idf.py monitor

REM To exit the monitor, press Ctrl+]	
