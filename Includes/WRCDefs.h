// Hardware
#define LOW 0
#define HIGH 1
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define RGB_PIN 18
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define RGB_PIN 48
#endif
#define NUM_LEDS 8

// Memory
#define NVS_LOG_SIZE 1024

// Timing
#define LEDTime_ms 1000
#define STARTDelay_ms 1000 
#define MAIN_LOOP_RATE_MS 1000
#define USB_RESPONSE_DELAY_MS 3000
#define USB_HOST_INIT_DELAY_MS 2000
#define USB_DEVICE_CONNECT_DELAY_MS 1000

// Other
#define MAGIC 247
#define MAX_INT16 32767
#define ZERO_KELVIN -273.15
#define NUM_ITERATIONS_TO_AVERAGE 20
#define DECLINATION_DEG  -7.66f  // Magnetic declination for Grenville NC

// Lowell Instruments TCM
#define TCM_VID 0x08AE
#define TCM_PID 0x2047
#define FIRMWARE_VERSION_CMD   "GFV"
#define CALIBRATION_CMD        "RHS"
#define INTERVAL_TIME_CMD      "GIT"
#define LOGGER_INFO_CMD        "RLI"
#define LOGGER_SETTINGS_CMD    "GLS"
#define PAGE_COUNT_CMD         "GPC"
#define RESET_CMD              "RST"
#define RUN_CMD                "RUN"
#define SD_CAPACITY_CMD        "CTS"
#define SD_FILE_SIZE_CMD       "FSZ"
#define SD_FREE_SPACE_CMD      "CFS"
#define SENSOR_READINGS_CMD    "GSR"
#define SERIAL_NUMBER_CMD      "GSN"
#define START_TIME_CMD         "GST"
#define STATUS_CMD             "STS"
#define STOP_CMD               "STP"
#define SWS_CMD                "SWS"
#define RWS_CMD                "RWS"
#define SET_TIME_CMD           "STM"
#define TIME_CMD               "GTM"
#define DEL_FILE_CMD           "DEL"
#define LOGGER_INFO_CMD_W      "WLI"
#define LOGGER_HSA_CMD_W       "WHS"
#define REQ_FILE_NAME_CMD      "RFN"
#define DIR_CMD                "DIR"
