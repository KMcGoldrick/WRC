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

// Other
#define MAGIC 247
#define MAX_INT16 32767
#define ZERO_KELVIN -273.15
#define NUM_ITERATIONS_TO_AVERAGE 20
#define DECLINATION_DEG  -7.66f  // Magnetic declination for Grenville NC


