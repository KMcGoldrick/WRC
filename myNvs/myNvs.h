#pragma once

#include <stdbool.h>    
#include "nvs_flash.h"
#include "nvs.h"

// ------------------------------
// NVS Logging Functions
// ------------------------------
void appendNvsLog(const char* data);
void initNvsLog(bool erase, bool printLog, int overall_log_level);
