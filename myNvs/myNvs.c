#include <stdio.h>           
#include <string.h>          
#include <stdlib.h>          
#include "esp_log.h"         
#include "nvs_flash.h"       
#include "nvs.h"             
#include "myNvs.h"           
#include "WRCDefs.h"  

#define TAG "myNvs"

char nvs_log[NVS_LOG_SIZE];

void getNvsStats() {
    nvs_stats_t stats;
    esp_err_t err = nvs_get_stats("nvs", &stats);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Total entries: %d", stats.total_entries);
        ESP_LOGI(TAG, "Used entries: %d", stats.used_entries);
        ESP_LOGI(TAG, "Free entries: %d", stats.free_entries);
    }
}

void printNvsLog(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Open NVS in read-only mode
    err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS (err=0x%x)", err);
        return;
    }

    // Read log data
    char buffer[NVS_LOG_SIZE] = { 0 };
    size_t required_size = sizeof(buffer);
    err = nvs_get_str(handle, "ascii_data", buffer, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No log found in NVS");
        nvs_close(handle);
        return;
    }
    else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ascii_data (err=0x%x)", err);
        nvs_close(handle);
        return;
    }

    // Read log position
    uint32_t log_pos = 0;
    err = nvs_get_u32(handle, "log_pos", &log_pos);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to read log_pos (err=0x%x)", err);
        log_pos = 0;
    }

    nvs_close(handle);

    size_t buf_len = strlen(buffer);
    ESP_LOGI(TAG, "Log length = %d, log_pos = %d", (int)buf_len, (int)log_pos);

    // Detect wrap (if your code wraps the buffer circularly)
    bool wrapped = (buf_len == (NVS_LOG_SIZE - 1)) && (log_pos < buf_len);

    printf("=== NVS LOG START ===\n");
    if (!wrapped) {
        // No wrap: just print buffer
        printf("%s\n", buffer);
    }
    else {
        // Wrapped: print from log_pos to end, then from start to log_pos
        printf("%s", &buffer[log_pos]);
        printf("%.*s\n", (int)log_pos, buffer);
    }
    printf("=== NVS LOG END ===\n");
}

void appendNvsLog(const char* data) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for log append");
        return;
    }

    // Load current log and position
    char buffer[NVS_LOG_SIZE] = { 0 };
    size_t required_size = sizeof(buffer);
    err = nvs_get_str(handle, "ascii_data", buffer, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) buffer[0] = '\0';

    uint32_t log_pos = 0;
    err = nvs_get_u32(handle, "log_pos", &log_pos);
    if (err == ESP_ERR_NVS_NOT_FOUND) log_pos = 0;

    size_t data_len = strlen(data);
    if (data_len >= NVS_LOG_SIZE) data_len = NVS_LOG_SIZE - 1;

    for (size_t i = 0; i < data_len; ++i) {
        buffer[log_pos] = data[i];
        log_pos = (log_pos + 1) % (NVS_LOG_SIZE - 1);
    }
    buffer[log_pos] = '\0';

    // Save
    nvs_set_str(handle, "ascii_data", buffer);
    nvs_set_u32(handle, "log_pos", log_pos);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Appended %d bytes", (int)data_len);
}

void initNvsLog(bool erase, bool printLog) {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_err_t err = nvs_flash_init();
    if (erase || err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
	getNvsStats();

    if (printLog) {
        printNvsLog();
    }

}
