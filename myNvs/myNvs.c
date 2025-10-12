#include <stdio.h>           
#include <string.h>          
#include <stdlib.h>          
#include "esp_log.h"         
#include "nvs_flash.h"       
#include "nvs.h"             
#include "myNvs.h"           
#include "WRCDefs.h"         

char nvs_log[NVS_LOG_SIZE];

void get_nvs_stats() {
    nvs_stats_t stats;
    esp_err_t err = nvs_get_stats("nvs", &stats);
    if (err == ESP_OK) {
        ESP_LOGI("NVS", "Total entries: %d", stats.total_entries);
        ESP_LOGI("NVS", "Used entries: %d", stats.used_entries);
        ESP_LOGI("NVS", "Free entries: %d", stats.free_entries);
    }
}

void print_nvs_raw_log() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS for reading: %s", esp_err_to_name(err));
        return;
    }

    size_t required_size = 0;
    // First, find how much space we need
    err = nvs_get_str(handle, "ascii_data", NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("NVS", "No log found in NVS.");
        nvs_close(handle);
        return;
    }
    else if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error reading log size: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    // Allocate buffer
    char* buffer = malloc(required_size);
    if (!buffer) {
        ESP_LOGE("NVS", "Failed to allocate memory for log");
        nvs_close(handle);
        return;
    }

    // Read the string
    err = nvs_get_str(handle, "ascii_data", buffer, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI("NVS_RAW", "\n===== RAW NVS LOG DUMP =====\n%s\n============================", buffer);
    }
    else {
        ESP_LOGE("NVS", "Failed to read log: %s", esp_err_to_name(err));
    }

    free(buffer);
    nvs_close(handle);
}

void read_nvs_log(char* out_buf, size_t buf_size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) return;

    size_t required_size = buf_size;
    err = nvs_get_str(handle, "ascii_data", out_buf, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI("NVS", "No existing log found");
        out_buf[0] = '\0';
    }
    nvs_close(handle);
    ESP_LOGI("NVS", "NVS Data: ");
    ESP_LOGI("NVS", "%s", nvs_log);
}

void print_nvs_log() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS for log read");
        return;
    }

    // Load log buffer
    char buffer[NVS_LOG_SIZE] = { 0 };
    size_t required_size = sizeof(buffer);
    err = nvs_get_str(handle, "ascii_data", buffer, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("NVS", "No log found");
        nvs_close(handle);
        return;
    }

    // Load write position
    uint32_t log_pos = 0;
    nvs_get_u32(handle, "log_pos", &log_pos);
    nvs_close(handle);

    size_t buf_len = strlen(buffer);

    ESP_LOGI("NVS", "Log length = %d, log_pos = %d", (int)buf_len, (int)log_pos);

    // Determine if wrapped
    bool wrapped = (buf_len == (NVS_LOG_SIZE - 1)) && (buffer[log_pos] != '\0');

    printf("=== NVS LOG START ===\n");
    if (!wrapped) {
        // No wrap: print entire buffer
        printf("%s\n", buffer);
    }
    else {
        // Wrapped: print from log_pos → end, then from 0 → log_pos-1
        printf("%s", &buffer[log_pos]);
        printf("%.*s\n", (int)log_pos, buffer);
    }
    printf("=== NVS LOG END ===\n");
}

void read_print_nvs_log(void)
{
    const char* TAG = "NVS_LOG";
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

void dump_all_nvs_entries_with_values(const char* partition_name) {
    const char* TAG = "NVS_DUMP";
    esp_err_t err;
    nvs_iterator_t it = NULL;
    nvs_entry_info_t info;

    err = nvs_entry_find(partition_name, NULL, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No entries found in partition '%s'", partition_name);
        return;
    }
    else if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_entry_find() failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "===== DUMPING ALL ENTRIES (WITH VALUES) FROM '%s' =====", partition_name);

    while (err == ESP_OK && it != NULL) {
        nvs_entry_info(it, &info);
        ESP_LOGI(TAG, "Namespace: '%s', Key: '%s', Type: %d", info.namespace_name, info.key, info.type);

        // Open namespace
        nvs_handle_t handle;
        if (nvs_open(info.namespace_name, NVS_READONLY, &handle) == ESP_OK) {
            switch (info.type) {
            case NVS_TYPE_U32: {
                uint32_t val = 0;
                if (nvs_get_u32(handle, info.key, &val) == ESP_OK)
                    ESP_LOGI(TAG, "  Value (u32): %u", val);
                break;
            }
            case NVS_TYPE_STR: {
                size_t size = 0;
                if (nvs_get_str(handle, info.key, NULL, &size) == ESP_OK) {
                    char* buf = malloc(size);
                    if (buf) {
                        if (nvs_get_str(handle, info.key, buf, &size) == ESP_OK)
                            ESP_LOGI(TAG, "  Value (str): \"%s\"", buf);
                        free(buf);
                    }
                }
                break;
            }
            default:
                ESP_LOGW(TAG, "  (Value print not implemented for type %d)", info.type);
                break;
            }
            nvs_close(handle);
        }
        // Advance to next entry
        err = nvs_entry_next(&it);
    }

    nvs_release_iterator(it);
    ESP_LOGI(TAG, "===== END OF DUMP =====");
}

void dump_all_nvs_entries(const char* partition_name) {
    const char* TAG = "NVS_DUMP";
    esp_err_t err;
    nvs_iterator_t it = NULL;
    nvs_entry_info_t info;

    // Start iterator for all namespaces and keys in this partition
    err = nvs_entry_find(partition_name, NULL, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No entries found in partition '%s'", partition_name);
        return;
    }
    else if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_entry_find() failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "===== DUMPING ALL NVS ENTRIES FROM PARTITION '%s' =====", partition_name);

    // Iterate through all entries
    while (err == ESP_OK && it != NULL) {
        nvs_entry_info(it, &info);
        ESP_LOGI(TAG, "Namespace: '%s', Key: '%s', Type: %d", info.namespace_name, info.key, info.type);

        // Advance iterator
        err = nvs_entry_next(&it);
    }

    nvs_release_iterator(it);
    ESP_LOGI(TAG, "===== END OF DUMP =====");
}

void append_nvs_log(const char* data) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS for log append");
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

    ESP_LOGI("NVS", "Appended %d bytes", (int)data_len);
}

void initNvsLog(bool erase) {
    esp_err_t err = nvs_flash_init();
    if (erase || err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

	read_print_nvs_log();
    append_nvs_log("\n......Start......\n");

}
