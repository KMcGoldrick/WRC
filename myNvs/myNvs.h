extern void get_nvs_stats();
extern void read_nvs_log(char* out_buf, size_t buf_size);
extern void append_nvs_log(const char* data);
extern void initNvsLog(bool erase);
extern void write_nvs_loop();