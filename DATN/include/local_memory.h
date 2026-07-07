#ifndef LOCAL_MEMORY_H
#define LOCAL_MEMORY_H

#include <SPI.h>
#include <SD.h>

// Khai báo hàm khởi tạo thẻ SD
extern bool sd_init(uint8_t csPin);

// Khai báo hàm ghi dữ liệu CSV vào thẻ SD (yêu cầu 9 tham số)
extern bool sd_append_csv(uint32_t ts_ms, double lat, double lng, uint32_t sats, float temp, float hum, float ax, float ay, float az);

// Khai báo hàm ghi dữ liệu vào file text
extern bool sd_append_line(const char* filename, const char* data);

// Khai báo hàm flush (để ghi dữ liệu vào thẻ SD trước khi tắt hoặc reset)
extern void sd_flush();

// EEPROM Boot Counter
extern uint32_t increment_and_get_boot_counter();

#endif // LOCAL_MEMORY_H
