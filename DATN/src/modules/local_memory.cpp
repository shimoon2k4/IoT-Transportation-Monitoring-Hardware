#include "local_memory.h"
#include <SPI.h>
#include <SD.h>
#include "FS.h"

static SPIClass sdSPI(VSPI);
static File     s_logFile;
static bool     s_ready = false;

static uint32_t s_lines = 0;
static const uint32_t SYNC_EVERY = 25;

// Tạo tên file /data_0001.csv, /data_0002.csv, ...
static String make_next_filename() {
  static int fileIndex = 1;  // Tạo một chỉ số file tự động tăng
  char name[20];
  snprintf(name, sizeof(name), "/data_%04d.csv", fileIndex++);
  return String(name);
}

static void write_header_if_new(File& f) {
  if (f && f.size() == 0) {
    f.println("ts_ms,lat,lng,sats,temperature,humidity,ax,ay,az");  // header cho log CSV
  }
}

// Khởi tạo SD
bool sd_init(uint8_t csPin) {
  // Bắt CS ở mức HIGH trước khi init
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  // VSPI pins: SCK=18, MISO=19, MOSI=23, CS=csPin
  sdSPI.begin(18, 19, 23, csPin);

  const uint32_t freqs[] = {10000000, 4000000};  // Thử với tần số thấp hơn (10MHz, 4MHz)
  bool ok = false;
  for (uint8_t i = 0; i < sizeof(freqs)/sizeof(freqs[0]); ++i) {
    Serial.printf("[SD] Trying freq %lu Hz...\n", (unsigned long)freqs[i]);
    ok = SD.begin(csPin, sdSPI, freqs[i]);
    if (ok) break;
  }
  if (!ok) {
    Serial.println("[SD] Init FAILED (check wiring, power, card format)");
    return false;
  }

  // Thông tin thẻ SD
  uint8_t type = SD.cardType();
  Serial.print("[SD] Card type: ");
  if (type == CARD_NONE) Serial.println("NONE");
  else if (type == CARD_MMC) Serial.println("MMC");
  else if (type == CARD_SD) Serial.println("SD");
  else if (type == CARD_SDHC) Serial.println("SDHC/SDXC");
  else Serial.println("UNKNOWN");

  uint64_t sizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("[SD] Card size: %llu MB\n", sizeMB);

  // Kiểm tra lỗi mở file
  String fname = make_next_filename();
  s_logFile = SD.open(fname, FILE_APPEND);
  if (!s_logFile) {
    Serial.println("[SD] Open log FAILED — cannot open file for writing");
    return false;
  }
  Serial.print("[SD] Logging to "); Serial.println(fname);

  // Đảm bảo viết header lần đầu tiên
  write_header_if_new(s_logFile);
  s_logFile.flush();

  s_ready = true;
  s_lines = 0;
  return true;
}

bool sd_append_csv(uint32_t ts_ms, double lat, double lng, uint32_t sats, float temp, float hum, float ax, float ay, float az) {
  if (!s_ready || !s_logFile) return false;

  // Ghi dữ liệu vào file CSV
  s_logFile.print(ts_ms); s_logFile.print(',');
  s_logFile.print(lat, 6); s_logFile.print(',');
  s_logFile.print(lng, 6); s_logFile.print(',');
  s_logFile.print(sats); s_logFile.print(',');
  s_logFile.print(temp); s_logFile.print(',');
  s_logFile.print(hum); s_logFile.print(',');
  s_logFile.print(ax); s_logFile.print(',');
  s_logFile.print(ay); s_logFile.print(',');
  s_logFile.println(az);

  if (++s_lines >= SYNC_EVERY) {
    s_logFile.flush();  // Đảm bảo dữ liệu được ghi vào thẻ SD sau mỗi `SYNC_EVERY` dòng
    s_lines = 0;
  }
  return true;
}

// Ghi dữ liệu vào file text
bool sd_append_line(const char* filename, const char* data) {
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.println(data);
    file.close();
    Serial.println("[SD] Data written successfully.");
    return true;
  } else {
    Serial.print("[SD] Failed to open file ");
    Serial.print(filename);
    Serial.println(" for writing.");
    // In chi tiết lỗi
    Serial.println("[SD] Error: Failed to write data.");
    return false;
  }
}

// Ép flush (trước khi reset/deep sleep)
void sd_flush() {
  if (s_logFile) s_logFile.flush();
}

#include <EEPROM.h>
#define EEPROM_SIZE 512
#define BOOT_COUNTER_ADDR 0

uint32_t increment_and_get_boot_counter() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("[EEPROM] Failed to initialize EEPROM");
    return 1;
  }
  uint32_t counter = 0;
  EEPROM.get(BOOT_COUNTER_ADDR, counter);
  if (counter == 0xFFFFFFFF) {
    counter = 0;
  }
  counter++;
  EEPROM.put(BOOT_COUNTER_ADDR, counter);
  EEPROM.commit();
  return counter;
}
