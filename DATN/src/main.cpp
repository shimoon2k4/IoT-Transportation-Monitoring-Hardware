#include <Arduino.h>
#include "security.h"

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <esp_system.h>   // For chip ID functions

#include "gps.h"
#include "dht11.h"
#include "adxl345.h"
#include "tamper.h"
#include "battery.h"
#include "vehicle_config.h"
#include "sensor_Data.h"

#pragma pack(push, 1)
struct LoRaFrameHeader {
    uint8_t magic[2];      // 0x25, 0x03
    uint8_t device_id;     // 1 - 100
    uint8_t vehicle_id;    // 1 - 100
    uint16_t packet_id;    // sequence number (anti-replay)
    uint32_t timestamp;    // millis or epoch
    uint8_t type;          // 1 = Telemetry, 2 = Alert/Tamper, 3 = Heartbeat
    uint8_t flags;         // Bitmask: Bit 0=Tamper, Bit 1=GPS Fix, Bit 2=Vibration, Bit 3=Low Batt
};

struct LoRaFramePayload {
    int16_t temperature;    // temp * 10
    int16_t humidity;       // hum * 10
    int16_t accel_mag;      // accel * 100
    uint16_t light_level;   // LDR (0 - 1023)
    int32_t latitude;       // lat * 1e6
    int32_t longitude;      // lng * 1e6
};

struct LoRaFrameFooter {
    uint16_t auth_tag;      // Authentication tag
    uint16_t crc;           // CRC16 over Header + Payload + Auth_Tag
};

struct LoRaFrame {
    LoRaFrameHeader header;
    LoRaFramePayload payload; // to be encrypted in-place (16 bytes)
    LoRaFrameFooter footer;
};
#pragma pack(pop)

// ===== Pins / Config =====
#define DHTPIN      14
#define DHTTYPE     DHT11
#define LED_PIN     2
#define TAMPER_PIN  27
#define BUZZER_PIN  13
#define BAT_ADC_PIN 34     

#ifndef VEHICLE_DEVICE_ID
  #define VEHICLE_DEVICE_ID "VX"
#endif

// GPS: UART2 RX=16, TX=17 @ 9600
GPSNeo6M  gps(16, 17, 9600);
DHTModule dht(DHTPIN, DHTTYPE);
ADXLModule adxl; // ADXL345
TamperModule tamper(TAMPER_PIN, BUZZER_PIN, LED_PIN);
BatteryModule battery(BAT_ADC_PIN); 

// --- LoRa UART (RA-08H TX connected here) on UART1 ---
#define LORA_RX   25     // ESP32 RX1  <= TX of RA-08H
#define LORA_TX   26     // ESP32 TX1  => RX of RA-08H
#define LORA_BAUD 115200 // match pingpong_tx UART baud

HardwareSerial LORA_SER(1); 

volatile uint32_t g_send_interval_ms = 2000; 
volatile bool g_tamper_alert = false;        

#define FORCE_NODE_ID 1  // Set to 1 for Transport-1, 2 for Transport-2, etc. | 0 = auto-increment from EEPROM

void detectOrGenerateNodeId(HardwareSerial &lora_uart) {
  (void)lora_uart; 
  
  if (FORCE_NODE_ID > 0 && FORCE_NODE_ID <= 100) {
    Serial.printf("[SYNC] FORCE_NODE_ID enabled - Using Transport-%d (override mode)\r\n", FORCE_NODE_ID);
    gVehicleConfig.setDeviceIdFromNodeId(FORCE_NODE_ID);
    return;
  }
  
  Serial.println("[SYNC] Auto-assigning node_id from EEPROM counter...");
  
  const uint8_t MAGIC_BYTE = 0xAA;
  uint8_t magic = EEPROM.read(1);
  
  if (magic != MAGIC_BYTE) {
    Serial.println("[SYNC] Old EEPROM detected - resetting counter to 0");
    EEPROM.write(0, 0);
    EEPROM.write(1, MAGIC_BYTE);
    EEPROM.commit();
  }
  
  uint8_t counter = EEPROM.read(0);
  
  if (counter == 0 || counter == 0xFF) {
    counter = 1;
  }
  
  uint8_t node_id = counter;
  
  // Increment counter for next device
  counter++;
  if (counter > 100) counter = 1;  // Wrap around after 100
  
  // Save incremented counter for next device
  EEPROM.write(0, counter);
  EEPROM.write(1, MAGIC_BYTE);  // Keep magic byte
  EEPROM.commit();
  
  Serial.printf("[SYNC] Assigned node_id = %u (Transport-%u)\r\n", node_id, node_id);
  Serial.printf("[SYNC] Next device will get node_id = %u\r\n", counter);
  
  gVehicleConfig.setDeviceIdFromNodeId(node_id);
}

static uint32_t getVehicleLoraSendDelayMs() {
  uint8_t veh_num = gVehicleConfig.getVehicleNumber();
  const uint32_t slot_ms = 250;
  static const uint8_t slot_order[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };

  if (veh_num == 0) {
    return 0;
  }

  uint8_t slot_index = (veh_num - 1) % 8;
  return slot_order[slot_index] * slot_ms;
}

// --- FreeRTOS Task: GPS Reader ---
void TaskGPS(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);

  for (;;) {
    gps.read();

    if (gps.updated()) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensorData.lat = gps.latitude();
        sensorData.lng = gps.longitude();
        sensorData.sats = gps.satellites();
        sensorData.speed = gps.gpsObject().speed.kmph();
        xSemaphoreGive(sensorDataMutex);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void TaskTamperMonitor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);

  for (;;) {
    // SW2 reads HIGH if box is opened (switch released)
    bool open_state = tamper.isTriggered();

    if (open_state) {
      if (!g_tamper_alert) {
        g_tamper_alert = true;
        Serial.println("[TAMPER ALERT] BOX OPENED!");
      }
      tamper.checkTamper(); // Set tamper_detected = true
      tamper.updateAlarm(true);
    } else {
      g_tamper_alert = false;
      tamper.resetTamper(); // Clear tamper state when closed
      tamper.updateAlarm(false);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void TaskLoraSend(void *pv) {
  delay(100);
  while (LORA_SER.available()) LORA_SER.read();

  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t send_delay = getVehicleLoraSendDelayMs();
  if (send_delay > 0) {
    Serial.printf("[SYNC] Slot offset: %lums\r\n", (unsigned long)send_delay);
    vTaskDelay(pdMS_TO_TICKS(send_delay));
    xLastWakeTime = xTaskGetTickCount();
  }

  const TickType_t xInterval = pdMS_TO_TICKS(g_send_interval_ms);
  static uint16_t packet_seq = 0;

  for (;;) {
    SensorData localData = {}; 

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      localData = sensorData;   // snapshot toàn bộ struct
      xSemaphoreGive(sensorDataMutex);
    }

    // extract ra biến local
    float temp = localData.temp;
    float hum = localData.hum;
    float accel_g = localData.accel;
    double lat = localData.lat;
    double lng = localData.lng;
    uint32_t sats = localData.sats;

    if (isnan(temp) || temp < -100 || temp > 150) {
      Serial.println("[DHT11-ERROR] Sensor read failed (no valid data)");
      temp = -999.0f;
    }
    if (isnan(hum) || hum < 0 || hum > 100) {
      hum = -999.0f;
    }
    if (accel_g < -900.0f) {
        Serial.println("[ADXL345-ERROR] Sensor data unavailable");
        accel_g = -999.0f;
    }

    // Read battery voltage in millivolts (e.g. 3850 mV)
    uint16_t battery_mv = battery.readVoltagemV();
    // Check tamper state from SW2
    bool is_tamper = tamper.getTamperState();

    uint32_t ts = millis();

    // Fill the binary frame
    LoRaFrame frame;
    memset(&frame, 0, sizeof(frame));

    frame.header.magic[0] = 0x25;
    frame.header.magic[1] = 0x03;
    frame.header.device_id = gVehicleConfig.getVehicleNumber();
    frame.header.vehicle_id = gVehicleConfig.getVehicleNumber();
    frame.header.packet_id = ++packet_seq;
    frame.header.timestamp = ts;
    frame.header.type = 1; // Telemetry
    
    // Set Flags bitmask
    frame.header.flags = 0;
    if (is_tamper) {
      frame.header.flags |= (1 << 0);
    }
    if (lat != 0.0 && lng != 0.0) {
      frame.header.flags |= (1 << 1); // GPS fix flag
    }
    if (accel_g > 1.2f || accel_g < -1.2f) { // simple vibration threshold
      frame.header.flags |= (1 << 2);
    }
    if (battery_mv < 3400) { // Low battery threshold (3.4V)
      frame.header.flags |= (1 << 3);
    }

    // Fill Payload
    frame.payload.temperature = (int16_t)round(temp * 10.0f);
    frame.payload.humidity = (int16_t)round(hum * 10.0f);
    frame.payload.accel_mag = (int16_t)round(accel_g * 100.0f);
    frame.payload.light_level = battery_mv; // Send battery voltage instead of light level
    frame.payload.latitude = (int32_t)round(lat * 1e6);
    frame.payload.longitude = (int32_t)round(lng * 1e6);

    // Encrypt payload in-place (AES-128 16 bytes)
    encryptBlockInPlace((uint8_t*)&frame.payload);

    // Compute Auth Tag (2 bytes) over Header + encrypted Payload
    frame.footer.auth_tag = computeFrameAuthTag((const uint8_t*)&frame.header, sizeof(frame.header), (const uint8_t*)&frame.payload, sizeof(frame.payload));

    // Compute CRC (2 bytes) over Header + Payload + Auth Tag (which is 30 bytes)
    frame.footer.crc = calculateCRC16((const uint8_t*)&frame, sizeof(frame) - sizeof(frame.footer.crc));

    // Convert the 32-byte frame to a 64-character Hex string
    char hex_payload[65];
    static const char hexDigits[] = "0123456789abcdef";
    uint8_t* raw_frame = (uint8_t*)&frame;
    for (size_t i = 0; i < sizeof(frame); i++) {
      uint8_t value = raw_frame[i];
      hex_payload[i * 2]     = hexDigits[(value >> 4) & 0x0F];
      hex_payload[i * 2 + 1] = hexDigits[value & 0x0F];
    }
    hex_payload[64] = '\0';

    // Send the hex-encoded frame via UART to RA-08H
    LORA_SER.println(hex_payload); 
    LORA_SER.flush();

    // Print Node debug logs
    Serial.printf("[SENSOR] Temp=%.1fC, Hum=%.1f%%, Accel=%.2fg, Battery=%umV (%u%%), Tamper=%d\r\n", 
                  temp, hum, accel_g, battery_mv, battery.getPercent(), is_tamper ? 1 : 0);
    Serial.printf("[GPS] Lat=%.6f, Lng=%.6f\r\n", lat, lng);
    Serial.printf("[PACKET] Packing Frame: Seq=%u, DevID=%u, VehID=%u, Type=%u\r\n", 
                  frame.header.packet_id, frame.header.device_id, frame.header.vehicle_id, frame.header.type);
    Serial.printf("[AES] Encrypted Payload block: 16 bytes\r\n");
    Serial.printf("[AUTH] Calculated Auth Tag: 0x%04X, CRC: 0x%04X\r\n", frame.footer.auth_tag, frame.footer.crc);
    Serial.printf("[ESP32->LORA] Binary frame sent (Hex: %s)\r\n", hex_payload);

    vTaskDelayUntil(&xLastWakeTime, xInterval);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  EEPROM.begin(512); 
  Serial.println("[INIT] Initializing modules...");

  sensorDataMutex = xSemaphoreCreateMutex();
  if (sensorDataMutex == NULL) {
    Serial.println("[ERROR] Failed to create sensorDataMutex!");
  }

  if (sensorDataMutex != NULL) {
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      sensorData.temp = -999.0f;
      sensorData.hum = -999.0f;
      sensorData.accel = -999.0f;
      sensorData.shock_detected = false;
      sensorData.is_moving = false;
      xSemaphoreGive(sensorDataMutex);
    }
  }

  gVehicleConfig.begin();
  Serial.printf("[INIT] Vehicle ID (default): %s\r\n", gVehicleConfig.getDeviceId());

  Wire.begin();

  dht.begin();
  if (!adxl.begin()) {
    Serial.println("[WARN] ADXL345 not found (check wiring)");
  }
  
  Serial.println("[INIT] Starting LoRa UART...");
  LORA_SER.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(100);
  while (LORA_SER.available()) LORA_SER.read(); 
  
  Serial.println("[INIT] Detecting/generating node_id...");
  detectOrGenerateNodeId(LORA_SER);
  Serial.printf("[INIT] Final Vehicle ID: %s\r\n", gVehicleConfig.getDeviceId());
  
  tamper.begin();
  battery.begin();
  Serial.println("[INIT] Tamper (SW2/Buzzer/LED) and Battery ADC initialized");


  xTaskCreate(TaskTamperMonitor,    "TamperMon",  2048, NULL, 2, NULL);
  startDhtTask(2048, 1);        
  startAdxlTelemetry(4096, 1);  
  xTaskCreate(TaskLoraSend, "LoraSend", 4096, NULL, 1, NULL);
  xTaskCreate(TaskGPS, "TaskGPS", 4096, NULL, 1, NULL); // Create GPS task

  gps.begin();
  
  Serial.printf("\r\n[INIT] All systems initialized\r\n");
  Serial.printf("[INIT] Starting patrol mode...\r\n");
  Serial.printf("\r\n");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}