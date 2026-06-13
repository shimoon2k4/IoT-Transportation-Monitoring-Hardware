#include <Arduino.h>
#include "security.h"

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <esp_system.h>   // For chip ID functions

#include "gps.h"
#include "dht11.h"
#include "adxl345.h"
#include "ldr.h"
#include "vehicle_config.h"
#include "sensor_Data.h"

// ===== Pins / Config =====
#define DHTPIN    14
#define DHTTYPE   DHT11
#define LED_PIN   2
#define LDR_PIN   35     

#ifndef VEHICLE_DEVICE_ID
  #define VEHICLE_DEVICE_ID "VX"
#endif

// GPS: UART2 RX=16, TX=17 @ 9600
GPSNeo6M  gps(16, 17, 9600);
DHTModule dht(DHTPIN, DHTTYPE);
ADXLModule adxl; // ADXL345
LDRModule ldr(LDR_PIN); 

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
    bool tamper = ldr.isTamper();

    if (tamper && !g_tamper_alert) {
      g_tamper_alert = true;
      uint16_t light_level = ldr.getLightLevel();

      Serial.println("[TAMPER ALERT] BOX OPENED!");
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

    // read tamper/light status
    uint16_t light_level = ldr.readSmoothed();
    bool is_tamper = ldr.getTamperState();

    uint32_t ts = millis();
    char payload[320];
    int n = snprintf(payload, sizeof(payload),
      "{\"v\":\"%s\",\"ts\":%lu,\"t\":%.1f,\"h\":%.1f,\"a\":%.2f,\"l\":%u,\"x\":%d,\"la\":%.5f,\"lo\":%.5f}",
      gVehicleConfig.getDeviceId(),
      (unsigned long)ts,
      isnan(temp) ? -999.0F : temp,
      isnan(hum) ? -999.0F : hum,
      (accel_g < -900.0f) ? -999.0F : accel_g,
      light_level,
      is_tamper ? 1 : 0,
      lat,
      lng
    );

    if (n > 0) {
      String jsonPayload(payload);
      String signature = hmacSha256(jsonPayload);
      String signedJson = jsonPayload;
      if (signedJson.endsWith("}")) {
        signedJson = signedJson.substring(0, signedJson.length() - 1);
      }
      signedJson += ",\"sig\":\"" + signature + "\"}";

      String securePayload = encryptDataToAESBase64(signedJson);
      LORA_SER.println(securePayload); 
      LORA_SER.flush();
      Serial.print("[ESP32->LORA] AES-128 CBC + BASE64 payload sent (len: ");
      Serial.print(securePayload.length());
      Serial.println(")");
    } else {
      Serial.printf("[ERROR] snprintf failed! n=%d\r\n", n);
    }

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
  
  ldr.begin();
  ldr.setTamperThreshold(600);  
  Serial.println("[INIT] LDR tamper detection initialized (threshold=600)");


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