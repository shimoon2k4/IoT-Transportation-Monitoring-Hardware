#include "adxl345.h"
#include "vehicle_config.h"
#include "sensor_Data.h"
#include <math.h>

extern VehicleConfig gVehicleConfig;
extern ADXLModule adxl;

// ---------- ADXL345 I2C ----------
static const uint8_t DEVICE_ADDRESS = 0x53; // ALT ADDRESS = GND
static const uint8_t REG_DATA_FORMAT = 0x31;
static const uint8_t REG_POWER_CTRL  = 0x2D;
static const uint8_t REG_INT_ENABLE  = 0x2E;

static const uint8_t REG_DATAX0 = 0x32;
static const uint8_t REG_DATAX1 = 0x33;
static const uint8_t REG_DATAY0 = 0x34;
static const uint8_t REG_DATAY1 = 0x35;
static const uint8_t REG_DATAZ0 = 0x36;
static const uint8_t REG_DATAZ1 = 0x37;

static int16_t x = 0, y = 0, z = 0;

static const float G_PER_LSB = 0.0039f;

static void writeRegister(uint8_t device, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void readRegister(uint8_t device, uint8_t startReg, uint8_t numBytes, uint8_t *outValues) {
  Wire.beginTransmission(device);
  Wire.write(startReg);
  Wire.endTransmission();

  uint8_t received = Wire.requestFrom(device, numBytes);
  uint8_t i = 0;
  while (Wire.available() && i < received && i < numBytes) {
    outValues[i++] = Wire.read();
  }
}

// Đọc 6 byte và ghép thành x,y,z (signed 16-bit, little endian)
static void readXYZ() {
  uint8_t buf[6];
  readRegister(DEVICE_ADDRESS, REG_DATAX0, 6, buf);
  x = (int16_t)((int16_t)buf[1] << 8 | buf[0]);
  y = (int16_t)((int16_t)buf[3] << 8 | buf[2]);
  z = (int16_t)((int16_t)buf[5] << 8 | buf[4]);
}

// === Class implementation (using direct I2C) ===
ADXLModule::ADXLModule() {}

bool ADXLModule::begin() {
  // Try Adafruit lib first
  if (accel.begin()) {
    accel.setRange(ADXL345_RANGE_16_G);
    delay(50);  // Wait for Adafruit init
    return true;
  }

  // Fallback: direct I2C init
  Wire.begin();
  writeRegister(DEVICE_ADDRESS, REG_DATA_FORMAT, 0x0B); // FULL_RES=1, Range=±16g
  writeRegister(DEVICE_ADDRESS, REG_POWER_CTRL,  0x08); // Measure=1
  delay(100);  // Critical: wait for sensor to boot
  writeRegister(DEVICE_ADDRESS, REG_INT_ENABLE,  0x80); // Data Ready int (optional)
  return true;
}

bool ADXLModule::read(float &xg, float &yg, float &zg) {
  // Try Adafruit lib first
  sensors_event_t event;
  accel.getEvent(&event);
  if (!isnan(event.acceleration.x) && !isnan(event.acceleration.y) && !isnan(event.acceleration.z)) {
    // Adafruit returns m/s^2, convert to g for unit consistency with fallback path.
    xg = event.acceleration.x / 9.80665f;
    yg = event.acceleration.y / 9.80665f;
    zg = event.acceleration.z / 9.80665f;
    return true;
  }

  uint8_t test_buf[1];
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission() != 0) {
    xg = -999.0f;
    yg = -999.0f;
    zg = -999.0f;
    return false;  
  }

  readXYZ();
  xg = x * G_PER_LSB;
  yg = y * G_PER_LSB;
  zg = z * G_PER_LSB;
  return true;
}

void ADXLModule::getRawLSB(int16_t &x_lsb, int16_t &y_lsb, int16_t &z_lsb) {
  readXYZ();
  x_lsb = x;
  y_lsb = y;
  z_lsb = z;
}

static const float SHOCK_G_THRESHOLD = 2.5f;
static const float MOTION_G_THRESHOLD = 0.15f;

void TaskADXLData(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    bool ok = adxl.read(ax, ay, az);
    float dynamic_g = -999.0f;
    bool shock = false;
    bool moving = false;

    if (ok) {
      // Remove static gravity using HIGH-PASS FILTER
      static float gx = 0, gy = 0, gz = 0;
      const float alpha = 0.9f;

      // Ước lượng gravity (low-pass filter)
      gx = alpha * gx + (1 - alpha) * ax;
      gy = alpha * gy + (1 - alpha) * ay;
      gz = alpha * gz + (1 - alpha) * az;

      // Loại bỏ gravity component
      float dx = ax - gx;
      float dy = ay - gy;
      float dz = az - gz;

      // Gia tốc động thực sự (high-pass filtered)
      dynamic_g = sqrtf(dx*dx + dy*dy + dz*dz);
      shock = (dynamic_g >= SHOCK_G_THRESHOLD);
      moving = (dynamic_g >= MOTION_G_THRESHOLD);
    }

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      sensorData.accel = dynamic_g;
      sensorData.shock_detected = shock;
      sensorData.is_moving = moving;
      xSemaphoreGive(sensorDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void startAdxlTelemetry(unsigned long stackSize, UBaseType_t priority) {
  xTaskCreate(TaskADXLData, "ADXL Data", stackSize, NULL, priority, NULL);
}
