#ifndef BATTERY_MODULE_H
#define BATTERY_MODULE_H

#include <Arduino.h>

class BatteryModule {
public:
    BatteryModule(uint8_t pin = 34);
    
    void begin();
    
    // Read raw ADC value (0 - 4095)
    uint16_t readRaw();
    
    // Read battery voltage in millivolts (mV, e.g. 4200 for 4.2V)
    uint16_t readVoltagemV();
    
    // Calculate battery percentage (0 - 100%)
    uint8_t getPercent();

private:
    uint8_t adc_pin;
    
    // ADC parameters
    static const uint8_t SAMPLE_SIZE = 10;
    
    // Voltage conversion parameters
    // V_batt = V_adc * 2 (due to 100k / 100k voltage divider)
    // V_adc = raw_adc * 3.3V / 4095
    // So V_batt = raw_adc * 6.6V / 4095 = raw_adc * 1.6117 mV/unit
    // We calibrate with a multiplier if needed (e.g. 1.6117f)
    const float ADC_TO_MV = 1.6117f;
};

#endif // BATTERY_MODULE_H
