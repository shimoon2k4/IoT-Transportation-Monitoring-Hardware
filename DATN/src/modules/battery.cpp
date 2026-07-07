#include "battery.h"

BatteryModule::BatteryModule(uint8_t pin) : adc_pin(pin) {}

void BatteryModule::begin() {
    pinMode(adc_pin, INPUT);
}

uint16_t BatteryModule::readRaw() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLE_SIZE; i++) {
        sum += analogRead(adc_pin);
        delay(2);
    }
    return sum / SAMPLE_SIZE;
}

uint16_t BatteryModule::readVoltagemV() {
    uint16_t raw = readRaw();
    return (uint16_t)(raw * ADC_TO_MV);
}

uint8_t BatteryModule::getPercent() {
    uint16_t mv = readVoltagemV();
    if (mv >= 4150) return 100;
    if (mv <= 3300) return 0;
    
    // Piecewise linear interpolation for Lithium-ion discharge curve
    if (mv > 4000) {
        // 4000mV to 4150mV -> 80% to 100%
        return 80 + (uint8_t)((mv - 4000) * 20 / 150);
    } else if (mv > 3800) {
        // 3800mV to 4000mV -> 50% to 80%
        return 50 + (uint8_t)((mv - 3800) * 30 / 200);
    } else if (mv > 3600) {
        // 3600mV to 3800mV -> 20% to 50%
        return 20 + (uint8_t)((mv - 3600) * 30 / 200);
    } else {
        // 3300mV to 3600mV -> 0% to 20%
        return (uint8_t)((mv - 3300) * 20 / 300);
    }
}
