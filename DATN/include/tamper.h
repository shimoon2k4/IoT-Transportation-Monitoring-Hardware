#ifndef TAMPER_MODULE_H
#define TAMPER_MODULE_H

#include <Arduino.h>

class TamperModule {
public:
    TamperModule(uint8_t sw_pin = 27, uint8_t buzzer_pin = 13, uint8_t led_pin = 2);
    
    void begin();
    
    // Check if the enclosure is currently open (SW2 released -> HIGH)
    bool isTriggered();
    
    // Check and update persistent tamper state
    bool checkTamper();
    
    // Alarm control
    void updateAlarm(bool active);
    void triggerAlarm();
    void clearAlarm();
    
    bool getTamperState() { return tamper_detected; }
    void resetTamper() { tamper_detected = false; }

private:
    uint8_t switch_pin;
    uint8_t bz_pin;
    uint8_t warn_led_pin;
    bool tamper_detected;
    
    // Debounce parameters
    unsigned long last_debounce_time;
    static const unsigned long DEBOUNCE_DELAY_MS = 50;
    int last_stable_state;
};

#endif // TAMPER_MODULE_H
