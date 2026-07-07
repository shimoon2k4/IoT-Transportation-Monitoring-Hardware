#include "tamper.h"

TamperModule::TamperModule(uint8_t sw_pin, uint8_t buzzer_pin, uint8_t led_pin)
    : switch_pin(sw_pin), bz_pin(buzzer_pin), warn_led_pin(led_pin), tamper_detected(false),
      last_debounce_time(0), last_stable_state(LOW) {}

void TamperModule::begin() {
    pinMode(switch_pin, INPUT_PULLUP);
    pinMode(bz_pin, OUTPUT);
    pinMode(warn_led_pin, OUTPUT);
    digitalWrite(bz_pin, LOW);
    digitalWrite(warn_led_pin, LOW);
    last_stable_state = digitalRead(switch_pin);
}

bool TamperModule::isTriggered() {
    // If the switch pin reads HIGH, it means the lid is opened (SW2 released)
    int current_state = digitalRead(switch_pin);
    if (current_state == HIGH) {
        // Quick debounce check
        delay(10);
        if (digitalRead(switch_pin) == HIGH) {
            return true;
        }
    }
    return false;
}

bool TamperModule::checkTamper() {
    if (isTriggered()) {
        tamper_detected = true;
    }
    return tamper_detected;
}

void TamperModule::updateAlarm(bool active) {
    if (active) {
        triggerAlarm();
    } else {
        clearAlarm();
    }
}

void TamperModule::triggerAlarm() {
    digitalWrite(bz_pin, HIGH);
    digitalWrite(warn_led_pin, HIGH);
}

void TamperModule::clearAlarm() {
    digitalWrite(bz_pin, LOW);
    digitalWrite(warn_led_pin, LOW);
}
