#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

#include <Arduino.h>

#pragma pack(push, 1)
struct LoRaFrameHeader {
    uint8_t magic[2];      // 0x25, 0x03
    uint8_t device_id;     // 1 - 100
    uint8_t vehicle_id;    // 1 - 100
    uint16_t session_id;   // Boot counter (anti-replay)
    uint16_t packet_id;    // sequence number (anti-replay)
    uint32_t timestamp;    // millis or epoch
    uint8_t type;          // 1 = Telemetry, 2 = Alert, 3 = Heartbeat, 0xFF = ACK
    uint8_t flags;         // Bitmask: Bit 0=Tamper, Bit 1=GPS Fix, Bit 2=Vibration, Bit 3=Low Batt
};

struct LoRaFramePayload {
    int16_t temperature;    // temp * 10
    int16_t humidity;       // hum * 10
    int16_t accel_mag;      // accel * 100
    uint16_t light_level;   // LDR (0 - 1023) or battery_mv
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

void encode_frame_to_hex(const LoRaFrame& frame, char* hex_out);
bool hex_string_to_bytes(const String& hex, uint8_t* bytes, size_t max_len);

#endif // FRAME_CODEC_H
