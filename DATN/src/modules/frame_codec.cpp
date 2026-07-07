#include "frame_codec.h"

void encode_frame_to_hex(const LoRaFrame& frame, char* hex_out) {
    static const char hexDigits[] = "0123456789abcdef";
    const uint8_t* raw_frame = (const uint8_t*)&frame;
    for (size_t i = 0; i < sizeof(LoRaFrame); i++) {
        uint8_t value = raw_frame[i];
        hex_out[i * 2]     = hexDigits[(value >> 4) & 0x0F];
        hex_out[i * 2 + 1] = hexDigits[value & 0x0F];
    }
    hex_out[sizeof(LoRaFrame) * 2] = '\0';
}

bool hex_string_to_bytes(const String& hex, uint8_t* bytes, size_t max_len) {
    if (hex.length() % 2 != 0 || hex.length() / 2 > max_len) {
        return false;
    }
    for (size_t i = 0; i < hex.length(); i += 2) {
        char c1 = hex[i];
        char c2 = hex[i+1];
        uint8_t b1 = (c1 >= 'A' && c1 <= 'F') ? (c1 - 'A' + 10) :
                     (c1 >= 'a' && c1 <= 'f') ? (c1 - 'a' + 10) :
                     (c1 >= '0' && c1 <= '9') ? (c1 - '0') : 0;
        uint8_t b2 = (c2 >= 'A' && c2 <= 'F') ? (c2 - 'A' + 10) :
                     (c2 >= 'a' && c2 <= 'f') ? (c2 - 'a' + 10) :
                     (c2 >= '0' && c2 <= '9') ? (c2 - '0') : 0;
        bytes[i / 2] = (b1 << 4) | b2;
    }
    return true;
}
