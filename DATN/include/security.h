#ifndef SECURITY_MODULE_H
#define SECURITY_MODULE_H

#include <Arduino.h>

String encryptDataToAESBase64(const String& jsonStr);
String hmacSha256(const String& message);

void encryptBlockInPlace(uint8_t* block);
void decryptBlockInPlace(uint8_t* block);
uint16_t calculateCRC16(const uint8_t* data, size_t len);
uint16_t computeFrameAuthTag(const uint8_t* header, size_t header_len, const uint8_t* payload, size_t payload_len);

#endif // SECURITY_MODULE_H