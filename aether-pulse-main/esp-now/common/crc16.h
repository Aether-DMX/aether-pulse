/**
 * AETHER ESP-NOW DMX - CRC16-CCITT Implementation
 * 
 * Standard CRC16-CCITT with polynomial 0x1021 and initial value 0xFFFF.
 * Used for both serial frame validation and ESP-NOW packet validation.
 */

#ifndef AETHER_ESPNOW_CRC16_H
#define AETHER_ESPNOW_CRC16_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// CRC16-CCITT
// ═══════════════════════════════════════════════════════════════════
// Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
// Initial value: 0xFFFF
// This is the "false" CCITT variant (no bit reflection)
// ═══════════════════════════════════════════════════════════════════

/**
 * Calculate CRC16-CCITT for a buffer
 * @param data Pointer to data buffer
 * @param length Number of bytes to process
 * @return 16-bit CRC value
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t length);

/**
 * Update running CRC with additional bytes
 * @param crc Current CRC value
 * @param data Pointer to data buffer
 * @param length Number of bytes to process
 * @return Updated CRC value
 */
uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t* data, size_t length);

/**
 * Update running CRC with a single byte
 * @param crc Current CRC value
 * @param byte Single byte to process
 * @return Updated CRC value
 */
uint16_t crc16_ccitt_byte(uint16_t crc, uint8_t byte);

/**
 * Verify CRC of a buffer (CRC should be appended at end)
 * @param data Pointer to data buffer including CRC at end
 * @param length Total length including 2-byte CRC
 * @return true if CRC is valid
 */
bool crc16_verify(const uint8_t* data, size_t length);

#endif // AETHER_ESPNOW_CRC16_H
