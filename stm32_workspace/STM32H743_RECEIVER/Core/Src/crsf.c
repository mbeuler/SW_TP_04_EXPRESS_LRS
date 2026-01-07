/*
 * crsf.c
 *
 *  Created on: 23.12.2025
 *      Author: mbeuler
 */

#include "crsf.h"

/* ===== Global CRSF data ===== */
volatile uint16_t rc_raw[16] = {0};
volatile uint16_t rc_us[16]  = {
    1500,1500,1500,1500,1500,1500,1500,1500,
    1500,1500,1500,1500,1500,1500,1500,1500
};

/* ===== Static parser variables ===== */
static uint8_t  crsf_frame[CRSF_MAX_FRAME_LEN];
static size_t   crsf_idx = 0;
static uint8_t  crsf_expected_len = 0;

typedef enum { S_WAIT_DEST, S_WAIT_LEN, S_READ_BODY } crsf_state_t;
static crsf_state_t crsf_state = S_WAIT_DEST;

// Lookup Table:  https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#crc
//                https://www.sunshine2k.de/coding/javascript/crc/crc_js.html  -->  CRC8_DVB_S2
static const uint8_t CRC8_DVB_S2_TABLE[256] = {
	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};


/* ===== Helper functions ===== */
static inline uint8_t crc8_dvb_s2_bitwise(const uint8_t *data, size_t len)
{
    uint8_t crc  = 0x00;        // Initial value
    const uint8_t POLY = 0xD5;  // CRC-8/DVB-S2 polynomial

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)(((crc << 1) ^ POLY) & 0xFF);
            } else {
                crc = (uint8_t)((crc << 1) & 0xFF);
            }
        }
    }
    return crc; // No XOR output, no reflection
}


static inline uint8_t crc8_dvb_s2_table(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00; // Initial value
    for (size_t i = 0; i < len; i++) {
        crc = CRC8_DVB_S2_TABLE[(uint8_t)(crc ^ data[i])];
    }
    return crc;
}


static inline int crsf_verify_frame(uint8_t *frame, size_t frame_len)
{
	uint8_t len;
	uint8_t crc_calc, crc_frame;
	size_t crc_in_len;

    if (frame_len < 4) return 0;
    len = frame[1];
    if (frame_len != (size_t)(len + 2)) return 0;  // Consistency
    uint8_t *type_ptr = &frame[2];
    crc_in_len = (size_t)(len - 1);                // type + payload
    //crc_calc  = crc8_dvb_s2_bitwise(type_ptr, crc_in_len);
    crc_calc  = crc8_dvb_s2_table(type_ptr, crc_in_len);
    crc_frame = frame[frame_len - 1];

    return (crc_calc == crc_frame);
}


static int is_valid_dest(uint8_t d)
{
    /* FC=0xC8, TX-Modul=0xEE, Radio-Handset=0xEA, RX=0xEC */
    return (d == 0xC8 || d == 0xEE || d == 0xEA || d == 0xEC);
}


static void crsf_unpack_rc16(const uint8_t *payload, size_t payload_len, uint16_t ch[16])
{
	// Erwartete Länge für 16*11 Bit: 176 Bit = 22 Bytes
	if (payload_len < 22) return;  // Konservativ prüfen

	// Für jede Kanalnummer i: 11-Bit-Wort ab Bitposition (11*i)
	for (int i = 0; i < 16; i++) {
		// 11-bit Startposition des Kanals i im Bitstrom
		const uint32_t bitpos    = 11U * i;
		const uint32_t byteIndex = bitpos >> 3;   // Byte-Start (bitpos/8)
		const uint32_t bitInByte = bitpos & 0x7U; // Offset im Startbyte (0...7)

		// 24-Bit Fenster: Hole bis zu 3 Bytes und baue sie little-endian zusammen
		uint32_t raw32 = 0;
		// byteIndex ist maximal 21; wir lesen defensiv bis zu 3 Bytes, prüfen die Grenzen
		for (uint32_t k = 0; k < 3; k++) {
			size_t idx = (size_t)byteIndex + (size_t)k;
			if (idx < payload_len) {
				raw32 |= ((uint32_t)payload[idx]) << (8U * (uint32_t)k);
			}
		}

		raw32 >>= bitInByte;                   // Zur Bitposition ausrichten
		ch[i]   = (uint16_t)(raw32 & 0x07FFU); // 11 Bit extrahieren
	}
}


static inline uint16_t map_rc_to_us(uint16_t raw)
{
	// Begrenzung auf CRSF-typischen Bereich
	if (raw < 172)  raw =  172;
	if (raw > 1811) raw = 1811;

	return RC_TO_US(raw);
}


static void crsf_process_valid_frame(const uint8_t *frame, size_t frame_len)
{
    /* Frame: [dest][len][type][payload][crc] */
	if (frame_len < 5) return;

	const uint8_t len  = frame[1];
	const uint8_t type = frame[2];

	const uint8_t *payload     = &frame[3];         // payload beginnt nach type
	const size_t   payload_len = (size_t)(len - 2); // len = type + payload + crc

	if (type == 0x16) {  // RC channels packet
		if (payload_len < 22) return;  // Konservativ prüfen

		uint16_t ch_temp[16] = {0};
		crsf_unpack_rc16(payload, payload_len, ch_temp);

		// Rohwerte und us für ALLE 16 Kanäle ablegen
		for (int i = 0; i < 16; i++) {
			rc_raw[i] = ch_temp[i];
			rc_us[i]  = map_rc_to_us(ch_temp[i]);
		}
	}

	// Andere types ignorieren (bei Bedarf später erweitern)
}


/* ===== Public API ===== */
void crsf_consume_byte(uint8_t b)
{
    switch (crsf_state) {
    case S_WAIT_DEST:
        if (is_valid_dest(b)) {
            crsf_frame[0] = b;
            crsf_idx = 1;
            crsf_state = S_WAIT_LEN;
        }
        break;

    case S_WAIT_LEN:
        crsf_expected_len = b;
        crsf_frame[crsf_idx++] = b;
        if (crsf_expected_len < 2) { // mindestens type(1)+crc(1)
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
            break;
        }
        if ((2 + crsf_expected_len) > CRSF_MAX_FRAME_LEN) {
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
            break;
        }
        crsf_state = S_READ_BODY;
        break;

    case S_READ_BODY:
        crsf_frame[crsf_idx++] = b;
        if (crsf_idx == (size_t)(2 + crsf_expected_len)) {
            /* Vollständiges Frame vorhanden */
            if (crsf_verify_frame(crsf_frame, crsf_idx)) {
                crsf_process_valid_frame(crsf_frame, crsf_idx);
            }
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
        }
        break;
    }
}
