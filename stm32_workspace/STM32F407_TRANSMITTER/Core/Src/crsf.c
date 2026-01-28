/*
 * crsf.c
 *
 *  Created on: 23.12.2025
 *      Author: mbeuler
 */

#include "crsf.h"

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
    return crc; // No xorout, no reflection
}

static inline uint8_t crc8_dvb_s2_table(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00; // Initial value
    for (size_t i = 0; i < len; i++) {
        crc = CRC8_DVB_S2_TABLE[(uint8_t)(crc ^ data[i])];
    }
    return crc;
}

/* -------------------------- Frame builder (TX) -------------------------- */

uint16_t CRSF_CreateFrame(uint8_t *outFrame, uint8_t type,
                          const uint8_t *payload, uint16_t payloadLen)
{
    if (!outFrame) return 0;
    if (payloadLen > CRSF_MAX_PAYLOAD_SIZE) return 0;

    outFrame[0] = CRSF_SYNC_BYTE_PRIMARY;        /* MUST be 0xC8 when sending */
    outFrame[1] = (uint8_t)(payloadLen + 2);     /* type + crc */
    outFrame[2] = type;

    for (uint16_t i = 0; i < payloadLen; i++) {
        outFrame[3 + i] = payload ? payload[i] : 0;
    }

    /* CRC over [type..payload] */
    outFrame[3 + payloadLen] = crc8_dvb_s2_table(&outFrame[2], (size_t)(payloadLen + 1));

    /* total length = LEN + 2 */
    return (uint16_t)(outFrame[1] + 2);
}

/* ------------------------ RC Packing / Unpacking ------------------------ */

void CRSF_PackChannels16x11(const uint16_t *ch_ticks, uint8_t *out22)
{
    // 16 * 11 bits = 176 bits = 22 bytes
    for (int i = 0; i < 22; i++) out22[i] = 0;

    for (int ch = 0; ch < 16; ch++) {
        uint32_t v = (uint32_t)(ch_ticks[ch] & 0x07FFu); // 11-bit
        uint32_t bitpos     = (uint32_t)(11 * ch);
        uint32_t byteIndex  = bitpos >> 3;               // start byte
        uint32_t bitInByte  = bitpos & 0x7u;             // bit offset in start byte (0..7)

        // Byte 0: lower part, left-shift into start byte
        out22[byteIndex] |= (uint8_t)(v << bitInByte);

        // Byte 1: middle part (overflowing bits from byte 0)
        if (byteIndex + 1 < 22) {
            out22[byteIndex + 1] |= (uint8_t)(v >> (8u - bitInByte));
        }

        /* Byte 2: remaining upper part only if the 11-bit field crosses two byte boundaries.
         * This happens when bitInByte > 5 (because 11 - (8 - bitInByte) > 8) */
        if (bitInByte > 5 && (byteIndex + 2) < 22) {
            out22[byteIndex + 2] |= (uint8_t)(v >> (16u - bitInByte));
        }
    }
}

void CRSF_UnpackChannels16x11(const uint8_t *in22, uint16_t *ch_ticks)
{
    /* Unpack the 16 * 11-bit fields from 22 bytes */
    for (int i = 0; i < 16; i++) {
        const uint32_t bitpos    = (uint32_t)(11 * i);
        const uint32_t byteIndex = bitpos >> 3;
        const uint32_t bitInByte = bitpos & 0x7U;

        /* Assemble a 24-bit window (up to 3 bytes), little-endian */
        uint32_t v = 0;
        /* Defensive reads */
        for (uint32_t k = 0; k < 3; k++) {
            uint32_t idx = byteIndex + k;
            uint8_t  b   = (idx < 22U) ? in22[idx] : 0;
            v |= ((uint32_t)b) << (8U * k);
        }
        v >>= bitInByte;
        ch_ticks[i] = (uint16_t)(v & 0x07FFu);
    }
}

/* ----------------------------- Parser (RX) ----------------------------- */

static inline int is_valid_sync(uint8_t s)
{
    return (s == CRSF_SYNC_BYTE_PRIMARY) ||
           (s == CRSF_SYNC_BYTE_ALT_TX)  ||
           (s == CRSF_SYNC_BYTE_ALT_RADIO) ||
           (s == CRSF_SYNC_BYTE_ALT_RX);
}

void CRSF_ParserInit(CRSF_Parser_t *p, uint32_t (*get_ms_ts_cb)(void))
{
    if (!p) return;

    p->st   = CRSF_PARSE_SYNC;
    p->idx  = 0;
    p->len  = 0;
    p->type = 0;

    for (int i = 0; i < 16; i++) p->rc.ch_ticks[i] = 992; /* Center */

    /* Clear LinkStats (payload fields) */
    p->linkStats.up_rssi_ant1 = 0;
    p->linkStats.up_rssi_ant2 = 0;
    p->linkStats.up_LQ        = 0;
    p->linkStats.up_SNR       = 0;
    p->linkStats.diversity    = 0;
    p->linkStats.rf_mode      = 0;
    p->linkStats.up_tx_power  = 0;
    p->linkStats.dn_rssi      = 0;
    p->linkStats.dn_LQ        = 0;
    p->linkStats.dn_SNR       = 0;

    /* Internal meta info */
    p->get_ms_timestamp = get_ms_ts_cb;
    p->linkstats_last_seen_ms = 0;
}

static void parse_and_store_frame(CRSF_Parser_t *p, const uint8_t *frame, size_t frame_len)
{
    if (!p || frame_len < 5) return;
    const uint8_t len = frame[1];
    const uint8_t type = frame[2];
    const uint8_t *payload = &frame[3];
    const size_t payload_len = (size_t)(len - 2); /* type + payload + crc = len => payload = len-2 */

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        if (payload_len >= 22) {
            CRSF_UnpackChannels16x11(payload, p->rc.ch_ticks);
        }
    }
    else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) {
        if (payload_len >= 10) {
            p->linkStats.up_rssi_ant1 = payload[0];
            p->linkStats.up_rssi_ant2 = payload[1];
            p->linkStats.up_LQ        = payload[2];
            p->linkStats.up_SNR       = (int8_t)payload[3];
            p->linkStats.diversity    = payload[4];
            p->linkStats.rf_mode      = payload[5];
            p->linkStats.up_tx_power  = payload[6];
            p->linkStats.dn_rssi      = payload[7];
            p->linkStats.dn_LQ        = payload[8];
            p->linkStats.dn_SNR       = (int8_t)payload[9];

            if (p->get_ms_timestamp) {
                p->linkstats_last_seen_ms = p->get_ms_timestamp();
            }
        }
    }
    /* Add more types when needed */
}

int CRSF_ParserFeed(CRSF_Parser_t *p, uint8_t byte, uint8_t *o_type)
{
    if (!p) return 0;

    switch (p->st)
    {
    case CRSF_PARSE_SYNC:
        if (is_valid_sync(byte)) {
            p->sync = byte;
            p->st = CRSF_PARSE_LEN;
        }
        break;

    case CRSF_PARSE_LEN:
        p->len = byte; /* = type + payload + crc */
        if (p->len < 2 || (uint32_t)p->len + 2U > CRSF_MAX_FRAME_SIZE) {
            /* invalid, resync */
            p->st = CRSF_PARSE_SYNC;
            break;
        }
        p->idx  = 0;
        p->st   = CRSF_PARSE_TYPE_PAYLOAD_CRC;
        break;

    case CRSF_PARSE_TYPE_PAYLOAD_CRC:
        /* collect [type..payload..crc] into p->buf[0..len-1] */
        p->buf[p->idx++] = byte;

        if (p->idx >= p->len) {
            /* full segment received */
            const uint8_t *type_ptr = &p->buf[0];
            const uint8_t rx_crc    = p->buf[p->len - 1];
            const uint8_t calc_crc  = crc8_dvb_s2_table(type_ptr, (size_t)(p->len - 1));

            if (calc_crc == rx_crc) {
                /* valid frame -> parse & update outputs */
                const uint8_t type = p->buf[0];
                uint8_t frame_local[CRSF_MAX_FRAME_SIZE];
                /* reconstruct full frame [sync][len][type][payload][crc] for parser */
                frame_local[0] = p->sync;
                frame_local[1] = p->len;
                for (uint8_t i = 0; i < p->len; i++) frame_local[2 + i] = p->buf[i];

                parse_and_store_frame(p, frame_local, (size_t)(p->len + 2));

                if (o_type) *o_type = type;

                /* back SYNC */
                p->st = CRSF_PARSE_SYNC;
                return 1; /* valid frame completed */
            } else {
                /* CRC error -> resync */
                p->st = CRSF_PARSE_SYNC;
            }
        }
        break;

    default:
        p->st = CRSF_PARSE_SYNC;
        break;
    }
    return 0;
}

/* ------------------------- Link-alive heuristic ------------------------- */

int CRSF_LinkIsAlive(const CRSF_Parser_t *p, uint32_t now_ms, uint32_t timeout_ms)
{
    if (!p) return 0;
    const uint32_t last = p->linkstats_last_seen_ms;
    if (last == 0U) return 0;
    return ((now_ms - last) <= timeout_ms) ? 1 : 0;
}
