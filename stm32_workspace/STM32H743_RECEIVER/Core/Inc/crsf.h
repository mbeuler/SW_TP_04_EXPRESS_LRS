/*
 * crsf.h
 *
 *  Created on: 23.12.2025
 *      Author: mbeuler
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_


#include "main.h"

/* ---------------------------- Constants & limits ---------------------------- */

#define CRSF_SYNC_BYTE_PRIMARY      0xC8  /* MUST use 0xC8 when transmitting */
#define CRSF_SYNC_BYTE_ALT_TX       0xEE  /* Parser: optionally accept */
#define CRSF_SYNC_BYTE_ALT_RADIO    0xEA  /* Parser: optionally accept */
#define CRSF_SYNC_BYTE_ALT_RX       0xEC  /* Parser: optionally accept */

#define CRSF_MAX_FRAME_SIZE         64    /* includes [sync] & [len] */
#define CRSF_MAX_PAYLOAD_SIZE       60

/* LEN = (type + payload + crc)  -> total length = LEN + 2 (sync + len) */

/* Frame types (subset) */
#define CRSF_FRAMETYPE_LINK_STATISTICS    0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
/* ... add more when needed */

/* ------------------------ Mapping µs <-> CRSF ticks ------------------------ */
/* Specification (center=992): US_TO_TICKS = (us - 1500)*8/5 + 992
 *                             TICKS_TO_US = (ticks - 992)*5/8 + 1500
 * This yields ~0.625 µs resolution over the typical RC range.
 */

static inline uint16_t CRSF_UsToTicks(int32_t us)
{
    /* Optional clamping should be done in application code; raw formula here */
    return (uint16_t)(((us - 1500) * 8) / 5 + 992);
}

static inline int32_t CRSF_TicksToUs(uint16_t ticks)
{
    return (int32_t)(((int32_t)ticks - 992) * 5 / 8 + 1500);
}

/* ----------------------------- Link statistics ----------------------------- */

typedef struct
{
    uint8_t  up_rssi_ant1;   /* dBm * -1 */
    uint8_t  up_rssi_ant2;   /* dBm * -1 */
    uint8_t  up_LQ;          /* % */
    int8_t   up_SNR;         /* dB */
    uint8_t  diversity;      /* 0..1 */
    uint8_t  rf_mode;        /* e.g., 0=4fps, 1=50fps,2=150Hz (historical) */
    uint8_t  up_tx_power;    /* enum (0..n) */
    uint8_t  dn_rssi;        /* dBm * -1 */
    uint8_t  dn_LQ;          /* % */
    int8_t   dn_SNR;         /* dB */
} CRSF_LinkStats_t;

/* ------------------------------ RC data block ------------------------------ */

typedef struct
{
    /* 16 channels within CRSF nominal range (11-bits) */
    uint16_t ch_ticks[16];
} CRSF_RcData_t;

/* --------------------------- Frame builder (TX) --------------------------- */
/**
 * @brief  Build a CRSF frame into outFrame.
 * @param  outFrame      Destination buffer (at least payloadLen + 4 bytes)
 * @param  type          Frame typ (e.g. 0x16 RC, 0x14 LinkStats)
 * @param  payload       Payload data pointer
 * @param  payloadLen    Payload length (0..CRSF_MAX_PAYLOAD_SIZE)
 * @return Total frame length (>= 4) or 0 on error
 *
 * Frame format: [sync=0xC8][len=payload+2][type][payload...][crc]
 * CRC8/DVB-S2 over [type..payloadEnd]
 */
uint16_t CRSF_CreateFrame(uint8_t *outFrame, uint8_t type,
                          const uint8_t *payload, uint16_t payloadLen);

/* ------------------------- RC-Packing / Unpacking ------------------------- */
/**
 * @brief  Pack 16*11-bit RC channels (ticks) into 22 bytes (little-endian, bit-contiguous).
 * @param  ch_ticks  16 channels in CRSF ticks (typ. 172..1811, Center=992)
 * @param  out22     Destination buffer (exactly 22 bytes)
 */
void CRSF_PackChannels16x11(const uint16_t *ch_ticks, uint8_t *out22);

/**
 * @brief  Unpack 16*11-bit RC channels from 22 bytes into ch_ticks[16].
 * @param  in22      Source buffer (>=22 Bytes)
 * @param  ch_ticks  Destination array with 16 entries
 */
void CRSF_UnpackChannels16x11(const uint8_t *in22, uint16_t *ch_ticks);

/* ------------------------------ Parser (RX) ------------------------------ */

typedef enum {
    CRSF_PARSE_SYNC = 0,
    CRSF_PARSE_LEN,
    CRSF_PARSE_TYPE_PAYLOAD_CRC
} CRSF_ParserState_t;

typedef struct
{
    /* Input/state */
    CRSF_ParserState_t st;
    uint8_t  sync;
    uint8_t  len;         /* = type + payload + crc */
    uint8_t  buf[CRSF_MAX_FRAME_SIZE];
    uint8_t  idx;         /* index within [type..payload..crc] */
    uint8_t  type;        /* current frame type */

    /* Outputs (updated for valid frames) */
    CRSF_RcData_t     rc;         /* last decoded RD data */
    CRSF_LinkStats_t  linkStats;  /* last decoded link statistics */


    /* Optional: application-supplied monotonous ms timer (e.g. HAL_GetTick) */
    uint32_t (*get_ms_timestamp)(void);

    /* ----- internal meta info (not part of CRSF payload) ----- */
    uint32_t linkstats_last_seen_ms;  /* only used parser-internally */
} CRSF_Parser_t;

/**
 * @brief  Initialize parser instance
 */
void CRSF_ParserInit(CRSF_Parser_t *p, uint32_t (*get_ms_ts_cb)(void));

/**
 * @brief  Feed one byte into the parser
 * @param  p       Parser instance
 * @param  byte    Incoming byte
 * @param  o_type  (optional) delivers frame type for valid frames
 * @return 1 if a complete, CRC-valid frame has been parsed, 0 otherwise
 *
 * On return 1, parser outputs (p->rc or p->linkStats) were updated according to type.
 */
int CRSF_ParserFeed(CRSF_Parser_t *p, uint8_t byte, uint8_t *o_type);

/* ------------------------- Link-alive heuristic ------------------------- */
/**
 * @brief  Check if any Link-Stats were seen within a timeout window
 * @param  p           Parser instance
 * @param  now_ms      Current time in ms (provided by the application)
 * @param  timeout_ms  e.g. 1500 ms
 * @return 1 = alive, 0 = timeout
 */
int CRSF_LinkIsAlive(const CRSF_Parser_t *p, uint32_t now_ms, uint32_t timeout_ms);

#endif /* INC_CRSF_H_ */
