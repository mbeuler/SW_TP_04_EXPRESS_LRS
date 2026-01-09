/*
 * crsf.c
 *
 *  Created on: 07.01.2026
 *      Author: mbeuler
 */

#include "crsf.h"


uint8_t CRSF_CRC8_DVB(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            }
            else {
                crc <<= 1;
            }

            crc &= 0xFF;
        }
    }
    return crc;
}


void CRSF_PackChannels(const uint16_t *channels, uint8_t *out)
{
    uint8_t bitstream[176] = {0};
    uint16_t bitIndex = 0;

    // 1) Kanäle reverse wie im Python-Code
    for (int ch = 15; ch >= 0; ch--)
    {
        uint16_t v = channels[ch];
        uint16_t mapped = (uint16_t)((v - 1500) * 8 / 5 + 992);

        // 2) Bits MSB → LSB in linearen Bitstream
        for (int b = 10; b >= 0; b--)
        {
            bitstream[bitIndex++] = (mapped >> b) & 1;
        }
    }

    // 3) 176 Bits in 22 Bytes packen (MSB-first)
    uint8_t temp[22] = {0};
    for (int i = 0; i < 176; i++)
    {
        if (bitstream[i])
            temp[i / 8] |= (1 << (7 - (i % 8)));
    }

    // 4) Byte-Reihenfolge reverse wie im Python-Code
    for (int i = 0; i < 22; i++)
        out[i] = temp[21 - i];
}


uint16_t CRSF_CreateFrame(uint8_t *outFrame, uint8_t type, const uint8_t *payload, uint16_t payloadLen)
{
    outFrame[0] = CRSF_SYNC_BYTE;
    outFrame[1] = payloadLen + 2;
    outFrame[2] = type;

    for (uint16_t i = 0; i < payloadLen; i++) {
        outFrame[3 + i] = payload[i];
    }

    outFrame[3 + payloadLen] = CRSF_CRC8_DVB(&outFrame[2], payloadLen + 1);

    return payloadLen + 4;
}

