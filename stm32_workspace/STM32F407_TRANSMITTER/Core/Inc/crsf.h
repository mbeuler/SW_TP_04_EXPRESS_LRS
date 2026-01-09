/*
 * crsf.h
 *
 *  Created on: 07.01.2026
 *      Author: mbeuler
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_


#include <stdint.h>

#define CRSF_SYNC_BYTE        0xC8
#define CRSF_TYPE_RC_CHANNELS 0x16

void CRSF_PackChannels(const uint16_t *channels, uint8_t *packed);
uint8_t CRSF_CRC8_DVB(const uint8_t *data, uint16_t len);
uint16_t CRSF_CreateFrame(uint8_t *outFrame, uint8_t type, const uint8_t *payload, uint16_t payloadLen);


#endif /* INC_CRSF_H_ */
