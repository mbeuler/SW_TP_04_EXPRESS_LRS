/*
 * crsf.h
 *
 *  Created on: 23.12.2025
 *      Author: mbeuler
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_


#include "main.h"

#define CRSF_MAX_FRAME_LEN 64
#define RC_TO_US(x)   (1500 + (((x) - 992) * 5 + 4) / 8)

extern volatile uint16_t rc_raw[16];
extern volatile uint16_t rc_us[16];

void crsf_consume_byte(uint8_t b);


#endif /* INC_CRSF_H_ */
