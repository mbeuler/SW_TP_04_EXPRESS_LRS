/*
 * usbh_hid_joystick.h
 *
 *  Created on: Nov 1, 2025
 *      Author: marcel.beuler
 */

#ifndef INC_USBH_HID_JOYSTICK_H_
#define INC_USBH_HID_JOYSTICK_H_

#include "usbh_core.h"
#include "usbh_hid.h"

#define JOYSTICK_REPORT_SIZE 12           // Size of the HID report descriptor
                                          // (Report ID 1, including 4 vendor-specific bytes)

#define JOYSTICK_REPORT_EFFECTIVE_SIZE 8  // Report ID 1, excluding 4 vendor-specific bytes

typedef struct {
    uint8_t buttons[19]; // 0 = not pressed, 1 = pressed
    uint8_t hat_switch;  // 0–7 = direction, 8 = neutral position
    uint16_t x;          // X-axis value
    uint16_t y;          // Y-axis value
} JoystickData_t;


USBH_StatusTypeDef USBH_HID_JoystickInit(USBH_HandleTypeDef *phost);
JoystickData_t USBH_HID_GetJoystickData(uint8_t *report);


#endif /* INC_USBH_HID_JOYSTICK_H_ */
