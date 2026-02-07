/*
 * usbh_hid_joystick.c
 *
 *  Created on: Nov 1, 2025
 *      Author: marcel.beuler
 */

#include "usbh_hid_joystick.h"

// Static buffer for incoming HID reports (input report with ID 1, 12 bytes)
static uint8_t joystick_rx_report_buf[JOYSTICK_REPORT_SIZE];

// Static FIFO buffer for storing multiple HID reports before processing
static uint8_t fifo_buffer[64];


USBH_StatusTypeDef USBH_HID_JoystickInit(USBH_HandleTypeDef *phost) {
	// Get pointer to the central HID class data structure
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

	// Initialize the report buffer to zero (clean start)
	for (uint32_t i = 0; i < JOYSTICK_REPORT_SIZE; i++) {
	    joystick_rx_report_buf[i] = 0;
	}

	// Set expected report length according to the HID report descriptor
	HID_Handle->length = JOYSTICK_REPORT_SIZE;

	// Assign the report buffer to the HID handle for incoming data
	//HID_Handle->pData = phost->device.Data;
	HID_Handle->pData = joystick_rx_report_buf;

	// Initialize the FIFO buffer for temporary storage of multiple HID reports
	// This prevents data loss if several reports arrive before the application reacts
	USBH_HID_FifoInit(&HID_Handle->fifo, fifo_buffer, sizeof(fifo_buffer));

	// Set the HID state to IDLE to enable polling and event callback processing
	HID_Handle->state = USBH_HID_IDLE;

	// Return success status after initialization
    return USBH_OK;
}


JoystickData_t USBH_HID_GetJoystickData(uint8_t *report) {
    JoystickData_t data;

    // Extract buttons 1–19 from the report
    // The button states are packed as a bitfield across report[1], report[2], and the lower 3 bits of report[3]
    uint32_t buttonBits = report[1] | (report[2] << 8) | ((report[3] & 0x07) << 16);
    for (int i = 0; i < 19; i++) {
    	// Each bit represents the state of one button: 0 = not pressed, 1 = pressed
        data.buttons[i] = (buttonBits >> i) & 0x01;
    }

    // Extract the hat switch value from bits 4–7 of report[3]
    // Values 0–7 indicate direction, 8 means neutral (not pressed).
    uint8_t raw_hat = (report[3] >> 4) & 0x0F;
    data.hat_switch = raw_hat;

    // Read X and Y axis values (16-bit, little endian)
    data.x = report[4] | (report[5] << 8);
    data.y = report[6] | (report[7] << 8);
    return data;
}
