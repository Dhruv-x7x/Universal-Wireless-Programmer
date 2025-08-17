/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_host.h
  * @version        : v1.0_Cube
  * @brief          : Header for usb_host.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_HOST__H__
#define __USB_HOST__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbh_core.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup USBH_OTG_DRIVER
  * @{
  */
#define RX_BUFF_SIZE 64

/** @defgroup USBH_HOST USBH_HOST
  * @brief Host file for Usb otg low level driver.
  * @{
  */

/** @defgroup USBH_HOST_Exported_Variables USBH_HOST_Exported_Variables
  * @brief Public variables.
  * @{
  */

/**
  * @}
  */

/** Status of the application. */
typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_READY,
  APPLICATION_DISCONNECT
}ApplicationTypeDef;

/* Type definition for Arduino programming state */
typedef enum {
    ARDUINO_PROG_IDLE,
    ARDUINO_PROG_RESET_SENT,
    ARDUINO_PROG_BOOTLOADER_READY,
	ARDUINO_WAIT_RESPONSE,
	ARDUINO_PROG_PROGRAMMED,
    ARDUINO_PROG_VERIFIED,
    ARDUINO_PROG_COMPLETED,
    ARDUINO_PROG_ERROR
} ArduinoProgrammingState;

/** @defgroup USBH_HOST_Exported_FunctionsPrototype USBH_HOST_Exported_FunctionsPrototype
  * @brief Declaration of public functions for Usb host.
  * @{
  */

/* Exported functions -------------------------------------------------------*/

/** @brief USB Host initialization function. */
void MX_USB_HOST_Init(void);
void MX_USB_HOST_Process(void);
const char* check_usb_device(void);
const char* classify_usb_device(uint16_t vid, uint16_t pid, uint8_t classCode);
void reset_arduino(void);
USBH_StatusTypeDef trigger_arduino_bootloader(void);
USBH_StatusTypeDef init_arduino_programming(void);
USBH_StatusTypeDef program_hex_file(const char *hex_data, uint32_t size);
USBH_StatusTypeDef verify_hex_file(const char *hex_data, uint32_t size);
ArduinoProgrammingState get_programming_state(void);
void reset_programming_state(void);
const char* get_programming_status_message(void);
USBH_StatusTypeDef send_stk_command(uint8_t cmd, uint8_t *params, uint8_t param_len);
void receive_stk_response(uint8_t *rx_buffer);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_HOST__H__ */


