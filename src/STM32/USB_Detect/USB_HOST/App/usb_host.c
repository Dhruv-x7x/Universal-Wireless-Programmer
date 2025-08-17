/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
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

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_audio.h"
#include "usbh_cdc.h"
#include "usbh_msc.h"
#include "usbh_hid.h"
#include "usbh_mtp.h"
#include "Boards/Arduino_UNO/programmer.h"
#include <string.h> /* Include for string functions */


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;
extern ApplicationTypeDef Appli_state;
#define MAX_BOARD_NAME_LENGTH 50
#define STK_OK         0x14
#define STK_GET_SYNC  0x30

extern char detected_board_name[MAX_BOARD_NAME_LENGTH];  // Use the extern defined in main.c


/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_AUDIO_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_HID_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MTP_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  printf("USB Host Init Successful\r\n");
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
    printf("USBH_UserProcess called with id: %d \r\n", id);
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
//  printf("Appli_STATE: " + Appli_state);
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */
const char* check_usb_device(void)
{
    static char board_name[MAX_BOARD_NAME_LENGTH] = "No Device";  // Static local storage
    // Check the application state to determine the USB connection status
    if (Appli_state == APPLICATION_READY) {
        // Device is connected and ready
//        printf("USB Device Connected\n");
        USBH_DevDescTypeDef *dev_desc = &hUsbHostFS.device.DevDesc;
        //printf("VID: 0x%04X, PID: 0x%04X, Class: 0x%02X\r\n",
                 //dev_desc->idVendor, dev_desc->idProduct, dev_desc->bDeviceClass);
        // Classify and return the device's board name
        printf("Application ready\n");
        const char* detected = classify_usb_device(dev_desc->idVendor, dev_desc->idProduct, dev_desc->bDeviceClass);
        strncpy(board_name, detected, MAX_BOARD_NAME_LENGTH - 1);

        board_name[MAX_BOARD_NAME_LENGTH - 1] = '\0';  // Ensure null termination
    }
    else if (Appli_state == APPLICATION_DISCONNECT) {
        // Device has been disconnected
        printf("USB Device Disconnected\n");
        strncpy(board_name, "No Device", MAX_BOARD_NAME_LENGTH - 1);
        board_name[MAX_BOARD_NAME_LENGTH - 1] = '\0';
    }
    else {
        // No device detected, or in an idle state
        //printf("No USB Device Detected\n");
        strncpy(board_name, "No Device", MAX_BOARD_NAME_LENGTH - 1);
        board_name[MAX_BOARD_NAME_LENGTH - 1] = '\0';
    }

    return board_name;
}

const char* classify_usb_device(uint16_t vid, uint16_t pid, uint8_t classCode) {
    static char detected_board_name[MAX_BOARD_NAME_LENGTH];  // Static so it persists after the function exits

    if (vid == 0x2341 && (pid == 0x0043 || pid == 0x0010)) {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "Arduino UNO R3");
    } else if (classCode == 0x02) {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "USB Serial Device");
    } else {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "Unknown Device");
    }
//    printf("%s\n", detected_board_name);  // Fixed string output
    return detected_board_name;
}

#include "usbh_cdc.h"

//#define CDC_SET_CONTROL_LINE_STATE 0x22
//#define CDC_ACTIVATE_SIGNAL_DTR    0x01
//#define CDC_DEACTIVATE_SIGNAL_DTR  0x00

#define ARDUINO_RESET_PULSE_DURATION 250  // Adjust as needed
#define ARDUINO_BOOTLOADER_TIMEOUT   1000 // Adjust as needed

/**
  * @brief  Set DTR line low then high to reset Arduino into bootloader mode
  * @param  None
  * @retval USBH_StatusTypeDef: Status of the operation
  */
USBH_StatusTypeDef trigger_arduino_bootloader(void)
{
    USBH_StatusTypeDef status = USBH_FAIL;

    // Check if a device is connected and ready
    if (Appli_state != APPLICATION_READY) {
        printf("No USB device connected or device not ready\r\n");
        return USBH_FAIL;
    }

    // Make sure the current device is an Arduino UNO
    const char* current_device = check_usb_device();
    if (strcmp(current_device, "Arduino UNO R3") != 0) {
        printf("Connected device is not an Arduino UNO R3\r\n");
        return USBH_FAIL;
    }

    printf("Triggering Arduino bootloader...\r\n");

    // Get CDC handle
    CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *)hUsbHostFS.pActiveClass->pData;
    if (CDC_Handle == NULL) {
        printf("Failed to get CDC handle\r\n");
        return USBH_FAIL;
    }

    // Get the interface number
    uint16_t interface_number = (uint16_t)CDC_Handle->CommItf.NotifEp;  // Correct way to get interface

    // Setup packet
    USB_Setup_TypeDef setup;
    setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    setup.b.bRequest = CDC_SET_CONTROL_LINE_STATE;
    setup.b.wIndex.w = interface_number;
    setup.b.wLength.w = 0;

    // Set DTR low (assert)
    setup.b.wValue.w = CDC_ACTIVATE_SIGNAL_DTR;

    status = USBH_CtlReq(&hUsbHostFS, (uint8_t *)&setup, 0);
    if (status != USBH_OK) {
        printf("Failed to set DTR low\r\n");
        return status;
    }

    // Wait for reset pulse duration
    HAL_Delay(ARDUINO_RESET_PULSE_DURATION);

    // Set DTR high (deassert)
    setup.b.wValue.w = CDC_DEACTIVATE_SIGNAL_DTR;

    status = USBH_CtlReq(&hUsbHostFS, (uint8_t *)&setup, 0);
    if (status != USBH_OK) {
        printf("Failed to set DTR high\r\n");
        return status;
    }

    // Wait for bootloader to be ready
    HAL_Delay(ARDUINO_BOOTLOADER_TIMEOUT);

    printf("Arduino bootloader trigger sequence completed\r\n");
//    arduino_prog_state = ARDUINO_PROG_RESET_SENT;

    return USBH_OK;
}



//
//void send_stk_get_sync(void) {
//    uint8_t stk_sync_command[1] = { STK_GET_SYNC };  // Command array with STK_GET_SYNC
//    uint8_t stk_ok_response[1];  // Array to store the response
//
//    // Send the STK_GET_SYNC command to the device via a control transfer
//    if (USBH_ControlSend(&hUsbHostFS, stk_sync_command, 1) == USBH_OK) {
//        printf("STK_GET_SYNC command sent successfully.\r\n");
//
//        // Now receive the STK_OK response via control transfer
//        if (USBH_ControlReceive(&hUsbHostFS, stk_ok_response, 1) == USBH_OK) {
//            if (stk_ok_response[0] == STK_OK) {
//                // If the response is STK_OK, we are synchronized with the device
//                printf("Received STK_OK response.\r\n");
//            } else {
//                // If an unexpected response is received
//                printf("Unexpected response: 0x%02X\r\n", stk_ok_response[0]);
//            }
//        } else {
//            // Handle failure in receiving response
//            printf("Failed to receive response.\r\n");
//        }
//    } else {
//        // Handle failure in sending the STK_GET_SYNC command
//        printf("Failed to send STK_GET_SYNC command.\r\n");
//    }
//}
/**
  * @}
  */

