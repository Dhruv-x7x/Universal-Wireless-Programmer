/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
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


	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */

	/* USER CODE END Includes */

	/* Private typedef -----------------------------------------------------------*/
	/* USER CODE BEGIN PTD */

	/* USER CODE END PTD */

	/* Private define ------------------------------------------------------------*/
	/* USER CODE BEGIN PD */

	/* USER CODE END PD */

	/* Private macro -------------------------------------------------------------*/
	/* USER CODE BEGIN PM */

	/* USER CODE END PM */

	/* Private variables ---------------------------------------------------------*/
	/* GLOBAL DEFINITIONS - DO NOT REMOVE THIS SECTION */
	// Define all protocol constants at the top level to ensure visibility
	#define STK_GET_SYNC               0x30  // Command: Get Sync
	#define STK_GET_SYNC_EXT           0x20  // Command param: Get Sync (extended)
	#define STK_INSYNC                 0x14  // Response: In Sync
	#define STK_OK                     0x10  // Response: OK

	// Arduino parameters
	#define ARDUINO_RESET_PIN          GPIO_PIN_4
	#define ARDUINO_RESET_PORT         GPIOB
	#define ARDUINO_BOOTLOADER_TIMEOUT 1500  // UNO bootloader active window (~1.5 seconds)
	#define ARDUINO_SYNC_RETRIES       10    // Increased number of sync attempts
	#define ARDUINO_RESET_TIMEOUT      800   // Timeout for Arduino to respond after reset
	#define MAX_RESET_ATTEMPTS         5     // Increased number of reset attempts

	// Reset timing parameters (in ms)
	#define RESET_PULSE_DURATION       2000  // Much longer reset pulse for visible LED blink
	#define POST_RESET_DELAY           250   // Increased wait time after reset
	#define INTER_RESET_DELAY          1000  // Longer delay between reset attempts
	#define SYNC_RETRY_DELAY           100   // Delay between sync command retries

	// Response timeouts
	#define SYNC_RESPONSE_TIMEOUT      500   // Increased timeout for sync response
	#define SECOND_BYTE_TIMEOUT        300   // Dedicated timeout for second byte

	// Buffer and UART settings
	#define UART_TIMEOUT               1000
	#define RX_BUFFER_SIZE             256
	#define DEBUG_BUFFER_SIZE          512

	// Feature flags
	#define DTR_CONTROL_ENABLED        1
	#define VERBOSE_DEBUG_ENABLED      1
	#define DIRECT_SYNC_COMMAND        1
	/* END OF GLOBAL DEFINITIONS */

	/* STK500 Protocol Commands for Hex File Upload */
	#define STK_LOAD_ADDRESS         0x55  // Command: Load Address
	#define STK_PROG_PAGE            0x64  // Command: Program Page
	#define STK_READ_PAGE            0x74  // Command: Read Page
	#define STK_READ_SIGN            0x75  // Command: Read Signature
	#define STK_LEAVE_PROGMODE       0x51  // Command: Leave Programming Mode
	#define STK_ENTER_PROGMODE       0x50  // Command: Enter Programming Mode
	#define STK_UNIVERSAL            0x56  // Command: Universal
	#define STK_CHIP_ERASE           0x52  // Command: Chip erase (ATmega328P)

	/* ATmega328P Memory Specifications */
	#define FLASH_PAGE_SIZE          128   // Flash page size in bytes
	#define EEPROM_PAGE_SIZE         4     // EEPROM page size in bytes
	#define MAX_FLASH_ADDRESS        0x8000 // 32KB flash memory
	#define BOOT_SECTION_START       0x7000 // Do not write to bootloader section

	/* Include the hex file data */

	/* APPLICATION START */

	/* Includes ------------------------------------------------------------------*/
	#include "main.h"
	#include "usart.h"
	#include "gpio.h"
	#include <stdio.h>
	#include <string.h>
	#include <stdbool.h>

	/* Private typedef -----------------------------------------------------------*/
	typedef enum {
	  LOG_STM_TO_ARDUINO,  // Messages from STM to Arduino
	  LOG_ARDUINO_TO_STM   // Messages from Arduino to STM
	} LogType;

	typedef enum {
	  MEMORY_TYPE_FLASH,
	  MEMORY_TYPE_EEPROM,
	  MEMORY_TYPE_FUSE
	} MemoryType;

	typedef struct {
	  uint32_t address;      // Current memory address
	  uint8_t *data;         // Pointer to data buffer
	  uint16_t size;         // Size of data in buffer
	  MemoryType type;       // Type of memory being programmed
	} ProgrammingState;

	/* Private define ------------------------------------------------------------*/
	// UARTs
	#define UART_ARDUINO    huart1  // Connected to Arduino hardware
	#define UART_DEBUG      huart6  // Connected to PL2303 for debug via Putty

	/* Private variables ---------------------------------------------------------*/
	uint8_t rxBuffer[RX_BUFFER_SIZE];
	uint8_t txBuffer[RX_BUFFER_SIZE];
	char debugBuffer[DEBUG_BUFFER_SIZE];
	uint32_t tickStart;
	uint8_t dtrState = 0;
	uint8_t prevDtrState = 0;

	/* Interrupt mode UART variables */
	volatile uint8_t uartRxCircBuffer[RX_BUFFER_SIZE];
	volatile uint16_t rxHead = 0;
	volatile uint16_t rxTail = 0;
	volatile bool uartRxReady = false;
	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	void ResetArduino(void);
	bool ResetAndSyncArduino(uint8_t attempts);
	void LogMessage(LogType type, uint8_t* buffer, uint16_t size);
	bool IsSync(uint8_t* buffer, uint16_t size);
	bool SendDirectSync(void);
	bool SendSyncWithRetries(uint8_t retries);

	/* Function prototypes for hex upload */
	bool EnterProgrammingMode(void);
	bool LeaveProgrammingMode(void);
	bool SendCommandAndWaitForSync(uint8_t cmd, uint8_t *params, uint8_t paramSize, uint8_t *response, uint8_t responseSize);
	bool LoadAddress(uint16_t address);
	bool ProgramPage(uint8_t *data, uint16_t size, MemoryType type);
	bool EraseChip(void);
	bool VerifySignature(void);
	bool UploadHexFile(void);
	void PrintUploadProgress(uint32_t currentAddress, uint32_t totalSize);
	bool GetSignOn(void);
	uint16_t PrepareHexFilePages(uint8_t preparedPages[][FLASH_PAGE_SIZE],
								uint16_t pageSizes[],
								uint16_t pageAddresses[],
								uint16_t maxPages);

	/* Private user code ---------------------------------------------------------*/

	/**
	  * @brief Reset Arduino UNO into bootloader mode with precise timing for ATmega328P
	  * @retval None
	  */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	  if (huart->Instance == UART_ARDUINO.Instance) {
	    // Save the received byte in the circular buffer
	    uartRxCircBuffer[rxHead] = rxBuffer[0]; // Get the received byte from the temp buffer
	    rxHead = (rxHead + 1) % RX_BUFFER_SIZE; // Increment head with wrap-around

	    // Set flag indicating data is available
	    uartRxReady = true;

	    // Re-enable the UART receive interrupt for the next byte
	    HAL_UART_Receive_IT(&UART_ARDUINO, rxBuffer, 1);
	  }
	}

	/**
	  * @brief Start UART receive in interrupt mode
	  * @retval None
	  */
	void StartUartInterruptMode(void)
	{
	  // Reset buffer pointers
	  rxHead = 0;
	  rxTail = 0;
	  uartRxReady = false;

	  // Enable UART receive interrupt
	  HAL_UART_Receive_IT(&UART_ARDUINO, rxBuffer, 1);

	  sprintf(debugBuffer, "[%lu] SYSTEM: UART interrupt mode started\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	}


	void ResetArduino(void)
	{
	  // Log the reset action start
	  sprintf(debugBuffer, "[%lu] SYSTEM: Arduino UNO reset sequence initiated\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // UNO/ATmega328P specific bootloader entry sequence:

	  // 1. Set RESET low (Arduino enters reset)
	  HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_RESET);

	  // 2. Hold reset for much longer to ensure the LED visibly blinks (2 seconds)
	  HAL_Delay(RESET_PULSE_DURATION);

	  // 3. Release RESET - bootloader starts immediately after release
	  HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_SET);

	  // 4. Wait longer for the bootloader to initialize properly
	  HAL_Delay(POST_RESET_DELAY);

	  // Log the reset action completion
	  sprintf(debugBuffer, "[%lu] SYSTEM: Arduino UNO reset sequence completed - bootloader window active\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	}

	/**
	  * @brief  Send STK_GET_SIGN_ON command to verify the bootloader is present
	  * @retval true if successful, bootloader responds with "AVR STK"
	  */bool GetSignOn(void)
	  {
		uint8_t response[7]; // Space for "AVR STK"
		bool success = false;

		sprintf(debugBuffer, "[%lu] UPLOAD: Sending GET_SIGN_ON command\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		// Send the GET_SIGN_ON command (0x31) with no parameters
		success = SendCommandAndWaitForSync(0x31, NULL, 0, response, 7); // 7 bytes for "AVR STK"

		if (success) {
		  // Validate the received signature
		  char signatureStr[8]; // +1 for null terminator
		  memcpy(signatureStr, response, 7);
		  signatureStr[7] = '\0';

		  sprintf(debugBuffer, "[%lu] UPLOAD: Received signature: %s\r\n", HAL_GetTick(), signatureStr);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  // Check if the signature is "AVR STK"
		  if (strcmp(signatureStr, "AVR STK") == 0) {
			sprintf(debugBuffer, "[%lu] UPLOAD: Valid signature confirmed\r\n", HAL_GetTick());
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  } else {
			sprintf(debugBuffer, "[%lu] UPLOAD: Invalid signature received\r\n", HAL_GetTick());
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			success = false;
		  }
		} else {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Failed to get signature\r\n", HAL_GetTick());
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		}

		return success;
	  }
	/**
	  * @brief Log message with timestamp and direction
	  * @param type: Type of message (direction)
	  * @param buffer: Message content
	  * @param size: Message size
	  * @retval None
	  */
	void LogMessage(LogType type, uint8_t* buffer, uint16_t size)
	{
	  uint32_t timestamp = HAL_GetTick();
	  char prefix[32];

	  // Determine message prefix based on type
	  switch(type) {
		case LOG_STM_TO_ARDUINO:
		  strcpy(prefix, "STM->ARDUINO");
		  break;
		case LOG_ARDUINO_TO_STM:
		  strcpy(prefix, "ARDUINO->STM");
		  break;
		default:
		  strcpy(prefix, "UNKNOWN");
	  }

	  // Format the log header with timestamp and direction
	  sprintf(debugBuffer, "[%lu] %s: ", timestamp, prefix);
	  uint16_t headerLen = strlen(debugBuffer);

	  // Send log header
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, headerLen, UART_TIMEOUT);

	  // For binary data, output hex values
	  for (uint16_t i = 0; i < size; i++) {
		sprintf(debugBuffer, "0x%02X ", buffer[i]);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  // Add newline
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"\r\n", 2, UART_TIMEOUT);
	}

	/**
	  * @brief Check if buffer contains an avrdude sync request
	  * @param buffer: Buffer to check
	  * @param size: Buffer size
	  * @retval true if sync request is detected
	  */
	bool IsSync(uint8_t* buffer, uint16_t size)
	{
	  // Check for the standard STK500 sync pattern (0x30, 0x20)
	  if (size >= 2 && buffer[0] == STK_GET_SYNC && buffer[1] == STK_GET_SYNC_EXT)
		return true;

	  // Check for alternate sync patterns
	  if (size >= 2 && buffer[0] == STK_GET_SYNC && buffer[1] == 0x00)
		return true;

	  // For UNO, sometimes just the first byte is enough to check
	  if (size >= 1 && buffer[0] == STK_GET_SYNC)
		return true;

	  return false;
	}

	/**
	  * @brief Send a sync command directly to Arduino bootloader
	  * @retval true if sync response received
	  */
	bool SendDirectSync(void)
	{
	  uint8_t syncCmd[2] = {STK_GET_SYNC, STK_GET_SYNC_EXT};
	  uint8_t response[2] = {0, 0};
	  bool success = false;

	  // Log action
	  sprintf(debugBuffer, "[%lu] SYSTEM: Sending direct sync command to bootloader\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // Flush any existing data in the UART buffers
	  HAL_UART_AbortReceive(&UART_ARDUINO);

	  // Wait for UART to stabilize
	  HAL_Delay(10);

	  // Send sync command
	  HAL_UART_Transmit(&UART_ARDUINO, syncCmd, 2, UART_TIMEOUT);

	  // Try to receive first byte with longer timeout
	  if (HAL_UART_Receive(&UART_ARDUINO, &response[0], 1, SYNC_RESPONSE_TIMEOUT) == HAL_OK) {
		// We got the first byte, check if it's STK_INSYNC
		if (response[0] == STK_INSYNC) {
		  sprintf(debugBuffer, "[%lu] SYSTEM: Received STK_INSYNC (0x%02X)\r\n", HAL_GetTick(), response[0]);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  // Try to get the second byte (STK_OK) with dedicated timeout
		  if (HAL_UART_Receive(&UART_ARDUINO, &response[1], 1, SECOND_BYTE_TIMEOUT) == HAL_OK) {
			sprintf(debugBuffer, "[%lu] SYSTEM: Received second byte: 0x%02X\r\n", HAL_GetTick(), response[1]);
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

			if (response[1] == STK_OK) {
			  sprintf(debugBuffer, "[%lu] SYSTEM: Second byte is STK_OK - Bootloader sync confirmed!\r\n", HAL_GetTick());
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			  success = true;
			} else {
			  sprintf(debugBuffer, "[%lu] SYSTEM: Second byte is not STK_OK\r\n", HAL_GetTick());
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			}
		  } else {
			sprintf(debugBuffer, "[%lu] SYSTEM: No second byte received after STK_INSYNC\r\n", HAL_GetTick());
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  }
		} else {
		  sprintf(debugBuffer, "[%lu] SYSTEM: Received 0x%02X instead of STK_INSYNC\r\n", HAL_GetTick(), response[0]);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		}
	  } else {
		sprintf(debugBuffer, "[%lu] SYSTEM: No response from bootloader\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  return success;
	}

	/* USER CODE BEGIN PV */

	/**
	  * @brief Send sync commands with multiple retries
	  * @param retries: Number of sync command retries
	  * @retval true if sync was achieved
	  */
	bool SendSyncWithRetries(uint8_t retries)
	{
	  bool syncSuccess = false;

	  for (uint8_t i = 0; i < retries; i++) {
		sprintf(debugBuffer, "[%lu] SYSTEM: Sync attempt %d of %d\r\n", HAL_GetTick(), i+1, retries);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		if (SendDirectSync()) {
		  syncSuccess = true;
		  break;
		}

		// Wait before trying again
		if (i < retries - 1) {
		  HAL_Delay(SYNC_RETRY_DELAY);
		}
	  }

	  return syncSuccess;
	}

	/**
	  * @brief Perform multiple reset attempts to ensure bootloader entry
	  * @param attempts: Number of reset attempts
	  * @retval true if sync was achieved
	  */
	bool ResetAndSyncArduino(uint8_t attempts)
	{
	  bool syncAchieved = false;

	  for (uint8_t i = 0; i < attempts; i++) {
		// Log attempt number if multiple attempts
		if (attempts > 1) {
		  sprintf(debugBuffer, "[%lu] SYSTEM: Reset attempt %d of %d\r\n", HAL_GetTick(), i+1, attempts);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		}

		// Reset Arduino
		ResetArduino();

		// Try multiple sync commands after each reset
		if (SendSyncWithRetries(ARDUINO_SYNC_RETRIES)) {
		  sprintf(debugBuffer, "[%lu] SYSTEM: Bootloader sync achieved on reset attempt %d!\r\n", HAL_GetTick(), i+1);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  syncAchieved = true;
		  break;
		}

		// If this wasn't the last attempt, wait before trying again
		if (i < attempts - 1) {
		  sprintf(debugBuffer, "[%lu] SYSTEM: Sync failed, waiting before next reset attempt...\r\n", HAL_GetTick());
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  HAL_Delay(INTER_RESET_DELAY);
		}
	  }

	  if (!syncAchieved && attempts > 1) {
		sprintf(debugBuffer, "[%lu] SYSTEM: Failed to sync after %d reset attempts\r\n", HAL_GetTick(), attempts);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  return syncAchieved;
	}
	/**
	  * @brief  Send STK500 command and wait for the INSYNC/OK response
	  * @param  cmd: STK500 command byte
	  * @param  params: Parameter buffer
	  * @param  paramSize: Parameter buffer size
	  * @param  response: Buffer to store any additional response data
	  * @param  responseSize: Expected response data size (excluding INSYNC/OK bytes)
	  * @retval true if command succeeded
	  */
	bool SendCommandAndWaitForSync(uint8_t cmd, uint8_t *params, uint8_t paramSize, uint8_t *response, uint8_t responseSize)
	  {
		uint8_t cmdBuffer[RX_BUFFER_SIZE];
	//    uint8_t respBuffer[2 + RX_BUFFER_SIZE]; // INSYNC + OK + any response data
		bool success = false;

		// Prepare command buffer
		cmdBuffer[0] = cmd;

		// Copy parameters if any
		if (params != NULL && paramSize > 0) {
		  memcpy(&cmdBuffer[1], params, paramSize);
		}

		// Add Sync_CRC_EOP byte at the end
		cmdBuffer[1 + paramSize] = 0x20;  // Sync_CRC_EOP

		// Total command size is: 1 byte command + paramSize + 1 byte Sync_CRC_EOP
		uint8_t totalSize = 1 + paramSize + 1;

		// Log command with simple format to minimize processing time
		sprintf(debugBuffer, "[%lu] UPLOAD: Command 0x%02X, %d params\r\n", HAL_GetTick(), cmd, paramSize);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		// Flush any existing data in the UART buffers
		HAL_UART_AbortReceive(&UART_ARDUINO);

		// Wait for UART to stabilize - keep this delay
		HAL_Delay(10);

		// Send command + parameters + Sync_CRC_EOP
		HAL_UART_Transmit(&UART_ARDUINO, cmdBuffer, totalSize, UART_TIMEOUT);

		// Calculate total expected response size: INSYNC + responseSize + STK_OK
		uint8_t totalResponseSize = 1 + responseSize + 1;

		// Create a single buffer for the entire response
		uint8_t fullResponse[2 + RX_BUFFER_SIZE] = {0}; // Size should be enough for all responses

		// Extended timeout for receiving complete response
		uint32_t extendedTimeout = SYNC_RESPONSE_TIMEOUT + (responseSize * 100);

		// Receive the entire response with a single call
		HAL_StatusTypeDef receiveStatus = HAL_UART_Receive(&UART_ARDUINO, fullResponse, totalResponseSize, extendedTimeout);

		// Log the received bytes without processing between receives
		if (receiveStatus == HAL_OK) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Received %d bytes in response:\r\n", HAL_GetTick(), totalResponseSize);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  for (int i = 0; i < totalResponseSize; i++) {
			sprintf(debugBuffer, "  Byte %d: 0x%02X\r\n", i, fullResponse[i]);
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  }

		  // Check if response pattern is correct
		  if (fullResponse[0] == STK_INSYNC && fullResponse[totalResponseSize-1] == STK_OK) {
			// Copy response data if needed
			if (response != NULL && responseSize > 0) {
			  memcpy(response, &fullResponse[1], responseSize);
			}

			sprintf(debugBuffer, "[%lu] UPLOAD: Command 0x%02X completed successfully!\r\n", HAL_GetTick(), cmd);
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			success = true;
		  } else {
			sprintf(debugBuffer, "[%lu] UPLOAD: Response pattern incorrect\r\n", HAL_GetTick());
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  }
		} else {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Failed to receive complete response (status: %d)\r\n",
				  HAL_GetTick(), receiveStatus);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  // Try to read whatever bytes we can get
		  uint8_t availableBytes = 0;
		  while (HAL_UART_Receive(&UART_ARDUINO, &fullResponse[availableBytes], 1, 10) == HAL_OK && availableBytes < sizeof(fullResponse)) {
			availableBytes++;
		  }

		  if (availableBytes > 0) {
			sprintf(debugBuffer, "[%lu] UPLOAD: Retrieved %d partial bytes:\r\n", HAL_GetTick(), availableBytes);
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

			for (int i = 0; i < availableBytes; i++) {
			  sprintf(debugBuffer, "  Byte %d: 0x%02X\r\n", i, fullResponse[i]);
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			}
		  }
		}

		return success;
	  }

	/**
	  * @brief  Enter programming mode
	  * @retval true if successful
	  */
	/**
	  * @brief  Enter programming mode
	  * @retval true if successful
	  */
	bool EnterProgrammingMode(void)
	{
	  sprintf(debugBuffer, "[%lu] UPLOAD: Entering programming mode\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // Use SendCommandAndWaitForSync with no parameters and no expected response data
	  if (!SendCommandAndWaitForSync(STK_ENTER_PROGMODE, NULL, 0, NULL, 0)) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Failed to enter programming mode\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }

	  sprintf(debugBuffer, "[%lu] UPLOAD: Successfully entered programming mode\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  return true;
	}

	/**
	  * @brief  Leave programming mode
	  * @retval true if successful
	  */
	bool LeaveProgrammingMode(void)
	{
	  sprintf(debugBuffer, "[%lu] UPLOAD: Leaving programming mode\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  return SendCommandAndWaitForSync(STK_LEAVE_PROGMODE, NULL, 0, NULL, 0);
	}

	/**
	  * @brief  Load address for subsequent operations
	  * @param  address: Word address (address/2 for flash)
	  * @retval true if successful
	  */
	bool LoadAddress(uint16_t address)
	{
	  uint8_t params[2];

	  // Convert byte address to word address for flash memory
	  uint16_t wordAddress = address >> 1;

	  // STK500 protocol expects low byte first
	  params[0] = wordAddress & 0xFF;        // Low byte
	  params[1] = (wordAddress >> 8) & 0xFF; // High byte

	  sprintf(debugBuffer, "[%lu] UPLOAD: Loading address 0x%04X (word: 0x%04X)\r\n",
			  HAL_GetTick(), address, wordAddress);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  return SendCommandAndWaitForSync(STK_LOAD_ADDRESS, params, 2, NULL, 0);
	}

	/**
	  * @brief  Program a page of data
	  * @param  data: Data buffer to program
	  * @param  size: Size of data to program
	  * @param  type: Type of memory (FLASH or EEPROM)
	  * @retval true if successful
	  */
	bool ProgramPage(uint8_t *data, uint16_t size, MemoryType type)
	{
	  uint8_t params[size + 3]; // size_low, size_high, memtype, data[size]

	  // Check size limits based on memory type
	  uint16_t maxSize = (type == MEMORY_TYPE_FLASH) ? FLASH_PAGE_SIZE : EEPROM_PAGE_SIZE;
	  if (size > maxSize) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Page size %d exceeds max %d for memory type %d\r\n",
				HAL_GetTick(), size, maxSize, type);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }

	  // Prepare parameters
	  params[0] = size & 0xFF;         // Size low byte
	  params[1] = (size >> 8) & 0xFF;  // Size high byte
	  params[2] = (type == MEMORY_TYPE_FLASH) ? 'F' : 'E'; // 'F' for flash, 'E' for EEPROM

	  // Copy data
	  memcpy(&params[3], data, size);

	  sprintf(debugBuffer, "[%lu] UPLOAD: Programming %d bytes to %c memory\r\n",
			  HAL_GetTick(), size, params[2]);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  return SendCommandAndWaitForSync(STK_PROG_PAGE, params, size + 3, NULL, 0);
	}

	bool ProgramPageSpecial(uint8_t *data, uint16_t size, char memoryType)
	{
	  uint8_t cmdBuffer[4 + FLASH_PAGE_SIZE + 1]; // Command + size high/low + memory type + data + CRC_EOP
	  uint8_t response[2] = {0}; // For INSYNC and OK responses
	  bool success = false;

	  // Prepare command buffer
	  cmdBuffer[0] = STK_PROG_PAGE;     // Command
	  cmdBuffer[1] = (size >> 8) & 0xFF; // Size high byte
	  cmdBuffer[2] = size & 0xFF;        // Size low byte
	  cmdBuffer[3] = memoryType;         // 'F' for flash, 'E' for EEPROM

	  // Copy data
	  memcpy(&cmdBuffer[4], data, size);

	  // Add Sync_CRC_EOP at the end
	  cmdBuffer[4 + size] = 0x20; // Sync_CRC_EOP

	  // Total length of command
	  uint16_t totalSize = 4 + size + 1;

	  sprintf(debugBuffer, "[%lu] UPLOAD: Programming %d bytes to %c memory (special method)\r\n",
			  HAL_GetTick(), size, memoryType);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // Flush any existing data in the UART buffers
	  HAL_UART_AbortReceive(&UART_ARDUINO);

	  // Wait for UART to stabilize
	  HAL_Delay(20);

	  // 1. Send command byte
	  HAL_UART_Transmit(&UART_ARDUINO, &cmdBuffer[0], 1, UART_TIMEOUT);
	  HAL_Delay(5);

	  // 2. Send size bytes and memory type
	  HAL_UART_Transmit(&UART_ARDUINO, &cmdBuffer[1], 3, UART_TIMEOUT);
	  HAL_Delay(5);

	  // 3. Send data in smaller chunks with delays
	  for (uint16_t offset = 0; offset < size; offset += 16) {
		uint16_t chunkSize = ((offset + 16) > size) ? (size - offset) : 16;
		HAL_UART_Transmit(&UART_ARDUINO, &data[offset], chunkSize, UART_TIMEOUT);
		HAL_Delay(1); // Brief delay between chunks
	  }

	  // 4. Send final Sync_CRC_EOP
	  HAL_UART_Transmit(&UART_ARDUINO, &cmdBuffer[4 + size], 1, UART_TIMEOUT);

	  // 5. Wait for INSYNC response with extended timeout
	  uint32_t programmingTimeout = 1000; // 1 second timeout for flash programming

	  if (HAL_UART_Receive(&UART_ARDUINO, &response[0], 1, programmingTimeout) == HAL_OK) {
		if (response[0] == STK_INSYNC) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Received STK_INSYNC after programming\r\n", HAL_GetTick());
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  // 6. Wait for OK response
		  if (HAL_UART_Receive(&UART_ARDUINO, &response[1], 1, 500) == HAL_OK) {
			if (response[1] == STK_OK) {
			  sprintf(debugBuffer, "[%lu] UPLOAD: Programming successful\r\n", HAL_GetTick());
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			  success = true;
			} else {
			  sprintf(debugBuffer, "[%lu] UPLOAD: Received non-OK response: 0x%02X\r\n",
					 HAL_GetTick(), response[1]);
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			}
		  } else {
			sprintf(debugBuffer, "[%lu] UPLOAD: No OK response received\r\n", HAL_GetTick());
			HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  }
		} else {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Received 0x%02X instead of STK_INSYNC\r\n",
				 HAL_GetTick(), response[0]);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		}
	  } else {
		sprintf(debugBuffer, "[%lu] UPLOAD: No response after programming\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  return success;
	}
	/**
	  * @brief  Erase chip
	  * @retval true if successful
	  */
	bool EraseChip(void)
	{
	  sprintf(debugBuffer, "[%lu] UPLOAD: Erasing chip\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  return SendCommandAndWaitForSync(STK_CHIP_ERASE, NULL, 0, NULL, 0);
	}

	/**
	  * @brief  Verify device signature
	  * @retval true if device is ATmega328P
	  */
	bool VerifySignature(void)
	{
	  uint8_t response[3];

	  sprintf(debugBuffer, "[%lu] UPLOAD: Reading device signature\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  if (!SendCommandAndWaitForSync(STK_READ_SIGN, NULL, 0, response, 3)) {
		return false;
	  }

	  // ATmega328P signature is 0x1E 0x95 0x0F
	  if (response[0] == 0x1E && response[1] == 0x95 && response[2] == 0x0F) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Signature verified: ATmega328P\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return true;
	  } else {
		sprintf(debugBuffer, "[%lu] UPLOAD: Unexpected signature: 0x%02X 0x%02X 0x%02X\r\n",
				HAL_GetTick(), response[0], response[1], response[2]);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }
	}

	/**
	  * @brief  Print upload progress to debug UART
	  * @param  currentAddress: Current address being programmed
	  * @param  totalSize: Total size to program
	  * @retval None
	  */
	void PrintUploadProgress(uint32_t currentAddress, uint32_t totalSize)
	{
	  uint8_t percent = (currentAddress * 100) / totalSize;
	  static uint8_t lastPercent = 0xFF;

	  // Only print when percentage changes significantly to avoid flooding debug port
	  if (percent != lastPercent && (percent % 10 == 0 || percent == 5 || percent == 95)) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Progress %d%% - Address 0x%04lX of 0x%04lX\r\n",
				HAL_GetTick(), percent, currentAddress, totalSize);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		lastPercent = percent;
	  }
	}

	/**
	  * @brief  Upload the hex file to Arduino
	  * @retval true if upload successful
	  */
	bool UploadHexFile(void)
	{
	  bool success = false;

	  // Define arrays for pre-processed pages (adjust MAX_PAGES as needed)
	  #define MAX_PAGES 64
	  uint8_t preparedPages[MAX_PAGES][FLASH_PAGE_SIZE];
	  uint16_t pageSizes[MAX_PAGES];
	  uint16_t pageAddresses[MAX_PAGES];

	  // Pre-process hex file first
	  uint16_t totalPages = PrepareHexFilePages(preparedPages, pageSizes, pageAddresses, MAX_PAGES);

	  if (totalPages == 0) {
		sprintf(debugBuffer, "[%lu] UPLOAD: No valid pages to upload\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }

	  // Now perform the actual upload
	  sprintf(debugBuffer, "[%lu] UPLOAD: Starting upload of %d pages\r\n", HAL_GetTick(), totalPages);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // Reset Arduino and establish sync
	  if (!ResetAndSyncArduino(MAX_RESET_ATTEMPTS)) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Failed to sync with Arduino\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }

	  // Verify device signature
	  if (!VerifySignature()) {
		return false;
	  }

	  // Enter programming mode
	  if (!EnterProgrammingMode()) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Failed to enter programming mode\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return false;
	  }

	  // Erase the chip
	  if (!EraseChip()) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Failed to erase chip\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		LeaveProgrammingMode(); // Try to exit gracefully
		return false;
	  }

	  // Upload all prepared pages
	  uint16_t i;
	  for (i = 0; i < totalPages; i++) {
		// Load address for this page
		if (!LoadAddress(pageAddresses[i])) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Failed to load address 0x%04X\r\n",
				  HAL_GetTick(), pageAddresses[i]);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  break;
		}

		// Program the page using the special method for better timing
		if (!ProgramPageSpecial(preparedPages[i], pageSizes[i], 'F')) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Failed to program page at 0x%04X\r\n",
				  HAL_GetTick(), pageAddresses[i]);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  break;
		}

		// Print progress
		uint8_t percent = ((i + 1) * 100) / totalPages;
		sprintf(debugBuffer, "[%lu] UPLOAD: Progress %d%% - Programmed page %d/%d at 0x%04X\r\n",
				HAL_GetTick(), percent, i+1, totalPages, pageAddresses[i]);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  // Check if upload was complete
	  if (i == totalPages) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Hex file upload complete\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		success = true;
	  } else {
		sprintf(debugBuffer, "[%lu] UPLOAD: Upload failed at page %d/%d\r\n",
				HAL_GetTick(), i+1, totalPages);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  }

	  // Leave programming mode
	  if (!LeaveProgrammingMode()) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Failed to leave programming mode\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		success = false;
	  }

	  // Final reset to run the newly uploaded program
	  if (success) {
		sprintf(debugBuffer, "[%lu] UPLOAD: Resetting Arduino to run new program\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		ResetArduino();
	  }

	  return success;
	}


	/**
	  * @brief Pre-processes the hex file into pages for faster upload
	  * @param preparedPages: Array to store prepared page data
	  * @param pageSizes: Array to store size of each page
	  * @param pageAddresses: Array to store starting address of each page
	  * @param maxPages: Maximum number of pages to prepare
	  * @retval Number of pages prepared
	  */
	/**
	  * @brief Pre-processes the hex file into pages for faster upload, ensuring correct byte ordering
	  * @param preparedPages: Array to store prepared page data
	  * @param pageSizes: Array to store size of each page
	  * @param pageAddresses: Array to store starting address of each page
	  * @param maxPages: Maximum number of pages to prepare
	  * @retval Number of pages prepared
	  */
	/**
	  * @brief Pre-processes the hex file into pages for faster upload
	  * @param preparedPages: Array to store prepared page data
	  * @param pageSizes: Array to store size of each page
	  * @param pageAddresses: Array to store starting address of each page
	  * @param maxPages: Maximum number of pages to prepare
	  * @retval Number of pages prepared
	  */
	//uint16_t PrepareHexFilePages(uint8_t preparedPages[][FLASH_PAGE_SIZE],
	//                            uint16_t pageSizes[],
	//                            uint16_t pageAddresses[],
	//                            uint16_t maxPages)
	//{
	//  uint16_t totalPages = 0;
	//  uint16_t currentAddress = 0;
	//  uint16_t bytesRemaining = sizeof(program_data);
	//
	//  sprintf(debugBuffer, "[%lu] UPLOAD: Pre-processing binary program data (size: %u bytes)...\r\n",
	//          HAL_GetTick(), bytesRemaining);
	//  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	//
	//  // Process all data in chunks of FLASH_PAGE_SIZE
	//  while (bytesRemaining > 0 && totalPages < maxPages) {
	//    // Skip bootloader area
	//    if (currentAddress >= BOOT_SECTION_START) {
	//      sprintf(debugBuffer, "[%lu] UPLOAD: Skipping bootloader area at 0x%04X\r\n",
	//              HAL_GetTick(), currentAddress);
	//      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	//      break; // Stop processing as we've reached the bootloader area
	//    }
	//
	//    // Calculate bytes to copy for this page
	//    uint16_t bytesToCopy = (bytesRemaining > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : bytesRemaining;
	//
	//    // Copy data for this page
	//    memcpy(preparedPages[totalPages], &program_data[currentAddress], bytesToCopy);
	//
	//    // If we didn't fill the page, pad with 0xFF (erased flash state)
	//    if (bytesToCopy < FLASH_PAGE_SIZE) {
	//      memset(&preparedPages[totalPages][bytesToCopy], 0xFF, FLASH_PAGE_SIZE - bytesToCopy);
	//    }
	//
	//    // Store page information
	//    pageAddresses[totalPages] = currentAddress;
	//    pageSizes[totalPages] = FLASH_PAGE_SIZE; // Always send full pages for consistent programming
	//
	//    // Debug output
	//    sprintf(debugBuffer, "[%lu] UPLOAD: Prepared page %d: address=0x%04X, size=%d bytes\r\n",
	//            HAL_GetTick(), totalPages, currentAddress, FLASH_PAGE_SIZE);
	//    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	//
	//    // Update counters
	//    currentAddress += bytesToCopy;
	//    bytesRemaining -= bytesToCopy;
	//    totalPages++;
	//
	//    // Progress indicator every few pages
	//    if (totalPages % 10 == 0) {
	//      sprintf(debugBuffer, "[%lu] UPLOAD: Processed %d pages...\r\n", HAL_GetTick(), totalPages);
	//      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	//    }
	//  }
	//
	//  sprintf(debugBuffer, "[%lu] UPLOAD: Pre-processing complete: %d pages prepared\r\n",
	//          HAL_GetTick(), totalPages);
	//  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	//
	//  return totalPages;
	//}
	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	/* USER CODE BEGIN PFP */

	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */

	/* USER CODE END 0 */

	/**
	  * @brief  The application entry point.
	  * @retval int
	  *//* Add these external variables at the file scope before main() */
	uint8_t* program_data_ptr = NULL;    // Will point to our dynamic program data
	uint32_t program_data_size = 0;      // Size of the received program data

	/**
	  * @brief  The application entry point with wireless programming support
	  * @retval int
	  *//* Add these external variables at the file scope before main() */

	/**
	  * @brief  The application entry point with wireless programming support
	  * @retval int
	  */
int main(void)
	{
	  /* MCU Configuration--------------------------------------------------------*/
	  HAL_Init();
	  SystemClock_Config();

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_USART1_UART_Init();  // Arduino UART
	  MX_USART2_UART_Init();  // Bluetooth TX (to laptop)
	  MX_USART6_UART_Init();  // Bluetooth RX (from laptop)

	  // Buffer for receiving commands and data from Bluetooth
	  uint8_t btRxBuffer[RX_BUFFER_SIZE];
	  uint8_t btTxBuffer[RX_BUFFER_SIZE];
	  static uint8_t dynamic_program_data[MAX_FLASH_ADDRESS]; // Buffer to store incoming hex file

	  // State machine states
	  typedef enum {
		STATE_IDLE,
		STATE_RECEIVING_HEX,
		STATE_PROGRAMMING,
		STATE_PASSTHROUGH
	  } SystemState;

	  SystemState currentState = STATE_IDLE;

	  // Initialize Arduino reset pin to high (not in reset)
	  HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_SET);
	  HAL_Delay(500); // Initial delay to stabilize

	  // Send startup message via Bluetooth
	  sprintf(debugBuffer, "[%lu] SYSTEM: STM32 Wireless Arduino Programmer started\r\n", HAL_GetTick());
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"SYSTEM:READY", 12, UART_TIMEOUT);

	  // Main loop
	  while (1) {
		// Check for commands from Bluetooth (UART6)
		uint8_t commandBuffer[32] = {0};
		uint16_t cmdSize = 0;

		// Try to receive a command byte
		if (HAL_UART_Receive(&huart6, &commandBuffer[0], 1, 10) == HAL_OK) {
		  cmdSize = 1;

		  // Try to receive more bytes that might be in the buffer (with short timeout)
		  while (cmdSize < sizeof(commandBuffer) - 1 &&
				 HAL_UART_Receive(&huart6, &commandBuffer[cmdSize], 1, 5) == HAL_OK) {
			cmdSize++;
		  }

		  // Null-terminate the command
		  commandBuffer[cmdSize] = '\0';

		  // Log the received command
		  sprintf(debugBuffer, "[%lu] BT_CMD: Received command: %s\r\n", HAL_GetTick(), commandBuffer);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		  // Process commands based on current state
		  switch (currentState) {
			case STATE_IDLE:
			  // Check for signature request command
			  if (strncmp((char*)commandBuffer, "s", 1) == 0) {
				sprintf(debugBuffer, "[%lu] SYSTEM: Signature request received\r\n", HAL_GetTick());
				HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

				// Reset Arduino and establish sync
				if (ResetAndSyncArduino(MAX_RESET_ATTEMPTS)) {
				  // Verify signature
				  uint8_t response[3];
				  bool success = SendCommandAndWaitForSync(STK_READ_SIGN, NULL, 0, response, 3);

				  if (success) {
					// Format signature response
					sprintf((char*)btTxBuffer, "SIG:%02X,%02X,%02X", response[0], response[1], response[2]);
					HAL_UART_Transmit(&UART_DEBUG, btTxBuffer, strlen((char*)btTxBuffer), UART_TIMEOUT);

					// Log signature
					sprintf(debugBuffer, "[%lu] SYSTEM: Signature sent: %s\r\n", HAL_GetTick(), btTxBuffer);
					HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
				  } else {
					// Send failure response
					HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"SIG:FAIL", 8, UART_TIMEOUT);
				  }
				} else {
				  // Send sync failure response
				  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"SIG:NOSYNC", 10, UART_TIMEOUT);
				}
			  }
			  // Check for programming start command
			  else if (strncmp((char*)commandBuffer, "p", 1) == 0) {
				sprintf(debugBuffer, "[%lu] SYSTEM: Programming request received\r\n", HAL_GetTick());
				HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

				// Reset program data buffer
				program_data_size = 0;
				memset(dynamic_program_data, 0xFF, sizeof(dynamic_program_data));

				// Set the global pointer to our buffer for the upload functions to use
				program_data_ptr = dynamic_program_data;

				// Switch to hex receiving state
				currentState = STATE_RECEIVING_HEX;
				HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PROG:READY", 10, UART_TIMEOUT);
			  }
			  // Check for passthrough mode command
			  else if (strncmp((char*)commandBuffer, "c", 1) == 0) {
				sprintf(debugBuffer, "[%lu] SYSTEM: Entering passthrough mode\r\n", HAL_GetTick());
				HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

				// Switch to passthrough state
				currentState = STATE_PASSTHROUGH;
				HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PASS:OK", 7, UART_TIMEOUT);
			  }
			  break;
			case STATE_RECEIVING_HEX:
			  // Check for end of hex transmission
			  if (strncmp((char*)commandBuffer, "e", 1) == 0) {
			    sprintf(debugBuffer, "[%lu] SYSTEM: Hex file reception complete, size: %lu bytes\r\n",
			            HAL_GetTick(), program_data_size);
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

			    // Switch to programming state
			    currentState = STATE_PROGRAMMING;
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PROG:START", 10, UART_TIMEOUT);

			    // Execute the upload process
			    bool uploadResult = UploadHexFile();

			    // Report status
			    if (uploadResult) {
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PROG:SUCCESS", 12, UART_TIMEOUT);
			      sprintf(debugBuffer, "[%lu] SYSTEM: Upload successful - Arduino is now running the new program\r\n", HAL_GetTick());
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			    } else {
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PROG:FAIL", 9, UART_TIMEOUT);
			      sprintf(debugBuffer, "[%lu] SYSTEM: Upload failed - Check connections and retry\r\n", HAL_GetTick());
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			    }

			    // Reset the program data pointer for safety
			    program_data_ptr = NULL;
			    program_data_size = 0;

			    // Return to idle state
			    currentState = STATE_IDLE;
			  }
			  // Process simple 'h' command - receive the whole file at once
			  else if (strncmp((char*)commandBuffer, "h", 1) == 0) {
			    // First, receive the 4-byte file size
			    uint32_t expected_size = 0;

			    // Log that we're waiting for the size information
			    sprintf(debugBuffer, "[%lu] SYSTEM: Waiting for file size information\r\n", HAL_GetTick());
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);



			    // Receive the 4-byte file size (timeout: 1 second)
			    if (HAL_UART_Receive(&huart6, (uint8_t*)&expected_size, 4, 1000) != HAL_OK) {
			      sprintf(debugBuffer, "[%lu] SYSTEM: Failed to receive file size\r\n", HAL_GetTick());
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			      return;
			    }

			    // Verify size is valid
			    if (expected_size == 0 || expected_size > sizeof(dynamic_program_data)) {
			      sprintf(debugBuffer, "[%lu] SYSTEM: Invalid file size: %lu bytes (max: %lu)\r\n",
			              HAL_GetTick(), expected_size, sizeof(dynamic_program_data));
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			      return;
			    }

			    // Log that we're about to receive the file
			    sprintf(debugBuffer, "[%lu] SYSTEM: Receiving %lu bytes of binary data...\r\n", HAL_GetTick(), expected_size);
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

			    // Calculate a reasonable timeout based on file size (10ms per byte + 2s base)
			    uint32_t receive_timeout = expected_size * 10 + 2000;

			    // Receive the entire file in one operation
			    HAL_StatusTypeDef receive_status = HAL_UART_Receive(&huart6, dynamic_program_data, expected_size, receive_timeout);

			    if (receive_status == HAL_OK) {
			      // Update the program data size
			      program_data_size = expected_size;
			      program_data_ptr = dynamic_program_data;

			      sprintf(debugBuffer, "[%lu] SYSTEM: Successfully received %lu bytes of binary data\r\n",
			              HAL_GetTick(), program_data_size);
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			    } else {
			      // Handle receive error
			      sprintf(debugBuffer, "[%lu] SYSTEM: Failed to receive binary data (status: %d)\r\n",
			              HAL_GetTick(), receive_status);
			      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			    }
			  }
			  // Cancel hex reception
			  else if (strncmp((char*)commandBuffer, "cancel", 6) == 0) {
			    currentState = STATE_IDLE;
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"HEX:CANCELLED", 13, UART_TIMEOUT);

			    sprintf(debugBuffer, "[%lu] SYSTEM: Hex file reception cancelled\r\n", HAL_GetTick());
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			  }
			  break;

			case STATE_PROGRAMMING:
			  // This state is handled by the UploadHexFile function
			  // We should not receive commands while in this state
			  // But if we do, just log them
			  sprintf(debugBuffer, "[%lu] SYSTEM: Ignoring command during programming\r\n", HAL_GetTick());
			  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
			  break;

			case STATE_PASSTHROUGH:
			  // Exit passthrough mode command
			  if (strncmp((char*)commandBuffer, "x", 1) == 0) {
			    sprintf(debugBuffer, "[%lu] SYSTEM: Exiting passthrough mode\r\n", HAL_GetTick());
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

			    // Stop UART interrupt reception
			    HAL_UART_AbortReceive_IT(&UART_ARDUINO);

			    currentState = STATE_IDLE;
			    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)"PASS:EXIT", 9, UART_TIMEOUT);
			  } else {
			    // Forward data from Bluetooth to Arduino
			    HAL_UART_Transmit(&UART_ARDUINO, commandBuffer, cmdSize, UART_TIMEOUT);
			  }
			  break;
		  }
		}

		// In passthrough mode, check for data from Arduino and forward to Bluetooth
		// In passthrough mode, check for data from Arduino and forward to Bluetooth
		// In passthrough mode, check for data from Arduino and forward to Bluetooth
		if (currentState == STATE_PASSTHROUGH) {
		  // Start interrupt mode if we just entered passthrough state
		  static bool interruptModeStarted = false;
		  if (!interruptModeStarted) {
		    StartUartInterruptMode();
		    interruptModeStarted = true;
		  }

		  // Check if we have received data from Arduino via interrupt
		  if (uartRxReady) {
		    // Process all available bytes in the circular buffer
		    while (rxHead != rxTail) {
		      // Get a byte from the buffer
		      uint8_t byte = uartRxCircBuffer[rxTail];
		      rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

		      // Forward the byte to the debug UART
		      HAL_UART_Transmit(&UART_DEBUG, &byte, 1, UART_TIMEOUT);
		    }

		    // Reset the flag after processing all available bytes
		    if (rxHead == rxTail) {
		      uartRxReady = false;
		    }
		  }
		} else {
		  // Reset interrupt mode flag when not in passthrough mode
		  static bool interruptModeStarted = false;
		  if (interruptModeStarted) {
		    HAL_UART_AbortReceive_IT(&UART_ARDUINO);
		    interruptModeStarted = false;

		    sprintf(debugBuffer, "[%lu] SYSTEM: UART interrupt mode stopped\r\n", HAL_GetTick());
		    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  }
		}

//		sprintf(debugBuffer, "[%lu] SYSTEM: Exiting While mode\r\n", HAL_GetTick());
//		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		// Blink status LED or perform other maintenance tasks
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Assuming LED is on PC13
	  }
	}



	/*
	 * You'll need to modify PrepareHexFilePages() as follows to use the dynamic data:
	 */
	uint16_t PrepareHexFilePages(uint8_t preparedPages[][FLASH_PAGE_SIZE],
								uint16_t pageSizes[],
								uint16_t pageAddresses[],
								uint16_t maxPages)
	{
	  uint16_t totalPages = 0;
	  uint16_t currentAddress = 0;

	  // Only use the dynamically received data
	  uint8_t* data_source = program_data_ptr;
	  uint16_t bytesRemaining = program_data_size;

	  // Safety check to ensure we have valid data
	  if (data_source == NULL || bytesRemaining == 0) {
		sprintf(debugBuffer, "[%lu] UPLOAD: No valid program data available\r\n", HAL_GetTick());
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		return 0;
	  }

	  sprintf(debugBuffer, "[%lu] UPLOAD: Pre-processing binary program data (size: %u bytes)...\r\n",
			  HAL_GetTick(), bytesRemaining);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  // Process all data in chunks of FLASH_PAGE_SIZE
	  while (bytesRemaining > 0 && totalPages < maxPages) {
		// Skip bootloader area
		if (currentAddress >= BOOT_SECTION_START) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Skipping bootloader area at 0x%04X\r\n",
				  HAL_GetTick(), currentAddress);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		  break; // Stop processing as we've reached the bootloader area
		}

		// Calculate bytes to copy for this page
		uint16_t bytesToCopy = (bytesRemaining > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : bytesRemaining;

		// Copy data for this page
		memcpy(preparedPages[totalPages], &data_source[currentAddress], bytesToCopy);

		// If we didn't fill the page, pad with 0xFF (erased flash state)
		if (bytesToCopy < FLASH_PAGE_SIZE) {
		  memset(&preparedPages[totalPages][bytesToCopy], 0xFF, FLASH_PAGE_SIZE - bytesToCopy);
		}

		// Store page information
		pageAddresses[totalPages] = currentAddress;
		pageSizes[totalPages] = FLASH_PAGE_SIZE; // Always send full pages for consistent programming

		// Debug output
		sprintf(debugBuffer, "[%lu] UPLOAD: Prepared page %d: address=0x%04X, size=%d bytes\r\n",
				HAL_GetTick(), totalPages, currentAddress, FLASH_PAGE_SIZE);
		HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

		// Update counters
		currentAddress += bytesToCopy;
		bytesRemaining -= bytesToCopy;
		totalPages++;

		// Progress indicator every few pages
		if (totalPages % 10 == 0) {
		  sprintf(debugBuffer, "[%lu] UPLOAD: Processed %d pages...\r\n", HAL_GetTick(), totalPages);
		  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
		}
	  }

	  sprintf(debugBuffer, "[%lu] UPLOAD: Pre-processing complete: %d pages prepared\r\n",
			  HAL_GetTick(), totalPages);
	  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

	  return totalPages;
	}


	/**
	  * @brief System Clock Configuration
	  * @retval None
	  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif

