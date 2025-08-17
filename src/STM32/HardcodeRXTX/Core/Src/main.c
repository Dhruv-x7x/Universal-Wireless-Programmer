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
#include "arduino_hex.h"

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
#define UART_DEBUG      huart2  // Connected to PL2303 for debug via Putty

/* Private variables ---------------------------------------------------------*/
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[RX_BUFFER_SIZE];
char debugBuffer[DEBUG_BUFFER_SIZE];
uint32_t tickStart;
uint8_t dtrState = 0;
uint8_t prevDtrState = 0;

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

/* Private user code ---------------------------------------------------------*/

/**
  * @brief Reset Arduino UNO into bootloader mode with precise timing for ATmega328P
  * @retval None
  */
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  // Initialize Arduino reset pin to high (not in reset)
  HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(500); // Initial delay to stabilize

  // Send startup message to debug UART
  sprintf(debugBuffer, "[%lu] SYSTEM: STM32 USB-Serial bridge for Arduino UNO started\r\n", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // UNO bootloader works at 115200 baud
  sprintf(debugBuffer, "[%lu] SYSTEM: Target: Arduino UNO (ATmega328P) at 115200 baud\r\n", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // Attempt to upload hex file
  sprintf(debugBuffer, "[%lu] SYSTEM: Starting Arduino hex file upload\r\n", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  if (UploadHexFile()) {
    sprintf(debugBuffer, "[%lu] SYSTEM: Upload successful - Arduino is now running the new program\r\n", HAL_GetTick());
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
  } else {
    sprintf(debugBuffer, "[%lu] SYSTEM: Upload failed - Check connections and retry\r\n", HAL_GetTick());
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
  }

  // Infinite loop after completing upload
  while (1) {
    // Check for data from Arduino
    if (HAL_UART_Receive(&UART_ARDUINO, rxBuffer, 1, 10) == HAL_OK) {
      // Got at least one byte, check for more with short timeout
      uint16_t rxSize = 1;

      // Try to receive more bytes that might be in the buffer (with short timeout)
      while (rxSize < RX_BUFFER_SIZE &&
             HAL_UART_Receive(&UART_ARDUINO, &rxBuffer[rxSize], 1, 2) == HAL_OK) {
        rxSize++;
      }

      // Log data received from Arduino
      LogMessage(LOG_ARDUINO_TO_STM, rxBuffer, rxSize);
    }

    // Blink an LED or perform other maintenance tasks
    HAL_Delay(100);
  }
}

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
  uint8_t respBuffer[2 + RX_BUFFER_SIZE]; // INSYNC + OK + any response data
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

  // Log command with detailed hex values
  sprintf(debugBuffer, "[%lu] UPLOAD: Sending command 0x%02X with %d params + Sync_CRC_EOP\r\n",
          HAL_GetTick(), cmd, paramSize);
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // For detailed debugging, log the exact bytes being sent
  sprintf(debugBuffer, "[%lu] UPLOAD: Raw bytes: ", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  for (int i = 0; i < totalSize; i++) {
    sprintf(debugBuffer, "0x%02X ", cmdBuffer[i]);
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
  }

  sprintf(debugBuffer, "\r\n");
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // Flush any existing data in the UART buffers
  HAL_UART_AbortReceive(&UART_ARDUINO);

  // Wait for UART to stabilize
  HAL_Delay(10);

  // Send command + parameters + Sync_CRC_EOP
  HAL_UART_Transmit(&UART_ARDUINO, cmdBuffer, totalSize, UART_TIMEOUT);

  // Try to receive first byte with longer timeout (INSYNC)
  if (HAL_UART_Receive(&UART_ARDUINO, &respBuffer[0], 1, SYNC_RESPONSE_TIMEOUT) == HAL_OK) {
    // We got the first byte, check if it's STK_INSYNC
    if (respBuffer[0] == STK_INSYNC) {
      sprintf(debugBuffer, "[%lu] UPLOAD: Received STK_INSYNC (0x%02X)\r\n", HAL_GetTick(), respBuffer[0]);
      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

      // Receive additional response data if expected
      if (response != NULL && responseSize > 0) {
        // Process response data one byte at a time with logging
        for (uint8_t i = 0; i < responseSize; i++) {
          if (HAL_UART_Receive(&UART_ARDUINO, &response[i], 1, UART_TIMEOUT) != HAL_OK) {
            sprintf(debugBuffer, "[%lu] UPLOAD: Failed to receive response byte %d\r\n", HAL_GetTick(), i+1);
            HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
            return false;
          }

          sprintf(debugBuffer, "[%lu] UPLOAD: Response byte %d: 0x%02X\r\n", HAL_GetTick(), i+1, response[i]);
          HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        }
      }

      // Try to get the final byte (STK_OK) with dedicated timeout
      if (HAL_UART_Receive(&UART_ARDUINO, &respBuffer[1], 1, SECOND_BYTE_TIMEOUT) == HAL_OK) {
        sprintf(debugBuffer, "[%lu] UPLOAD: Received final byte: 0x%02X\r\n", HAL_GetTick(), respBuffer[1]);
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

        if (respBuffer[1] == STK_OK) {
          sprintf(debugBuffer, "[%lu] UPLOAD: Command 0x%02X completed successfully!\r\n", HAL_GetTick(), cmd);
          HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
          success = true;
        } else {
          sprintf(debugBuffer, "[%lu] UPLOAD: Final byte is not STK_OK\r\n", HAL_GetTick());
          HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        }
      } else {
        sprintf(debugBuffer, "[%lu] UPLOAD: No final byte received after STK_INSYNC\r\n", HAL_GetTick());
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
      }
    } else {
      sprintf(debugBuffer, "[%lu] UPLOAD: Received 0x%02X instead of STK_INSYNC\r\n", HAL_GetTick(), respBuffer[0]);
      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
    }
  } else {
    sprintf(debugBuffer, "[%lu] UPLOAD: No response from bootloader\r\n", HAL_GetTick());
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
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
  uint8_t cmd[2] = {STK_ENTER_PROGMODE, 0x20};  // Command + Sync_CRC_EOP
  uint8_t response[2] = {0, 0};
  bool success = false;

  // Log action
  sprintf(debugBuffer, "[%lu] UPLOAD: Sending enter programming mode command\r\n", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // Flush any existing data in the UART buffers
  HAL_UART_AbortReceive(&UART_ARDUINO);

  // Wait for UART to stabilize
  HAL_Delay(10);

  // Send command
  HAL_UART_Transmit(&UART_ARDUINO, cmd, 2, UART_TIMEOUT);

  // Log the bytes sent
  sprintf(debugBuffer, "[%lu] UPLOAD: Sent bytes: 0x%02X 0x%02X\r\n", HAL_GetTick(), cmd[0], cmd[1]);
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  // Try to receive first byte with longer timeout
  if (HAL_UART_Receive(&UART_ARDUINO, &response[0], 1, SYNC_RESPONSE_TIMEOUT) == HAL_OK) {
    // We got the first byte, check if it's STK_INSYNC
    if (response[0] == STK_INSYNC) {
      sprintf(debugBuffer, "[%lu] UPLOAD: Received STK_INSYNC (0x%02X)\r\n", HAL_GetTick(), response[0]);
      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

      // Try to get the second byte (STK_OK) with dedicated timeout
      if (HAL_UART_Receive(&UART_ARDUINO, &response[1], 1, SECOND_BYTE_TIMEOUT) == HAL_OK) {
        sprintf(debugBuffer, "[%lu] UPLOAD: Received second byte: 0x%02X\r\n", HAL_GetTick(), response[1]);
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

        if (response[1] == STK_OK) {
          sprintf(debugBuffer, "[%lu] UPLOAD: Programming mode entered successfully!\r\n", HAL_GetTick());
          HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
          success = true;
        } else {
          sprintf(debugBuffer, "[%lu] UPLOAD: Second byte is not STK_OK\r\n", HAL_GetTick());
          HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        }
      } else {
        sprintf(debugBuffer, "[%lu] UPLOAD: No second byte received after STK_INSYNC\r\n", HAL_GetTick());
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
      }
    } else {
      sprintf(debugBuffer, "[%lu] UPLOAD: Received 0x%02X instead of STK_INSYNC\r\n", HAL_GetTick(), response[0]);
      HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
    }
  } else {
    sprintf(debugBuffer, "[%lu] UPLOAD: No response from bootloader\r\n", HAL_GetTick());
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
  }

  return success;
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
  uint32_t currentAddress = 0;
  uint16_t pageStart = 0;
  uint8_t pageBuffer[FLASH_PAGE_SIZE];
  uint16_t pageSize = 0;
  bool success = false;

  sprintf(debugBuffer, "[%lu] UPLOAD: Starting hex file upload (size: %u bytes)\r\n",
          HAL_GetTick(), sizeof(arduino_hex_data));
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

  // Process hex file in pages
  while (currentAddress < sizeof(arduino_hex_data)) {
    // Check if we need to start a new page
    if (pageSize == 0) {
      pageStart = currentAddress;

      // Load address for the new page
      if (!LoadAddress(pageStart)) {
        sprintf(debugBuffer, "[%lu] UPLOAD: Failed to load address 0x%04X\r\n",
                HAL_GetTick(), pageStart);
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        break;
      }
    }

    // Add byte to page buffer
    pageBuffer[pageSize++] = arduino_hex_data[currentAddress++];

    // Print progress
    PrintUploadProgress(currentAddress, sizeof(arduino_hex_data));

    // When page is full or we've reached the end, write it
    if (pageSize == FLASH_PAGE_SIZE || currentAddress == sizeof(arduino_hex_data)) {
      // Skip if we're trying to write to bootloader section
      if (pageStart >= BOOT_SECTION_START) {
        sprintf(debugBuffer, "[%lu] UPLOAD: Skipping bootloader area at 0x%04X\r\n",
                HAL_GetTick(), pageStart);
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        pageSize = 0; // Reset for next page
        continue;
      }

      // Program the page
      if (!ProgramPage(pageBuffer, pageSize, MEMORY_TYPE_FLASH)) {
        sprintf(debugBuffer, "[%lu] UPLOAD: Failed to program page at 0x%04X\r\n",
                HAL_GetTick(), pageStart);
        HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
        break;
      }

      // Reset page buffer for next page
      pageSize = 0;
    }
  }

  // Check if upload was complete
  if (currentAddress == sizeof(arduino_hex_data)) {
    sprintf(debugBuffer, "[%lu] UPLOAD: Hex file upload complete\r\n", HAL_GetTick());
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
    success = true;
  } else {
    sprintf(debugBuffer, "[%lu] UPLOAD: Upload failed at address 0x%04lX\r\n",
            HAL_GetTick(), currentAddress);
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();

  // Send error message to debug UART
  sprintf(debugBuffer, "[%lu] ERROR: System halted due to critical error\r\n", HAL_GetTick());
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);

  while (1)
  {
  }
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
  // Report assert failure to debug UART
  sprintf(debugBuffer, "[%lu] ASSERT FAILED: File %s on line %d\r\n",
          HAL_GetTick(), file, line);
  HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)debugBuffer, strlen(debugBuffer), UART_TIMEOUT);
}
#endif /* USE_FULL_ASSERT */
