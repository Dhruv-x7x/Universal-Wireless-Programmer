#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_audio.h"
#include "usbh_cdc.h"
#include "usbh_msc.h"
#include "usbh_hid.h"
#include "usbh_mtp.h"
#include <string.h> /* Include for string functions */
#include <stdio.h>  /* Include for printf */
#include "usart.h"
#include "stm32f4xx_hal.h"
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;
extern ApplicationTypeDef Appli_state;
#define MAX_BOARD_NAME_LENGTH 50
#define STK_OK             0x10
#define STK_INSYNC         0x14
#define STK_CRC_EOP        0x20
#define STK_GET_SYNC       0x30
#define STK_LOAD_ADDRESS   0x55
#define STK_PROGRAM_PAGE   0x64
#define STK_READ_PAGE      0x74
#define STK_READ_SIGN      0x75
#define STK_LEAVE_PROGMODE 0x51
#define ARDUINO_RESET_PULSE_DURATION   500  // ms
#define ARDUINO_BOOTLOADER_TIMEOUT     1000 // ms
#define PROG_PAGE_SIZE                 128  // bytes per page for Atmega328P
#define MAX_STK_RESPONSE_LENGTH        32
#define HEX_LINE_SIZE 23  // 21 bytes of data + 2 bytes for \r\n
#define DATA_BYTES_PER_LINE 16  // 16 data bytes per line
#define LINES_PER_PAGE (PAGE_SIZE / DATA_BYTES_PER_LINE)  // 8 lines per page
#define PAGE_SIZE 128  // 128 bytes of actual data

#define HEX_RECORD_TYPE_DATA           0
#define HEX_RECORD_TYPE_EOF            1
#define HEX_RECORD_TYPE_EXT_SEG_ADDR   2
#define HEX_RECORD_TYPE_START_SEG_ADDR 3
#define HEX_RECORD_TYPE_EXT_LINEAR_ADDR 4
#define HEX_RECORD_TYPE_START_LINEAR_ADDR 5

#define ARDUINO_RESET_PIN    GPIO_PIN_3
#define ARDUINO_RESET_PORT   GPIOB

static ArduinoProgrammingState arduino_prog_state = ARDUINO_PROG_IDLE;

static uint8_t stk_response_buffer[MAX_STK_RESPONSE_LENGTH];

extern char detected_board_name[MAX_BOARD_NAME_LENGTH];  // Use the extern defined in main.c

static USBH_StatusTypeDef program_page(uint16_t address, uint8_t *data, uint16_t length);
static uint8_t hex_to_byte(const uint8_t *hex);

static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

void MX_USB_HOST_Init(void)
{
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
  printf("USB Host Init Successful\r\n");
}

void MX_USB_HOST_Process(void)
{
  USBH_Process(&hUsbHostFS);
}
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  // Reset programming state if device disconnected during programming
  if (arduino_prog_state != ARDUINO_PROG_IDLE &&
      arduino_prog_state != ARDUINO_PROG_COMPLETED) {
      arduino_prog_state = ARDUINO_PROG_ERROR;
      printf("USB device disconnected during programming\r\n");
  }
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
        USBH_DevDescTypeDef *dev_desc = &hUsbHostFS.device.DevDesc;

        // Classify and return the device's board name
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
        strncpy(board_name, "No Device", MAX_BOARD_NAME_LENGTH - 1);
        board_name[MAX_BOARD_NAME_LENGTH - 1] = '\0';
    }

    return board_name;
}

const char* classify_usb_device(uint16_t vid, uint16_t pid, uint8_t classCode) {
    static char detected_board_name[MAX_BOARD_NAME_LENGTH];  // Static so it persists after the function exits

    if (vid == 0x2341 && (pid == 0x0043 || pid == 0x0010)) {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "Arduino UNO R3");
    } else if (vid == 0x2341 && pid == 0x0001) {
        // This is the Arduino in bootloader mode (different PID)
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "Arduino UNO R3 (Bootloader)");
    } else if (classCode == 0x02) {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "USB Serial Device");
    } else {
        snprintf(detected_board_name, MAX_BOARD_NAME_LENGTH, "Unknown Device");
    }

    return detected_board_name;
}

#define ARDUINO_RESET_PIN    GPIO_PIN_3
#define ARDUINO_RESET_PORT   GPIOB

USBH_StatusTypeDef trigger_arduino_bootloader(void)
{

    printf("Triggering Arduino bootloader...\r\n");

    HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(ARDUINO_RESET_PULSE_DURATION);
    HAL_GPIO_WritePin(ARDUINO_RESET_PORT, ARDUINO_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(300);

    arduino_prog_state = ARDUINO_PROG_RESET_SENT;
    return USBH_OK;
}
// This approach uses lower-level USB CDC control rather than the higher-level USBH functions
USBH_StatusTypeDef send_stk_command_raw(uint8_t cmd, uint8_t *params, uint8_t param_len) {
    uint8_t tx_buffer[32];
    uint8_t length = 0;

    // Start with command
    tx_buffer[length++] = cmd;

    // Add parameters if any
    if (params != NULL && param_len > 0) {
        memcpy(&tx_buffer[length], params, param_len);
        length += param_len;
    }

    // End with CRC/EOP
    tx_buffer[length++] = STK_CRC_EOP;

    printf("Sending raw STK command: ");
    for (int i = 0; i < length; i++) {
        printf("0x%02X ", tx_buffer[i]);
    }
    printf("\r\n");

    // Try direct write to USB CDC interface
    CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *)hUsbHostFS.pActiveClass->pData;
    if (CDC_Handle != NULL) {
        // Send data directly using low-level transfer
        USBH_StatusTypeDef status = USBH_BulkSendData(&hUsbHostFS,
                                                       tx_buffer,
                                                       length,
                                                       CDC_Handle->DataItf.OutPipe,
                                                       1);
        if(status == USBH_OK){
            arduino_prog_state = ARDUINO_WAIT_RESPONSE;
      	  printf("Sent command succesffuly");

        }
        return status;
    }

    return USBH_FAIL;
}

// Similarly with receiving
USBH_StatusTypeDef receive_stk_response_raw(uint8_t *rx_buffer, uint8_t length) {
    CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *)hUsbHostFS.pActiveClass->pData;

    if (CDC_Handle != NULL) {
        // Set up receive
        USBH_StatusTypeDef status = USBH_BulkReceiveData(&hUsbHostFS,
                                                         rx_buffer,
                                                         length,
                                                         CDC_Handle->DataItf.InPipe);

        // Wait for completion with timeout
        uint32_t start_time = HAL_GetTick();
        USBH_URBStateTypeDef URB_State;

        while ((HAL_GetTick() - start_time) < 500) {
            // Process USB tasks
            MX_USB_HOST_Process();

            // Check URB state
            URB_State = USBH_LL_GetURBState(&hUsbHostFS, CDC_Handle->DataItf.InPipe);
            print_urb_status(URB_State);
            if (URB_State == USBH_URB_DONE) {
                // Data received successfully
                uint16_t received_size = USBH_LL_GetLastXferSize(&hUsbHostFS, CDC_Handle->DataItf.InPipe);

                // Print received data
                printf("Response received (%d bytes): ", received_size);
                for (int i = 0; i < received_size && i < length; i++) {
                    printf("0x%02X ", rx_buffer[i]);
                }
                printf("\r\n");

                // Check if response is valid (STK_INSYNC + STK_OK)
                if (received_size >= 2 && rx_buffer[0] == STK_INSYNC && rx_buffer[1] == STK_OK) {
                    printf("SUCCESS! Bootloader responded correctly!\r\n");
                    arduino_prog_state = ARDUINO_PROG_BOOTLOADER_READY;
                } else {
                    printf("Invalid response. Expected 0x14 0x10\r\n");
                }

                return USBH_OK;
            }

            HAL_Delay(1);
        }

        // Timeout
        printf("Timeout waiting for bootloader response\r\n");
        return USBH_FAIL;
    }

    printf("CDC Handle is NULL\r\n");
    return USBH_FAIL;
}
void print_urb_status(USBH_URBStateTypeDef status) {
    switch (status) {
        case USBH_URB_IDLE:
            printf("URB Status: IDLE - No transfer in progress\r\n");
            break;
        case USBH_URB_DONE:
            printf("URB Status: DONE - Transfer completed successfully\r\n");
            break;
        case USBH_URB_NOTREADY:
            printf("URB Status: NOT READY - Device not ready for transfer\r\n");
            break;
        case USBH_URB_NYET:
            printf("URB Status: NYET - Device not ready yet\r\n");
            break;
        case USBH_URB_ERROR:
            printf("URB Status: ERROR - Transfer failed due to error\r\n");
            break;
        case USBH_URB_STALL:
            printf("URB Status: STALL - Endpoint stalled\r\n");
            break;
        default:
            printf("URB Status: UNKNOWN (0x%02X)\r\n", status);
            break;
    }
}

USBH_StatusTypeDef send_stk_command(uint8_t cmd, uint8_t *params, uint8_t param_len) {
//    printCDCstatus();

//    USBH_CDC_Stop(&hUsbHostFS);
//    printCDCstatus();

    // Create a local buffer for transmission
    uint8_t tx_buffer[32]; // Temporary buffer, 32 bytes should be enough

    // Set communication parameters (115200 8N1)
    CDC_LineCodingTypeDef linecoding;
    linecoding.b.dwDTERate = 115200;
    linecoding.b.bCharFormat = 0;
    linecoding.b.bParityType = 0;
    linecoding.b.bDataBits = 8;
    USBH_CDC_SetLineCoding(&hUsbHostFS, &linecoding);
    uint32_t timeout = HAL_GetTick() + 1000; // 1 second timeout
    USBH_HandleTypeDef *phost = &hUsbHostFS;
        CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *) phost->pActiveClass->pData;
        while (CDC_Handle->state != CDC_IDLE_STATE && CDC_Handle->state != CDC_TRANSFER_DATA) {
            // Process USB events to make the state machine progress
            MX_USB_HOST_Process();


            // Check for timeout
            if (HAL_GetTick() > timeout) {
                printf("Timeout waiting for CDC state to return to IDLE\r\n");
                return USBH_FAIL;
            }

            HAL_Delay(10); // Small delay to not hog CPU
        }
    // Build command packet
    tx_buffer[0] = cmd;

    // Add parameters if any
    if (params != NULL && param_len > 0) {
        memcpy(&tx_buffer[1], params, param_len);
    }

    // Add CRC/EOP
    tx_buffer[param_len + 1] = STK_CRC_EOP;

    printf("Sending STK command: 0x%02X 0x%02X\r\n",
           cmd, STK_CRC_EOP);

    // For simple cases like STK_GET_SYNC
    if (cmd == STK_GET_SYNC) {
        tx_buffer[0] = STK_GET_SYNC;
        tx_buffer[1] = STK_CRC_EOP;
    }
    printCDCstatus();

    if (USBH_CDC_Transmit(&hUsbHostFS, tx_buffer, param_len + 2) == USBH_OK) {
        printf("Command sent successfully\r\n");
        printCDCstatus();

        arduino_prog_state = ARDUINO_WAIT_RESPONSE;
        return USBH_OK;
    } else {
        printCDCstatus();

        printf("Failed to send command\r\n");
        arduino_prog_state = ARDUINO_PROG_ERROR;
        return USBH_FAIL;
    }
}

void printCDCstatus(){

    USBH_HandleTypeDef *phost = &hUsbHostFS;
    CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *) phost->pActiveClass->pData;
    if (CDC_Handle->state == CDC_IDLE_STATE) {
        printf("CDC is IDLE\r\n");
    } else if (CDC_Handle->state == CDC_SET_LINE_CODING_STATE) {
        printf("CDC is in SET LINE CODING STATE\r\n");
    } else if (CDC_Handle->state == CDC_GET_LAST_LINE_CODING_STATE) {
        printf("CDC is in GET LAST LINE CODING STATE\r\n");
    } else if (CDC_Handle->state == CDC_TRANSFER_DATA) {
        printf("CDC is in TRANSFER DATA state\r\n");
    } else if (CDC_Handle->state == CDC_ERROR_STATE) {
        printf("CDC is in ERROR STATE\r\n");
    } else {
        printf("Unknown CDC State\r\n");
    }
}

void receive_stk_response(uint8_t *rx_buffer) {
    // Clear the buffer first
    memset(rx_buffer, 0, RX_BUFF_SIZE);

    // Start the receive operation
    USBH_StatusTypeDef status = USBH_CDC_Receive(&hUsbHostFS, rx_buffer, 2);

    if (status != USBH_OK) {
        printf("Failed to start receive operation, status: %d\r\n", status);
        arduino_prog_state = ARDUINO_PROG_ERROR;
        return;
    }

    // Set up a timeout - use 500ms like in the Python code
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout = 500; // 500ms timeout

    // Poll until we actually get data
    while (1) {
        // Process USB host tasks while waiting
        MX_USB_HOST_Process();

        // Check if we've received data using GetLastReceivedDataSize
        uint16_t received_size = USBH_CDC_GetLastReceivedDataSize(&hUsbHostFS);

        if (received_size > 0) {
            // Data received!
            printf("Response received: 0x%02X 0x%02X\r\n", rx_buffer[0], rx_buffer[1]);

            if (rx_buffer[0] == STK_INSYNC && rx_buffer[1] == STK_OK) {
                printf("SUCCESS! Bootloader responded correctly!\r\n");
                arduino_prog_state = ARDUINO_PROG_BOOTLOADER_READY;
            } else {
                printf("Invalid response. Expected 0x14 0x10, got 0x%02X 0x%02X\r\n",
                      rx_buffer[0], rx_buffer[1]);
                arduino_prog_state = ARDUINO_PROG_ERROR;
            }
            break;
        }

        // Check for timeout
        if (HAL_GetTick() - start_time > timeout) {
            printf("Timeout waiting for bootloader response\r\n");
            arduino_prog_state = ARDUINO_PROG_ERROR;
            break;
        }

        // Small delay to prevent hogging the CPU
        HAL_Delay(1);
    }
}

uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum + 1);  // Two's complement
}

// Function to send one page (8 lines) in Intel HEX format
USBH_StatusTypeDef program_page(uint16_t address, uint8_t *data, uint16_t length) {
    USBH_StatusTypeDef status;
    uint8_t hex_buffer[HEX_LINE_SIZE]; // Buffer for one HEX line (23 bytes)

    // Ensure the data length is at least PAGE_SIZE
    if (length < PAGE_SIZE) {
        printf("Error: Data length is less than PAGE_SIZE\r\n");
        return USBH_FAIL;
    }

    // Generate and send HEX format lines
    for (int i = 0; i < LINES_PER_PAGE; i++) {
        // Clear the buffer
        memset(hex_buffer, 0, HEX_LINE_SIZE);

        // Start of HEX line
        hex_buffer[0] = ':';  // Start code
        hex_buffer[1] = DATA_BYTES_PER_LINE; // Byte count (16 bytes per line)
        hex_buffer[2] = (address >> 8) & 0xFF; // Address high byte
        hex_buffer[3] = address & 0xFF;  // Address low byte
        hex_buffer[4] = 0x00; // Record type (00 = data)

        // Copy 16 bytes of actual data
        memcpy(&hex_buffer[5], &data[i * DATA_BYTES_PER_LINE], DATA_BYTES_PER_LINE);

        // Calculate checksum
        hex_buffer[21] = calculate_checksum(&hex_buffer[1], 20);

        // Add line terminators
        hex_buffer[22] = '\r';
        hex_buffer[23] = '\n';

        // Send the HEX line over USB
        status =  USBH_CDC_Transmit(&hUsbHostFS, hex_buffer, HEX_LINE_SIZE);
       while(status != USBH_OK){
    	   status =  USBH_CDC_Transmit(&hUsbHostFS, hex_buffer, HEX_LINE_SIZE);
       }
        // Increment address by 16 bytes for next line
        address += DATA_BYTES_PER_LINE;
    }

    return USBH_OK;
}
static uint8_t hex_to_byte(const uint8_t *hex)
{
	uint8_t byte = 0;

	for (int i = 0; i < 2; i++) {
		byte <<= 4;
		if (hex[i] >= '0' && hex[i] <= '9') {
			byte |= (hex[i] - '0');
		}
		else if (hex[i] >= 'A' && hex[i] <= 'F') {
			byte |= (hex[i] - 'A' + 10);
		}
		else if (hex[i] >= 'a' && hex[i] <= 'f') {
			byte |= (hex[i] - 'a' + 10);
		}
	}

	return byte;
}
    	int parse_and_program_hex_line(const char *hex_line, uint32_t *base_address)
    	{
    	    uint8_t length, type;
    	    uint16_t address;
    	    uint8_t data[PROG_PAGE_SIZE];
    	    uint8_t checksum, calculated_checksum = 0;
    	    int match =0;

      	    // Parse length
    	    length = hex_to_byte((const uint8_t *)hex_line);
    	    calculated_checksum += length;
    	    hex_line += 2;

    	    // Parse address
    	    address = (uint16_t)hex_to_byte((const uint8_t *)hex_line) << 8;
    	    calculated_checksum += hex_to_byte((const uint8_t *)hex_line);
    	    hex_line += 2;

    	    address |= hex_to_byte((const uint8_t *)hex_line);
    	    calculated_checksum += hex_to_byte((const uint8_t *)hex_line);
    	    hex_line += 2;

    	    // Parse record type
    	    type = hex_to_byte((const uint8_t *)hex_line);
    	    calculated_checksum += type;
    	    hex_line += 2;

    	    // Handle different record types
    	    switch (type) {
    	        case HEX_RECORD_TYPE_DATA:
    	            // Data record
    	            for (uint8_t i = 0; i < length; i++) {
    	                data[i] = hex_to_byte((const uint8_t *)hex_line);
    	                calculated_checksum += data[i];
    	                hex_line += 2;
    	            }

    	            // Parse checksum
    	            checksum = hex_to_byte((const uint8_t *)hex_line);
    	            calculated_checksum = ~calculated_checksum + 1; // Two's complement

    	            if (checksum == calculated_checksum) {
    	               // printf("Checksum error in HEX line\r\n");
                        match = 1;
    	            }
    	            else{
    	            	match = -1;
    	            	break;
    	            }

    	            // Program the data if checksums match
    	            if(match==1){
						uint32_t full_address = *base_address | address;
						program_page((uint16_t)(full_address >> 1), data, length); // Divide by 2 for word addressing
						break;
    	            }

    	        case HEX_RECORD_TYPE_EOF:
    	            // End of file record
    	            break;

    	        default:
    	            // Skip other record types
    	            for (uint8_t i = 0; i < length; i++) {
    	                calculated_checksum += hex_to_byte((const uint8_t *)hex_line);
    	                hex_line += 2;
    	            }
    	            break;
    	    }
    	    return 0;
    	}

USBH_StatusTypeDef program_hex_file(const char *hex_data, uint32_t size)
{
	USBH_StatusTypeDef status;
	char line_buffer[256];
	uint32_t base_address = 0;
	int line_length = 0;
	int i = 0;
	int result;

	while (i < size) {

		if(hex_data[i] == ':'){
			//start of line
			i=i+1;
		    line_length = 0;
		    int j=0;

		    while (hex_data[j]!='\r' && hex_data[j]!='\n'){
			  line_buffer[j] = hex_data[j];
			  line_length++;
			  j++;
			  i++;
		    }
		    line_buffer[j] = '\0';
		    line_length++;
		// Parse and program the line
		result = parse_and_program_hex_line(line_buffer, &base_address);

		while (result != 0) {
			result = parse_and_program_hex_line(line_buffer, &base_address);
		}

		line_length = 0;
		i++;


		if (i >= size) {
			break; // End of file
		}

	}
	}

	// Exit programming mode
	status = send_stk_command(STK_LEAVE_PROGMODE, NULL, 0);

	while(status != USBH_OK){
     status = send_stk_command(STK_LEAVE_PROGMODE, NULL, 0);
	}

	arduino_prog_state = ARDUINO_PROG_PROGRAMMED;

	return USBH_OK;
}
ArduinoProgrammingState get_programming_state(void)
{
	return arduino_prog_state;
}

void reset_programming_state(void)
{
	arduino_prog_state = ARDUINO_PROG_IDLE;
}

USBH_StatusTypeDef verify_hex_file(const char *hex_data, uint32_t size)
{
	USBH_StatusTypeDef status;
	char line_buffer[256];
	uint32_t base_address = 0;
	int line_length = 0;
	int i = 0;
	uint8_t length, type;
	uint16_t address;
	uint8_t data[PROG_PAGE_SIZE];
	uint8_t read_data[PROG_PAGE_SIZE];
	uint8_t params[4];

	//printf("Starting verification of hex file...\r\n");


	while (i < size) {
		// Find start of line (colon)
		while (i < size && hex_data[i] != ':') {
			i++;
		}

		if (i >= size) {
			break; // End of file
		}

		// Read line into buffer
		line_length = 0;
		while (i < size && hex_data[i] != '\n' && hex_data[i] != '\r' && line_length < 255) {
			line_buffer[line_length++] = hex_data[i++];
		}
		line_buffer[line_length] = '\0';

		// Skip line if it doesn't start with ':'
		if (line_buffer[0] != ':') {
			continue;
		}

		// Parse line
		const char *line_ptr = line_buffer + 1; // Skip ':'

		// Parse length
		length = hex_to_byte((const uint8_t *)line_ptr);
		line_ptr += 2;

		// Parse address
		address = (uint16_t)hex_to_byte((const uint8_t *)line_ptr) << 8;
		line_ptr += 2;
		address |= hex_to_byte((const uint8_t *)line_ptr);
		line_ptr += 2;

		// Parse record type
		type = hex_to_byte((const uint8_t *)line_ptr);
		line_ptr += 2;

		if (type == HEX_RECORD_TYPE_DATA) {
			// Parse data
			for (uint8_t j = 0; j < length; j++) {
				data[j] = hex_to_byte((const uint8_t *)line_ptr);
				line_ptr += 2;
			}

			// Calculate full address
			uint32_t full_address = base_address | address;
			uint16_t word_address = (uint16_t)(full_address >> 1); // Divide by 2 for word addressing

			// Load address
			params[0] = (uint8_t)(word_address & 0xFF);    // Low byte
			params[1] = (uint8_t)((word_address >> 8) & 0xFF); // High byte

			status = send_stk_command(STK_LOAD_ADDRESS, params, 2);
			if (status != USBH_OK) {
				printf("Failed to load address during verification\r\n");
				arduino_prog_state = ARDUINO_PROG_ERROR;
				return status;
			}

			// Read page
			uint8_t read_cmd[3];
			read_cmd[0] = (uint8_t)(length >> 8); // Length high byte
			read_cmd[1] = (uint8_t)(length & 0xFF); // Length low byte
			read_cmd[2] = 'F'; // Memory type: 'F' for Flash

			status = send_stk_command(STK_READ_PAGE, read_cmd, 3);
			printf("Status: 0x%02X\r\n", status);


			// Receive data
			 status = USBH_FAIL;
			 while(status != USBH_OK){
				status =USBH_CDC_Receive(&hUsbHostFS, read_data, length + 2);// Typically INSYNC + OK
			 }
			 // Check for valid response (INSYNC + OK)
			 if (stk_response_buffer[0] != STK_INSYNC) {
				//printf("Invalid INSYNC byte received: 0x%02X\r\n", stk_response_buffer[0]);
				return USBH_FAIL;
			 }

			 if (stk_response_buffer[1] != STK_OK) {
			   // printf("Invalid OK byte received: 0x%02X\r\n", stk_response_buffer[1]);
				return USBH_FAIL;
			 }

			 else if(stk_response_buffer[0] == STK_INSYNC && stk_response_buffer[1] == STK_OK){
				 arduino_prog_state = ARDUINO_PROG_BOOTLOADER_READY;
				 return USBH_OK;

			// Verify data (skip INSYNC byte)
			for (uint8_t j = 0; j < length; j++) {
				if (data[j] != read_data[j + 1]) { // +1 to skip INSYNC byte
					printf("Verification failed at address 0x%04X: expected 0x%02X, got 0x%02X\r\n",
						   full_address + j, data[j], read_data[j + 1]);
					arduino_prog_state = ARDUINO_PROG_ERROR;
					return USBH_FAIL;
				}
			}
		} else if (type == HEX_RECORD_TYPE_EOF) {
			// End of file
			break;
		} else if (type == HEX_RECORD_TYPE_EXT_LINEAR_ADDR) {
			// Update base address
			base_address = (uint32_t)hex_to_byte((const uint8_t *)line_ptr) << 24;
			line_ptr += 2;
			base_address |= (uint32_t)hex_to_byte((const uint8_t *)line_ptr) << 16;
		}

		// Skip remaining EOL characters
		while (i < size && (hex_data[i] == '\n' || hex_data[i] == '\r')) {
			i++;
		}
	}

	//printf("Verification completed successfully\r\n");
	arduino_prog_state = ARDUINO_PROG_VERIFIED;

	return USBH_OK;
}
}
const char* get_programming_status_message(void)
{
	switch (arduino_prog_state) {
		case ARDUINO_PROG_IDLE:
			return "Ready to program";
		case ARDUINO_PROG_RESET_SENT:
			return "Reset sent, waiting for bootloader";
		case ARDUINO_PROG_BOOTLOADER_READY:
			return "Bootloader ready";
		case ARDUINO_WAIT_RESPONSE:
			return "Waiting for stk response";
		case ARDUINO_PROG_PROGRAMMED:
			return "Programming in progress";
		case ARDUINO_PROG_VERIFIED:
			return "Verifying flash";
		case ARDUINO_PROG_COMPLETED:
			return "Programming completed successfully";
		case ARDUINO_PROG_ERROR:
			return "Error during programming";
		default:
			return "Unknown state";
	}
	return NULL;
}
