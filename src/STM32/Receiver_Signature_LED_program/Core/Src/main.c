
#include "main.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include <string.h>
#include "stdio.h"


#define BUFFER_SIZE 1024  // Define a buffer size
#define MAX_BOARD_NAME_LENGTH 50
char detected_board_name[MAX_BOARD_NAME_LENGTH] = "No Device";  // Store the detected board name
#define STK_GET_SYNC       0x30

uint8_t receivedData; // To store the received data
uint8_t receivedBuffer[BUFFER_SIZE];  // Buffer to hold received data
uint16_t bufferIndex = 0;  // Index to track the position in the buffer

uint8_t selected_board_name[MAX_BOARD_NAME_LENGTH]; //board name

#define HEX_LENGTH 17500
uint8_t hex_file[HEX_LENGTH]; //board name
uint16_t hexIndex = 0;
#define RX_BUFF_SIZE 64
uint8_t CDC_RX_Buffer[RX_BUFF_SIZE];
uint8_t CDC_TX_Buffer[RX_BUFF_SIZE];

int count =0; // flag showing cleared milestone 1 (END1)
int hex_received =0;
int signature_match=0;

volatile uint8_t upload_requested = 0; // Flag to track upload button state

void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

void process_upload_request(void);
void update_status_led(void);


void TurnOnLED(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Turn LED ON
}

void TurnOffLED(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Turn LED OFF
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();

  HAL_UART_Init(&huart2);
  HAL_UART_Receive_IT(&huart1, &receivedData, 1);
  while (1)
   {
       MX_USB_HOST_Process();  // Process USB tasks

       const char* detected_device = check_usb_device();

       strncpy(detected_board_name, detected_device, MAX_BOARD_NAME_LENGTH - 1);
       detected_board_name[MAX_BOARD_NAME_LENGTH - 1] = '\0'; // Ensure null termination

       if (strcmp(detected_board_name, "No Device") == 0) {
           TurnOffLED();  // No device detected, turn off LED
       }
       else {
           if (strcmp(detected_board_name, (char*)selected_board_name) == 0) {
//              printf("Detected board matches the selected board: %s\r\n", detected_board_name);
               TurnOnLED();  // ✅ Turn on LED for a matching board
               signature_match = 1;
           } else {
              // printf("Board mismatch! Detected: %s, Selected: %s\r\n", detected_board_name, (char*)selected_board_name);
               TurnOffLED();  // ❌ Turn off LED for mismatch
               signature_match=0;
           }
       }
       int upload_requested = 0;
       if(signature_match==1 && hex_received ==1 && upload_requested == 0){
    	   HAL_UART_Transmit(&huart2, (uint8_t *)"upload_requested\r\n", 200, HAL_MAX_DELAY);
    	   process_upload_request();
    	   upload_requested = 1;
       }
       HAL_Delay(100);
   }
}

void process_upload_request(void) //FSM implementation
{

  ArduinoProgrammingState prog_state = get_programming_state();

  while(prog_state != ARDUINO_PROG_COMPLETED)
  {
	  if(prog_state == ARDUINO_PROG_IDLE){
		  printf("IDLE STATE\r\n");
		  trigger_arduino_bootloader();
		  prog_state = get_programming_state();
	  }

	  else if (prog_state == ARDUINO_PROG_RESET_SENT) {
		  printf("RESET SENT\r\n");
          if(send_stk_command(STK_GET_SYNC, NULL, 0) == USBH_OK){
          } else{
        	  printf("command not sent");
          }
		  prog_state = get_programming_state();
	  }
	  else if(prog_state == ARDUINO_WAIT_RESPONSE){
		  memset(CDC_RX_Buffer, 0, RX_BUFF_SIZE);
		  receive_stk_response(CDC_RX_Buffer);
		  prog_state = get_programming_state();

	  }
	  else if (prog_state == ARDUINO_PROG_BOOTLOADER_READY) {
		  printf("Boothloader in SYNC\r\n");
          program_hex_file((const char*)hex_file, strlen((char*)hex_file));
		  prog_state = get_programming_state();
	  }

	  else if (prog_state == ARDUINO_PROG_PROGRAMMED){
		  printf("Programmed\r\n");
          verify_hex_file((const char*)hex_file, strlen((char*)hex_file));
          prog_state = get_programming_state();
	  }

	  else if (prog_state == ARDUINO_PROG_VERIFIED){
		  printf("Verified\r\n");
          reset_programming_state();
          prog_state = get_programming_state();
	  }

	  else if (prog_state == ARDUINO_PROG_ERROR){
		  printf("Error Programming\r\n");
          reset_programming_state();
          prog_state = get_programming_state();
	  }

	  else {
		  printf("Unknown state\r\n");
		  reset_programming_state();
		  prog_state = get_programming_state();
	  }

  }

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // Check if the interrupt is from USART1
    {
        receivedBuffer[bufferIndex++] = receivedData;

        if (bufferIndex == BUFFER_SIZE)
        {

            if(count == 1)
            {
            	for (int i=0; i<=(bufferIndex);i++)
            	{
            		hex_file[hexIndex++] = receivedBuffer[i];
            	}
            }
            else
            	hex_received = 0;

            bufferIndex = 0;
        }

        if (receivedBuffer[bufferIndex - 4] == 'E' && receivedBuffer[bufferIndex - 3] == 'N' && receivedBuffer[bufferIndex - 2] == 'D' && receivedBuffer[bufferIndex - 1] == '1') {
        	for (int i=0; i<=(bufferIndex-5);i++)
        	{
        	  selected_board_name[i] = receivedBuffer[i];
        	}
        	for(int i=0; i<=(bufferIndex); i++)
        	    {
        	     receivedBuffer[bufferIndex] = "";
        	     }
        	     bufferIndex=0;
        	     count =1;
        }

        //Detecting end
        if (receivedBuffer[bufferIndex - 4] == 'E' && receivedBuffer[bufferIndex - 3] == 'N' && receivedBuffer[bufferIndex - 2] == 'D' && receivedBuffer[bufferIndex - 1] == '2') {
        	for (int i=0; i<=(bufferIndex-5);i++)
        	{
        	  hex_file[hexIndex++] = receivedBuffer[i];
        	}
        	hex_received = 1;
        	for(int i=0;i<=HEX_LENGTH;i++)
        	        	{
        	        		hex_file[i] = "";
        	        	}
        	 for(int i=0; i<=(bufferIndex); i++)
        	  {
        	   	receivedBuffer[bufferIndex]="";
        	   }
        	   bufferIndex=0;
        	   count=0;

        }
        HAL_UART_Receive_IT(&huart1, &receivedData, 1);
     }
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
