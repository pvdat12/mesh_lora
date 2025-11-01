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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
//#include "LoRa.h"
#include "lora_sx1276.h"
#include "mesh_AODV.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t count = 0;
lora_sx1276 myLoRa;
uint8_t read_data[128];
uint8_t send_data[128];
uint16_t check_lora;
uint8_t len;
int			RSSI;
int ret;
char buff[50];
uint8_t tx_buf[32];
uint8_t rx_buf[32];
uint8_t err;
uint32_t seq = 0;


#define PI_NODE 0		//this  determines whether or not this node is "special" or "stock"
#define DATA_REQ 55
void uart_print(const char *s) {
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	myLoRa = newLoRa();
//
//	myLoRa.hSPIx                 = &hspi1;
//	myLoRa.CS_port               = NSS_GPIO_Port;
//	myLoRa.CS_pin                = NSS_Pin;
//	myLoRa.reset_port            = RESET_GPIO_Port;
//	myLoRa.reset_pin             = RESET_Pin;
//	myLoRa.DIO0_port						 = DIO0_GPIO_Port;
//	myLoRa.DIO0_pin							 = DIO0_Pin;
//
//	myLoRa.frequency             = 867;							  // default = 433 MHz
//	myLoRa.spredingFactor        = SF_7;							// default = SF_7
//	myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
//	myLoRa.crcRate				       = CR_4_5;						// default = CR_4_5
//	myLoRa.power					       = POWER_20db;				// default = 20db
//	myLoRa.overCurrentProtection = 100; 							// default = 100 mA
//	myLoRa.preamble				       = 5;
//
//	LoRa_reset(&myLoRa);
//	check_lora = LoRa_init(&myLoRa);

	myLoRa.reset_port = RESET_GPIO_Port;
	myLoRa.reset_pin = RESET_Pin;
  lora_reset(&myLoRa);
  check_lora = lora_init(&myLoRa, &hspi1, NSS_GPIO_Port, NSS_Pin, LORA_BASE_FREQUENCY_VN);
  DEBUG_PRINT("Init success\n");
  lora_set_spreading_factor(&myLoRa, 7);                 // SF7
  lora_set_signal_bandwidth(&myLoRa, LORA_BANDWIDTH_125_KHZ); // BW 125kHz
  lora_set_coding_rate(&myLoRa, LORA_CODING_RATE_4_5);   // CR 4/5
  lora_set_tx_power(&myLoRa, 17);                        // 17 dBm
  lora_set_preamble_length(&myLoRa, 8);                  // Preamble = 8
  lora_set_crc(&myLoRa, 1);                              // Enable CRC
  lora_set_explicit_header_mode(&myLoRa);                // Explicit header




	// START CONTINUOUS RECEIVING -----------------------------------
//	LoRa_startReceiving(&myLoRa);
//   Chuẩn bị payload với số thứ tự

		  uint32_t unique_seed = get_UID();	//initialize with a random ID
		  DEBUG_PRINT("device id: %d\n\r", unique_seed);
		  srand(unique_seed);
		  uint32_t new_id;
		  new_id = rand();
		  while(new_id < 11){
			  new_id = rand();
		  }
		  DEBUG_PRINT("New ID: %d\n\r", new_id);
		  polkadot_init(new_id);
		  rand_delay();
		  uint32_t current_time = HAL_GetTick();
		  if (new_id == 1437203090)
		  {
			  mesh_send_hello();
		  }
//		  mesh_send_hello();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	printf("Sending package...\r\n");
//	send_data[0] = 'H'; // MY ADDRESS
//	send_data[1] = 'E'; // MY ADDRESS
//	send_data[2] = 'L'; // MY ADDRESS
//	send_data[3] = 'L'; // MY ADDRESS
//	send_data[4] = 'O'; // MY ADDRESS
////	  	  send_data[0] = 0x3B; // MY ADDRESS
////	  		for(int i=0; i<26; i++)
////	  			send_data[i+1] = 48+i;
//	ret = 	LoRa_transmit(&myLoRa, send_data, 5, 500);
//	printf("Send status: %d\r\n", ret);
//	HAL_Delay(1500);
//	uint8_t rssi= LoRa_getRSSI(&myLoRa);
//	printf("RSSI: %d\r\n", rssi);
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	printf("Done\r\n");
//	HAL_Delay(5000);
//	send_data[0] = 'W'; // MY ADDRESS
//	send_data[1] = 'O'; // MY ADDRESS
//	send_data[2] = 'R'; // MY ADDRESS
//	send_data[3] = 'L'; // MY ADDRESS
//	send_data[4] = 'D'; // MY ADDRESS
//	ret = 	LoRa_transmit(&myLoRa, send_data, 5, 500);
//	HAL_Delay(1500);
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  uint8_t current_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

//	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
//	  {
//		  HAL_Delay(20);
//		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
//		  {
//			  mesh_send_rreq(0, new_id, my_sequence_number, 0, rreq_id);
//			  HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_13);
//			  count++;
//			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0);
//		  }
//
//	  }
//	  if (HAL_GetTick() - current_time > 12 * 1000)
//	  {
//		  mesh_send_hello();
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_13);
//		  current_time = HAL_GetTick();
//	  }

	  lora_mode_receive_continuous(&myLoRa);
	  len = lora_receive_packet_blocking(&myLoRa, rx_buf, sizeof(rx_buf), 2000, &err);
	  if (len > 0 && err == LORA_OK)
	  {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_13);
		  receive_packet_handler(rx_buf, sizeof(rx_buf));
	  }
//	  lora_mode_receive_continuous(&myLoRa);
//	  len = lora_receive_packet_blocking(&myLoRa, rx_buf, sizeof(rx_buf), 2000, &err);
//	  if (len > 0 && err == LORA_OK)
//	  {
////		  char buf[256];
////		  int pos = 0;
////		  pos += sprintf(buf + pos, "RX(%d bytes: )", len);
////		  for (int i = 0; i < len; i++)
////		  {
////			  pos += sprintf(buf + pos, "%02X  ", rx_buf[i]);
////		  }
////		  sprintf(buf + pos, "| RSSI=%d dBm | SNR=%d dB\r\n", lora_packet_rssi(&myLoRa), lora_packet_snr(&myLoRa));
////		  uart_print(buf);
//		  receive_packet_handler(rx_buf, sizeof(rx_buf));
//	  }


      // Chuyển sang chế độ nhận để chờ ACK
//      lora_mode_receive_continuous(&myLoRa);
      // 3️⃣ Chuyển sang chế độ nhận để chờ ACK
//      lora_mode_receive_single(&myLoRa); // Nghe 1 gói thôi, tiết kiệm năng lượng
//      uint32_t start_time = HAL_GetTick();
//      len = 0;
//
//      // 4️⃣ Chờ đến khi nhận được phản hồi hoặc timeout (5 giây)
//      do {
//          len = lora_receive_packet_blocking(&myLoRa, rx_buf, sizeof(rx_buf), 2000, &err);
//          if (len > 0 && err == LORA_OK) {
//              rx_buf[len] = '\0';
//              char buf[128];
//              sprintf(buf, "RX_ACK: %s | RSSI=%d dBm | SNR=%d dB\r\n", rx_buf, lora_packet_rssi(&myLoRa), lora_packet_snr(&myLoRa));
//              uart_print(buf);
//              HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//              // Kiểm tra ACK có khớp số thứ tự không
//              if (strncmp((char*)rx_buf, "ACK_", 4) == 0 && rx_buf[4] == ('0' + seq)) {
//                  char buf[64];
//            	  sprintf(buf, "ACK verified for seq=%lu\r\n", seq);
//                  seq++;
//              } else {
//                  uart_print("Invalid ACK!\r\n");
//              }
//              break;
//          }
//      } while (HAL_GetTick() - start_time < 5000);
//
//      //if (len == 0 || err != LORA_OK) {
//      if (1) {
//    	  char buf[64];
//          sprintf(buf, "No ACK received for seq=%lu (err=%d)\r\n", seq, err);
//          uart_print(buf);
//      }

      //seq++;
//      HAL_Delay(1000); // 1 giây giữa các gói
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
#endif /* USE_FULL_ASSERT */
