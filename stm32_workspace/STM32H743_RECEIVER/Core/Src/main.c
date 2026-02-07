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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_ring.h"  // Ring buffer API: UART_Ring_Init(), UART_Ring_GetByte()
#include "crsf.h"

#include <stdio.h>  // snprintf

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRINT_LINK_STATUS  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t led_enabled = 0;

static CRSF_Parser_t gCrsf;      // Parser-Instanz

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t app_millis(void) { return HAL_GetTick(); }

// formatiert n Kanäle (µs) aus dem Parser in eine Textzeile "xxxx xxxx ...\r\n"
static int format_channels_us(char *msg, int n, const CRSF_Parser_t* p)      /*-- Änderung --*/
{
    int pos = 0;
    for (int i = 0; i < n; i++) {
        int32_t v = CRSF_TicksToUs(p->rc.ch_ticks[i]);
        // Tausenderstelle
        if (v >= 1000) msg[pos++] = '0' + (v / 1000);
        else           msg[pos++] = ' ';
        // Hunderterstelle
        if (v >= 100)  msg[pos++] = '0' + ((v / 100) % 10);
        else           msg[pos++] = ' ';
        // Zehnerstelle
        if (v >= 10)   msg[pos++] = '0' + ((v / 10) % 10);
        else           msg[pos++] = ' ';
        // Einerstelle
        msg[pos++] = '0' + (v % 10);

        // Trennzeichen (außer nach letztem Kanal)
        if (i < n - 1) msg[pos++] = ' ';
    }
    msg[pos++] = '\r';
    msg[pos++] = '\n';
    msg[pos] = '\0';
    return pos;
}

// ---- H743: Dummy-GPS über CRSF 0x02 zum ELRS-Receiver senden ----
static void H743_SendGpsDummy(UART_HandleTypeDef *huart)
{
	static int32_t lat_val = 487000000; // // 48.7000000° (Startwert)
    CRSF_GPS_t payload;

    // Latitude anpassen (pro Aufruf +1)
    payload.latitude = lat_val++;

    // Restliche Felder statisch lassen
    payload.longitude   =  91000000;  // 9.1000000°
    payload.groundspeed = 3650;       // 36.50 km/h
    payload.heading     = 12345;      // 123.45°
    payload.altitude    = (uint16_t)(450 + 1000); // 450 m MSL -> +1000 Offset => 1450
    payload.satellites  = 17;

    uint8_t frame[32];
    uint16_t flen = CRSF_CreateFrame(frame, 0x02, (const uint8_t*)&payload, sizeof(payload));

    // UART: Full-Duplex, non-inverted, 420000 baud
    HAL_UART_Transmit(huart, frame, flen, 5);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t b;
	uint32_t last_print_ms = 0;

	uint8_t  ls_seen_flag = 0;
	uint32_t last_ls_print = 0;
	uint32_t last_gps_tx = 0;

	char msg[128];

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Ring_Init(&huart3);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);  // RX interrupt aktivieren

  /* Wait until key K1 is pressed (PC13, high active, pull-down) */
  while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
	  HAL_Delay(1); // Short delay
  }

  led_enabled = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // Eingehende Bytes vom Ringbuffer in den Parser füttern
	  while (UART_Ring_GetByte(&b)) {
		  uint8_t ftype;
		  if (CRSF_ParserFeed(&gCrsf, b, &ftype)) {
			  // Link Statistics message
			  if (ftype == CRSF_FRAMETYPE_LINK_STATISTICS) {
				  ls_seen_flag = 1; // Nur Flag setzen, KEIN sofortiger Print
				  //const char msg[] = "Link Statistics packet received\r\n";
				  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 20);
			  }

			  // optional: auf RC/LinkStats reagieren
			  // if (ftype == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) { ... }
			  // if (ftype == CRSF_FRAMETYPE_LINK_STATISTICS)   { ... }
		  }
	  }

	  // Alle 100 ms: 8 RC-Kanäle in µs ausgeben
	  uint32_t now = HAL_GetTick();
	  if (now - last_print_ms >= 100) {
		  last_print_ms = now;
		  int k = format_channels_us(msg, 8, &gCrsf);
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)k, 50);
	  }

	  if (now - last_ls_print >= 1000) {
		  last_ls_print = now;

		  if (ls_seen_flag) {
			  ls_seen_flag = 0; // Flag zuruecksetzen, damit wir nur einmal pro Sekunde drucken

			  // Wir lesen die LQ direkt aus dem Parserzustand:
			  // up_LQ = Uplink Link Quality in %, dn_LQ = Downlink Link Quality in %
			  uint8_t up   = gCrsf.linkStats.up_LQ;
			  uint8_t dn   = gCrsf.linkStats.dn_LQ;
			  uint8_t pidx = gCrsf.linkStats.up_tx_power;

			  // Puffer vorbereiten und snprintf verwenden
			  char line[80];
			  int n = snprintf(line, sizeof(line),
					  "Link Quality: up=%u%%, down=%u%%, up_tx_power(idx)=%u\r\n", up, dn, pidx);

			  if (n > 0) {
				  // Achtung: n kann >= sizeof(line) sein; wir senden max. Puffergroesse-1
				  uint16_t txlen = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line) - 1);
				  HAL_UART_Transmit(&huart2, (uint8_t*)line, txlen, 20);
			  }

			  //const char txt1[] = "Link Statistics packet received\r\n";
			  //HAL_UART_Transmit(&huart2, (uint8_t*)txt1, sizeof(txt1) - 1, 20);
		  } else {
			  const char txt2[] = "No Link Statistics in last second\r\n";
			  HAL_UART_Transmit(&huart2, (uint8_t*)txt2, sizeof(txt2) - 1, 20);
		  }
	  }

	  if (now - last_gps_tx >= 200U) { // 5 Hz
	      last_gps_tx = now;
	      H743_SendGpsDummy(&huart3);   // huart3 = ELRS-Receiver-UART
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
