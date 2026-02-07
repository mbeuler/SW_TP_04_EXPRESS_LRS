/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_ring.h"  // Ring buffer API: UART_Ring_Init(), UART_Ring_GetByte()
#include "crsf.h"
#include "usbh_hid.h"

#include <stdio.h>  // snprintf

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Sendezyklus für RC-Frames: 4 ms
#define CRSF_RC_PERIOD_MS   (4U)
// 1-Hz Debugausgabe
#define STATUS_PERIOD_MS    (1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Demo: RC-Kanäle in µs (CH1 wippt zwischen 1000 und 1500)
uint16_t rc_us[16] = {
    1000,1100,1200,1300,1400,1500,1600,1700,
    1800,1900,2000,1111,1122,1133,1144,1155
};

static CRSF_Parser_t gCrsf;      // Parser-Instanz

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t app_millis(void) { return HAL_GetTick(); }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t b;

	uint8_t  ls_seen_flag = 0;

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  // Ringbuffer für USART3 initialisieren & RX-Interrupt aktivieren
  UART_Ring_Init(&huart3);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);  // RX interrupt aktivieren

  // CRSF-Parser initialisieren (Zeit-Callback für LinkIsAlive etc.)
  CRSF_ParserInit(&gCrsf, app_millis);

  // Start im RX-Betrieb (Halbduplex)
  HAL_HalfDuplex_EnableReceiver(&huart3);

  // Demo-Variablen
    int      direction      = +10;  // +10 = hochzählen, -10 = runterzählen
    uint32_t lastUpdate     = 0;    // für 1-Sekunden-Intervall
    uint32_t lastFrame      = 0;    // für 250 Hz
    uint32_t last_ls_print  = 0;
    uint32_t last_gps_print = 0;

    // Arbeitsbuffer für TX
    uint8_t payload[22];
    uint8_t frame[32];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick();

	  /* --- 1) Demo: CH1 jede Sekunde ändern --------------------------------- */
	  if (now - lastUpdate >= 1000) {
		  lastUpdate = now;
		  rc_us[0] += direction;
	      if (rc_us[0] >= 1500) {
	    	  rc_us[0] = 1500;
	          direction = -10;
	      }
	      else if (rc_us[0] <= 1000) {
	    	  rc_us[0] = 1000;
	          direction = +10;
	      }
	  }

	  /* --- 2) RC-Frame alle 4 ms senden (250 Hz) ---------------------------- */
	  if (now - lastFrame >= CRSF_RC_PERIOD_MS) {
	      lastFrame = now;

	      // a) µs -> CRSF-Ticks & 22-Byte-Payload packen
	      uint16_t ticks[16];
	      for (int i = 0; i < 16; i++) ticks[i] = CRSF_UsToTicks(rc_us[i]);
	      CRSF_PackChannels16x11(ticks, payload);

	      // b) Kompletten Frame bauen
	      uint16_t flen = CRSF_CreateFrame(frame, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, payload, 22);

	      // c) Senden im Half-Duplex
	      HAL_HalfDuplex_EnableTransmitter(&huart3);
	      HAL_UART_Transmit(&huart3, frame, flen, 2);

	      // WICHTIG: Auf TC warten (inkl. Stop-Bit), erst dann auf RX schalten
	      while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) { /* warten */ }
	      HAL_HalfDuplex_EnableReceiver(&huart3);
	      // Ab hier sammelt die ISR (UART_Ring_IRQ_Handler) eingehende Bytes
	  }

	  /* --- 3) Eingehende Bytes -> Parser (Ringbuffer leeren) ---------------- */
	  while (UART_Ring_GetByte(&b)) {
		  uint8_t ftype;
		  if (CRSF_ParserFeed(&gCrsf, b, &ftype)) {
			  if (ftype == CRSF_FRAMETYPE_LINK_STATISTICS) {
				  ls_seen_flag = 1;   // Signal, dass neue Link-Stats angekommen sind
	          }
	      }
	  }

	  if (now - last_ls_print >= 1000U) {
		  last_ls_print = now;

		  if (ls_seen_flag) {
			  ls_seen_flag = 0;  // zurücksetzen, damit wir nur einmal pro Sekunde drucken

			  // Wir lesen die LQ direkt aus dem Parserzustand:
			  uint8_t up   = gCrsf.linkStats.up_LQ;
			  uint8_t dn   = gCrsf.linkStats.dn_LQ;
			  uint8_t pidx = gCrsf.linkStats.up_tx_power;

			  char line[80];
			  int n = snprintf(line, sizeof(line),
					  "Link Quality: up=%u%%, down=%u%%, up_tx_power(idx)=%u\r\n", up, dn, pidx);

			  if (n > 0) {
				  uint16_t len = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line)-1);
				  HAL_UART_Transmit(&huart2, (uint8_t*)line, len, 20);
			  }
		  } else {
			  const char no[] = "No Link Statistics in last second\r\n";
			  HAL_UART_Transmit(&huart2, (uint8_t*)no, sizeof(no) - 1, 20);
		  }
	  }

	  if (now - last_gps_print >= 1000U) {
		  last_gps_print = now;

		  // Einfache Plausibilitätsprüfung:
		  if (gCrsf.gps.latitude != 0 || gCrsf.gps.longitude != 0) {
			  char line[128];
			  //int alt_m = (int)gCrsf.gps.altitude - 1000; // Offset -1000m zurückrechnen
			  int n = snprintf(line, sizeof(line),
			      "GPS RAW: lat=%ld lon=%ld gs=%u hdg=%u alt=%u sats=%u\r\n",
			      (long)gCrsf.gps.latitude,
			      (long)gCrsf.gps.longitude,
			      (unsigned)gCrsf.gps.groundspeed,
			      (unsigned)gCrsf.gps.heading,
			      (unsigned)gCrsf.gps.altitude,
			      (unsigned)gCrsf.gps.satellites
			  );

			  if (n > 0) {
				  uint16_t txlen = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line)-1);
				  HAL_UART_Transmit(&huart2, (uint8_t*)line, txlen, 20);
			  }
		  }
	  }

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
