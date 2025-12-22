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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRSF_MAX_FRAME_LEN 64

#define RC_TO_US(x)   (1500 + (((x) - 992) * 5 + 4) / 8)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern UART_HandleTypeDef huart3;

// Einzelbyte-Puffer fuer Interrupt-Empfang (USART3)
static uint8_t uart3_rx_byte = 0;

// Rohwerte aller 16 Kanäle
volatile uint16_t rc_raw[16] = {0};

// Umgerechnete us-Werte aller 16 Kanäle (Servo-kompatibel, 1000...2000 us)
volatile uint16_t rc_us[16] = {
		1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
		1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

static uint8_t  crsf_frame[CRSF_MAX_FRAME_LEN]; // CRSF frame buffer
static size_t   crsf_idx = 0;
static uint8_t  crsf_expected_len = 0;

typedef enum { S_WAIT_DEST, S_WAIT_LEN, S_READ_BODY } crsf_state_t;
static crsf_state_t crsf_state = S_WAIT_DEST;

// Lookup Table:  https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#crc
//                https://www.sunshine2k.de/coding/javascript/crc/crc_js.html  -->  CRC8_DVB_S2
static const uint8_t CRC8_DVB_S2_TABLE[256] = {
	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint8_t crc8_dvb_s2_bitwise(const uint8_t *data, size_t len)
{
    uint8_t crc  = 0x00;        // Initial value
    const uint8_t POLY = 0xD5;  // CRC-8/DVB-S2 polynomial

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)(((crc << 1) ^ POLY) & 0xFF);
            } else {
                crc = (uint8_t)((crc << 1) & 0xFF);
            }
        }
    }
    return crc; // No XOR output, no reflection
}


static inline uint8_t crc8_dvb_s2_table(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00; // Initial value
    for (size_t i = 0; i < len; i++) {
        crc = CRC8_DVB_S2_TABLE[(uint8_t)(crc ^ data[i])];
    }
    return crc;
}


static inline int crsf_verify_frame(uint8_t *frame, size_t frame_len)
{
	uint8_t len;
	uint8_t crc_calc, crc_frame;
	size_t crc_in_len;

    if (frame_len < 4) return 0;
    len = frame[1];
    if (frame_len != (size_t)(len + 2)) return 0;  // Consistency
    uint8_t *type_ptr = &frame[2];
    crc_in_len = (size_t)(len - 1);                // type + payload
    //crc_calc  = crc8_dvb_s2_bitwise(type_ptr, crc_in_len);
    crc_calc  = crc8_dvb_s2_table(type_ptr, crc_in_len);
    crc_frame = frame[frame_len - 1];

    return (crc_calc == crc_frame);
}


static int is_valid_dest(uint8_t d)
{
    /* FC=0xC8, TX-Modul=0xEE, Radio-Handset=0xEA, RX=0xEC */
    return (d == 0xC8 || d == 0xEE || d == 0xEA || d == 0xEC);
}


static void crsf_unpack_rc16(const uint8_t *payload, size_t payload_len, uint16_t ch[16])
{
	// Erwartete Länge für 16*11 Bit: 176 Bit = 22 Bytes
	if (payload_len < 22) return;  // Konservativ prüfen

	// Für jede Kanalnummer i: 11-Bit-Wort ab Bitposition (11*i)
	for (int i = 0; i < 16; i++) {
		// 11-bit Startposition des Kanals i im Bitstrom
		const uint32_t bitpos    = 11U * i;
		const uint32_t byteIndex = bitpos >> 3;   // Byte-Start (bitpos/8)
		const uint32_t bitInByte = bitpos & 0x7U; // Offset im Startbyte (0...7)

		// 24-Bit Fenster: Hole bis zu 3 Bytes und baue sie little-endian zusammen
		uint32_t raw32 = 0;
		// byteIndex ist maximal 21; wir lesen defensiv bis zu 3 Bytes, prüfen die Grenzen
		for (uint32_t k = 0; k < 3; k++) {
			size_t idx = (size_t)byteIndex + (size_t)k;
			if (idx < payload_len) {
				raw32 |= ((uint32_t)payload[idx]) << (8U * (uint32_t)k);
			}
		}

		raw32 >>= bitInByte;                   // Zur Bitposition ausrichten
		ch[i]   = (uint16_t)(raw32 & 0x07FFU); // 11 Bit extrahieren
	}
}


static inline uint16_t map_rc_to_us(uint16_t raw)
{
	// Begrenzung auf CRSF-typischen Bereich
	if (raw < 172)  raw =  172;
	if (raw > 1811) raw = 1811;

	return RC_TO_US(raw);
}


static void crsf_process_valid_frame(const uint8_t *frame, size_t frame_len)
{
    /* Frame: [dest][len][type][payload][crc] */
	if (frame_len < 5) return;

	const uint8_t len  = frame[1];
	const uint8_t type = frame[2];

	const uint8_t *payload     = &frame[3];         // payload beginnt nach type
	const size_t   payload_len = (size_t)(len - 2); // len = type + payload + crc

	if (type == 0x16) {  // RC channels packet
		if (payload_len < 22) return;  // Konservativ prüfen

		uint16_t ch_temp[16] = {0};
		crsf_unpack_rc16(payload, payload_len, ch_temp);

		// Rohwerte und us für ALLE 16 Kanäle ablegen
		for (int i = 0; i < 16; i++) {
			rc_raw[i] = ch_temp[i];
			rc_us[i]  = map_rc_to_us(ch_temp[i]);
		}
	}

	// Andere types ignorieren (bei Bedarf später erweitern)
}


static void crsf_consume_byte(uint8_t b)
{
    switch (crsf_state) {
    case S_WAIT_DEST:
        if (is_valid_dest(b)) {
            crsf_frame[0] = b;
            crsf_idx = 1;
            crsf_state = S_WAIT_LEN;
        }
        break;

    case S_WAIT_LEN:
        crsf_expected_len = b;
        crsf_frame[crsf_idx++] = b;
        if (crsf_expected_len < 2) { // mindestens type(1)+crc(1)
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
            break;
        }
        if ((2 + crsf_expected_len) > CRSF_MAX_FRAME_LEN) {
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
            break;
        }
        crsf_state = S_READ_BODY;
        break;

    case S_READ_BODY:
        crsf_frame[crsf_idx++] = b;
        if (crsf_idx == (size_t)(2 + crsf_expected_len)) {
            /* Vollständiges Frame vorhanden */
            if (crsf_verify_frame(crsf_frame, crsf_idx)) {
                crsf_process_valid_frame(crsf_frame, crsf_idx);
            }
            crsf_state = S_WAIT_DEST;
            crsf_idx   = 0;
        }
        break;
    }
}


int format_channels(char *msg, int n)
{
	int pos = 0;

	for (int i = 0; i < n; i++) {
		uint16_t v = rc_us[i];
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

	// Zeilenende
	msg[pos++] = '\r';
	msg[pos++] = '\n';
	msg[pos]   = '\0'; // String-Ende

	return pos;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart3) {
		crsf_consume_byte(uart3_rx_byte);

		/* If we set a breakpoint in the callback function, we must clear ORE bit */
		if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) {
			__HAL_UART_CLEAR_OREFLAG(&huart3);
		}

		// Naechsten Byte-Empfang sofort wieder starten
		HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t last_print_ms = 0;

	char msg[64];

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
  __disable_irq();  /* Disable global interrupts */

  /* CMSIS */
  SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;  // HCLK as clock source
  SysTick_Config(SystemCoreClock/1000); /* Start SysTick_Handler */

  /* Wait until key K1 is pressed (PC13, high active, pull-down) */
  while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));

  __enable_irq();  /* Enable global interrupts */

  /* Clear ORE bit in USART3 (overrun error bit) */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(&huart3);
  }

  if (HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1) != HAL_OK) {
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // Alle 100ms Kanalwerte in us ausgeben
	  if (HAL_GetTick() - last_print_ms >= 100) {
		  last_print_ms = HAL_GetTick();

		  int k = format_channels(msg, 8); // 8 Kanalwerte

		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, k, 50);
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
