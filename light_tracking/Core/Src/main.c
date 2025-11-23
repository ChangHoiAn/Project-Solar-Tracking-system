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
#include <stdio.h>
#include <string.h>

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)

#define EEPROM_ADDR 0x08060000

#define IN1_Pin GPIO_PIN_0
#define IN2_Pin GPIO_PIN_1
#define IN3_Pin GPIO_PIN_2
#define IN4_Pin GPIO_PIN_3
#define IN_GPIO_Port GPIOC

/* 모터 28BYJ-48 기준 스텝 상수 정의 */
/* Half-step 모드: 4096 스텝 = 360도 */
#define FULL_ROTATION_STEPS   4096                    // 1회전 = 360도
#define QUARTER_ROTATION_STEPS (FULL_ROTATION_STEPS / 4)  // 90도 = 1024스텝
#define HALF_ROTATION_STEPS    (FULL_ROTATION_STEPS / 2)  // 180도 = 2048스텝

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t adcValues[5];
char msg[64];
uint32_t lastUpdate = 0;
static int currentIndex;
static uint8_t lcd_error_count = 0;
uint32_t ledOnTime = 0;
uint8_t ledActive = 0;
uint8_t stepSequence[8][4] = {
		{1,0,0,1},
		{1,0,0,0},
		{1,1,0,0},
		{0,1,0,0},
		{0,1,1,0},
		{0,0,1,0},
		{0,0,1,1},
		{0,0,0,1}
};
typedef struct {
    int active;           // 동작 중 여부
    int direction;        // 1: CW, -1: CCW
    int totalSteps;       // 전체 회전 스텝 수
    int done;             // 완료 여부
    int stepIndex;        // 시퀀스용 인덱스
    int stepCount;        // 현재까지 회전한 스텝 수
    uint32_t lastTick;    // 마지막 스텝 시간
} MotorControl_t;

MotorControl_t motor = {0};  // 모터 상태 구조체
typedef struct {
    int valid;         // 큐에 명령이 존재함
    int direction;     // 1 또는 -1
    int totalSteps;    // 1024 또는 2048
    int nextIndex;     // 회전 후 예상 인덱스 (EEPROM 기록용)
} MotorQueue_t;

MotorQueue_t motorQueue = {0};
volatile uint32_t lcdTick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void SendStatusUART(float solarVoltage, int currentIndex);

/* LCD 관련 함수 프로토타입 --------------------------------------*/
void LCD_SendCmd(char cmd);
void LCD_SendData(char data);
void LCD_Init(void);
void LCD_SetCursor(int row, int col);
void LCD_Print(char *str);
void EEPROM_Write(uint32_t data);
uint32_t EEPROM_Read(void);

/* 모터 제어 함수 프로토타입 --------------------------------------*/
void StepMotor(int step);

void LCD_Wait(uint32_t delay_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SendStatusUART(float solarVoltage, int currentIndex)
{
    char posChar[3];

    switch (currentIndex)
    {
			case 0: strcpy(posChar, "BR"); break;
			case 1: strcpy(posChar, "TR"); break;
			case 2: strcpy(posChar, "TL"); break;
			case 3: strcpy(posChar, "BL"); break;
    }

    snprintf(msg, sizeof(msg),
             "TL:%4u  TR:%4u  BL:%4u  BR:%4u  Solar:%.4fV  POS:%s\r\n",
             (unsigned int)adcValues[2], // TL
             (unsigned int)adcValues[1], // TR
             (unsigned int)adcValues[3], // BL
             (unsigned int)adcValues[0], // BR
             solarVoltage,
             posChar);

    /* LED ON (비동기) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    ledOnTime = HAL_GetTick();
    ledActive = 1;

    /* UART 송신 */
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
void EEPROM_Write(uint32_t data)
{
    // 현재 저장된 값 읽기
    uint32_t current = *(uint32_t*)EEPROM_ADDR;

    // 값이 같으면 플래시 작업 생략
    if (current == data)
        return;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_7;   // 마지막 섹터 (보드에 맞게)
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EEPROM_ADDR, data);

    HAL_FLASH_Lock();
}

uint32_t EEPROM_Read(void)
{
  return *(uint32_t*)EEPROM_ADDR;
}
void Motor_Queue(int direction, int steps, int targetIndex)
{
    motorQueue.valid = 1;
    motorQueue.direction = direction;
    motorQueue.totalSteps = steps;
    motorQueue.nextIndex = targetIndex;
}
void StepMotor(int step)
{
    HAL_GPIO_WritePin(IN_GPIO_Port, IN1_Pin, stepSequence[step][0]);
    HAL_GPIO_WritePin(IN_GPIO_Port, IN2_Pin, stepSequence[step][1]);
    HAL_GPIO_WritePin(IN_GPIO_Port, IN3_Pin, stepSequence[step][2]);
    HAL_GPIO_WritePin(IN_GPIO_Port, IN4_Pin, stepSequence[step][3]);
}
void Motor_Start(int direction, int steps)
{
    if (motor.active) return;  // 현재 회전 중이면 무시

    motor.active = 1;
    motor.direction = direction;
    motor.totalSteps = steps;
    motor.stepIndex = 0;   // 또는 static 유지 원하면 생략 가능
    motor.stepCount = 0;
    motor.done = 0;
    motor.lastTick = HAL_GetTick();
}
void Motor_ProcessStep()
{
    if (!motor.active || motor.done) return;

    if (HAL_GetTick() - motor.lastTick < 2) return;  // 2ms마다
    motor.lastTick = HAL_GetTick();

    motor.stepIndex = (motor.stepIndex + motor.direction + 8) % 8;
    StepMotor(motor.stepIndex);

    motor.stepCount++;
    if (motor.stepCount >= motor.totalSteps)
    {
        motor.active = 0;
        motor.done = 1;

        // 큐가 존재하면 다음 회전 실행
        if (motorQueue.valid)
        {
            Motor_Start(motorQueue.direction, motorQueue.totalSteps);
            currentIndex = motorQueue.nextIndex;
            EEPROM_Write(currentIndex);
            motorQueue.valid = 0;
        }
    }
}
void LCD_SendCmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);
    data_t[0] = data_u | 0x0C;
    data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C;
    data_t[3] = data_l | 0x08;

    if (HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100) != HAL_OK)
    {
        lcd_error_count++;
    }
    else
    {
        lcd_error_count = 0; // 정상 동작 시 리셋
    }

    LCD_Wait(2);

    // 에러 누적되면 LCD 재초기화
    if (lcd_error_count >= 3)
    {
        LCD_Init();
        lcd_error_count = 0;
    }
}

void LCD_SendData(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);
    data_t[0] = data_u | 0x0D;
    data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D;
    data_t[3] = data_l | 0x09;

    if (HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100) != HAL_OK)
    {
        lcd_error_count++;
    }
    else
    {
        lcd_error_count = 0;
    }

    LCD_Wait(2);

    if (lcd_error_count >= 3)
    {
        LCD_Init();
        lcd_error_count = 0;
    }
}
void LCD_Init(void)
{
    LCD_Wait(50); // 전원 안정화 대기

    LCD_SendCmd(0x30); LCD_Wait(5);
    LCD_SendCmd(0x30); LCD_Wait(5);
    LCD_SendCmd(0x30); LCD_Wait(5);
    LCD_SendCmd(0x20); LCD_Wait(5); // 4-bit mode

    LCD_SendCmd(0x28); // Function set: 4-bit, 2-line, 5x8 dots
    LCD_SendCmd(0x08); // Display off
    LCD_SendCmd(0x01); LCD_Wait(3); // Clear display (2ms 이상)
    LCD_SendCmd(0x06); // Entry mode set: increment
    LCD_SendCmd(0x0C); // Display on, cursor off
}
void LCD_SetCursor(int row, int col)
{
  int row_offsets[] = {0x00, 0x40};
  LCD_SendCmd(0x80 | (col + row_offsets[row]));
}

void LCD_Print(char *str)
{
  while (*str) LCD_SendData(*str++);
}
uint32_t GetLCDTick(void)
{
    return lcdTick;
}
void LCD_Wait(uint32_t delay_ms)
{
    uint32_t start = GetLCDTick();
    while ((GetLCDTick() - start) < delay_ms)
    {
        Motor_ProcessStep();  // <-- 중요: 모터 회전 중에도 처리 계속
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  HAL_ADC_Start_DMA(&hadc1, adcValues, 5);

	LCD_SetCursor(0, 0);
	char lcdLine[17];
	snprintf(lcdLine, sizeof(lcdLine), "Solar:");
	LCD_Print(lcdLine);

	LCD_SetCursor(1, 0);
	snprintf(lcdLine, sizeof(lcdLine), "Position:");
	LCD_Print(lcdLine);
				
  static int lastDirection = 0;
  static int mismatchCount = 0;
  int sensorToDir[4] = {0, 1, 2, 3};
	
  uint32_t storedIndex = EEPROM_Read();
  currentIndex = (storedIndex <= 3) ? storedIndex : 0;
	
  // 초기 보정
  int rawMaxIndex = 0;
  uint32_t maxValue = 0, secondMax = 0;
  for (int i = 0; i < 4; i++)
  {
    if (adcValues[i] > maxValue)
    {
      maxValue = adcValues[i];
      rawMaxIndex = i;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    if (i != rawMaxIndex && adcValues[i] > secondMax)
      secondMax = adcValues[i];
  }

  int maxIndex = sensorToDir[rawMaxIndex];
  if ((maxValue - secondMax) >= 300 && maxIndex != currentIndex)
  {
    int diff = maxIndex - currentIndex;
    if (diff > 3) diff -= 4;
    if (diff < -3) diff += 4;

    if (diff == 1 || diff == -3)
    {
      Motor_Start(1, QUARTER_ROTATION_STEPS);
      lastDirection = 1;
    }
    else if (diff == -1 || diff == 3)
    {
      Motor_Start(-1, QUARTER_ROTATION_STEPS);
      lastDirection = -1;
    }
    else if (diff == 2 || diff == -2)
    {
      int dir = (lastDirection == 1) ? -1 : 1;
			Motor_Start(dir, HALF_ROTATION_STEPS);
      lastDirection = dir;
    }

    currentIndex = maxIndex;
    EEPROM_Write(currentIndex);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Motor_ProcessStep();
		
		if (ledActive && (HAL_GetTick() - ledOnTime >= 100))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			ledActive = 0;
		}
		
		/* --- 태양광 전압 계산 추가 --- */
		float solarVoltage = (adcValues[4] * 3.3f / 4095.0f) * 2.0f;

    /* 가장 밝은 센서 찾기 */
    rawMaxIndex = 0;
    maxValue = adcValues[0];
    for (int i = 1; i < 4; i++)
    {
      if (adcValues[i] > maxValue)
      {
        maxValue = adcValues[i];
        rawMaxIndex = i;
      }
    }

    /* 방향 인덱스로 변환 */
    maxIndex = sensorToDir[rawMaxIndex];

    /* 두 번째로 큰 값 찾기 (Threshold 체크용) */
    secondMax = 0;
    for (int i = 0; i < 4; i++)
    {
      if (i != rawMaxIndex && adcValues[i] > secondMax)
      {
        secondMax = adcValues[i];
      }
    }

    if (HAL_GetTick() - lastUpdate >= 500)
    {
        // UART 송신
        SendStatusUART(solarVoltage, currentIndex);

        // LCD 출력
        char posChar[3];
        switch (currentIndex)
        {
					case 0: strcpy(posChar, "BR"); break;
					case 1: strcpy(posChar, "TR"); break;
					case 2: strcpy(posChar, "TL"); break;
					case 3: strcpy(posChar, "BL"); break;
        }

        LCD_SetCursor(0, 7);
        char lcdLine[17];
        snprintf(lcdLine, sizeof(lcdLine), "%.3fV", solarVoltage);
        LCD_Print(lcdLine);

        LCD_SetCursor(1, 10);
        snprintf(lcdLine, sizeof(lcdLine), "%s", posChar);
        LCD_Print(lcdLine);

        lastUpdate = HAL_GetTick();
    }

    /* 센서값 차이가 Threshold보다 작으면 회전하지 않음 */
    if ((maxValue - secondMax) < 300) continue;
		
    /* 목표 방향과 현재 방향 차이 계산 */
    int diff = maxIndex - currentIndex;

    /* diff 정규화 (-3 ~ +3) */
    if (diff > 3) diff -= 4;
    if (diff < -3) diff += 4;

		if (diff != 0 && !motor.active)
		{
				mismatchCount++;
				if (mismatchCount >= 2)
				{
						int dir = 0;
						int steps = 0;

						if (diff == 1 || diff == -3) {
								dir = 1;
								steps = QUARTER_ROTATION_STEPS;
						}
						else if (diff == -1 || diff == 3) {
								dir = -1;
								steps = QUARTER_ROTATION_STEPS;
						}
						else if (diff == 2 || diff == -2) {
								dir = (lastDirection == 1) ? -1 : 1;
								steps = HALF_ROTATION_STEPS;
						}

						if (dir != 0) {
								Motor_Start(dir, steps);
								lastDirection = dir;
								currentIndex = maxIndex;
								EEPROM_Write(currentIndex);
						}

						mismatchCount = 0;
				}
		}
		else if (diff != 0 && motor.active)
		{
				// 회전 중일 때는 큐에 저장
				int dir = 0;
				int steps = 0;

				if (diff == 1 || diff == -3) {
						dir = 1;
						steps = QUARTER_ROTATION_STEPS;
				}
				else if (diff == -1 || diff == 3) {
						dir = -1;
						steps = QUARTER_ROTATION_STEPS;
				}
				else if (diff == 2 || diff == -2) {
						dir = (lastDirection == 1) ? -1 : 1;
						steps = HALF_ROTATION_STEPS;
				}

				if (dir != 0)
				{
						Motor_Queue(dir, steps, maxIndex);
				}

				mismatchCount = 0; // 현재 값은 큐에 넣었으므로 초기화
		}
		else if (diff == 0)
		{
				mismatchCount = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
