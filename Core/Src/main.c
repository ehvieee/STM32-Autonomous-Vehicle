#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct State {
	char * Movement;
	float Speed;
    int Delay;
	struct State *Next[2];
};
typedef struct State STyp;

#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_9
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
char strCopy[15];

#define IN1 GPIO_PIN_4
#define IN1_PORT GPIOB
#define IN2 GPIO_PIN_5
#define IN2_PORT GPIOB
#define IN3 GPIO_PIN_3
#define IN3_PORT GPIOB
#define IN4 GPIO_PIN_10
#define IN4_PORT GPIOA


#define MF &FSM[0]
#define ST &FSM[1]
#define SL &FSM[2]
#define TL &FSM[3]
#define STLT &FSM[4]
#define GL &FSM[5]

#define SR &FSM[6]
#define TR &FSM[7]
#define STRT &FSM[8]
#define GR &FSM[9]
#define RE &FSM[10]
#define IST &FSM[11]


STyp FSM[12] = {
		{"FORWARD",60,0,{MF,ST}},		// moving forward, detects less than 40 STOP
		{"STOP",0,1000,{SL,SL}}, //STOP for half second, then looks left
		{"STOP",0,1000,{TL,SR}}, // if clear turn left, else look right
		{"LEFT",50,430,{STLT,STLT}}, // turn left
		{"STOP",0,1000,{GL,GL}}, // after turning left stop for 1sec
		{"LFORWARD",60,400,{MF,MF}}, //gradually readjust rear wheel
		{"STOP",0,1000,{TR,RE}}, // if clear turn right, else reverse
		{"RIGHT",50,430,{STRT,STRT}}, // turn right
		{"STOP",0,1000,{GR,GR}}, // stop for 1sec after turning right
		{"RFORWARD",60,400,{MF,MF}}, //gradually readjust rear wheel
		{"REVERSE",60,400,{ST,ST}}, // reverse for 400ms
		{"STOP",0,2000,{MF,MF}} //stop for 2sec

};

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void PassengerSideMotor(float duty);
void DriverSideMotor(float duty);
void SERVOmove(float Duty);
void moveForward(float speed);
void motorSTOP(void);
void moveRight(void);
void moveLeft(void);
void move(char *direction,float speed);
float readSensor(void);

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

int main(void)
{
  float dist;
  int blocked;
  STyp *Pt;
  Pt = IST;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // CH1 used for passenger motor same freq
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //TIM3 pwm used for servo motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //CH3 used for driver motor same freq

  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  while (1)
  {
	  move(Pt->Movement, Pt->Speed);
	  if(Pt == SL){
		  SERVOmove(12.5); // look left
	  } else if (Pt == SR) {
		  SERVOmove(3); // look right
	  } else {
		  SERVOmove(8.2); //look forward
	  }
	  HAL_Delay(Pt->Delay);
	  dist = readSensor();
	  if (dist <= 30.0){
		  blocked = 1;
	  } else {
		  blocked = 0;
	  }
	  HAL_Delay(100); //delay of 100us to slow the sensor speed, too fast freq for it gives some 0 readings
	  Pt = Pt->Next[blocked];

  }

}

float readSensor(void){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10); // delay 10us to send 10us pulse
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop

	while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	Value1 = __HAL_TIM_GET_COUNTER (&htim1);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)

	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	Value2 = __HAL_TIM_GET_COUNTER (&htim1);

	Distance = (Value2-Value1)* 0.034 / 2; // conversion into cm using speed of light
	return Distance;
}

void move(char *direction,float speed){
	if (strcmp(direction,"FORWARD") == 0) {
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_SET);   // Set IN1 to HIGH
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_RESET); // Set IN2 to LOW
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_RESET); // Set IN3 to LOW
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_SET);   // Set IN4 to HIGH
		PassengerSideMotor(speed-1);
		DriverSideMotor(speed);
	} else if (strcmp(direction,"RIGHT") == 0){
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_SET);   // Set IN1 to HIGH
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_RESET); // Set IN2 to LOW
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_SET); // Set IN3 to HIGH
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_RESET);   // Set IN4 to LOW
		PassengerSideMotor(speed);
		DriverSideMotor(speed);
	} else if (strcmp(direction,"RFORWARD") == 0) {
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_SET);   // Set IN1 to HIGH
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_RESET); // Set IN2 to LOW
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_RESET); // Set IN3 to LOW
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_SET);   // Set IN4 to HIGH
		PassengerSideMotor(speed);
		DriverSideMotor(speed-10);
	} else if (strcmp(direction,"LEFT") == 0) {
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_RESET);   // Set IN1 to LOW
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_SET); // Set IN2 to HIGH
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_RESET); // Set IN3 to LOW
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_SET);   // Set IN4 to HIGH
		PassengerSideMotor(speed);
		DriverSideMotor(speed);
	} else if (strcmp(direction,"LFORWARD") == 0){
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_SET);   // Set IN1 to HIGH
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_RESET); // Set IN2 to LOW
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_RESET); // Set IN3 to LOW
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_SET);   // Set IN4 to HIGH
		PassengerSideMotor(speed-15);
		DriverSideMotor(speed+5);
	} else if (strcmp(direction,"STOP") == 0){
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_RESET);   // Set IN1 to LOW
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_RESET); // Set IN2 to LOW
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_RESET);   // Set IN3 to LOW
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_RESET); // Set IN4 to LOW
	} else if (strcmp(direction,"REVERSE") == 0) {
		HAL_GPIO_WritePin(IN1_PORT, IN1, GPIO_PIN_RESET);   // Set IN1 to LOW
		HAL_GPIO_WritePin(IN2_PORT, IN2, GPIO_PIN_SET); // Set IN2 to HIGH
		HAL_GPIO_WritePin(IN3_PORT, IN3, GPIO_PIN_SET);   // Set IN3 to HIGH
		HAL_GPIO_WritePin(IN4_PORT, IN4, GPIO_PIN_RESET); // Set IN4 to LOW
		PassengerSideMotor(speed);
		DriverSideMotor(speed-10);
	}
}


void PassengerSideMotor(float Duty)
{
	uint16_t AutoReload, DutyCycleSet;

	AutoReload = __HAL_TIM_GET_AUTORELOAD(&htim4);
	DutyCycleSet = AutoReload * Duty / 100.0;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DutyCycleSet);
}

void DriverSideMotor(float Duty)
{
	uint16_t AutoReload, DutyCycleSet;

	AutoReload = __HAL_TIM_GET_AUTORELOAD(&htim4);
	DutyCycleSet = AutoReload * Duty / 100.0;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, DutyCycleSet);
}

void SERVOmove(float Duty)
{
	uint16_t AutoReload, DutyCycleSet;

	AutoReload = __HAL_TIM_GET_AUTORELOAD(&htim3);
	DutyCycleSet = AutoReload * Duty / 100.0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DutyCycleSet);
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 66;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
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
#endif /* USE_FULL_ASSERT */
