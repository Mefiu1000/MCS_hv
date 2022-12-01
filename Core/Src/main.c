/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <can_config.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Kody do ramek
#define TxID 0x001A  //MCS_hv
#define RxID 0x001F  //MCS_hv

#define MCU_Tx 0x0190 //0x0190
#define MCU_Rx 0x0191 //0x0191

#define WRITE 0x2D
#define READ 0x3D
#define ERROR_D 0x001D

#define MAX_CURRENT_DIVERGENCE 0.5 	// random value, change later
#define MAX_CURRENT 5 				// random value, change later

#define MAIN_TIMEOUT 5000   		// random value, change later
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*IMD PWM */
extern ResponseMessageFrame ResponseMessage[NUMBER_OF_READ_REGS];
uint32_t IC_Value = 0;
volatile uint32_t Frequency = 0;
float Duty_Cycle = 0;
uint8_t Read_Ins_resistance[2];
/*IMD PWM */

uint16_t AIR_P_Current, AIR_N_Current;
uint8_t Read_AIR_AVG[2];

_Bool Write_MAIN_Status = 0; //init value 0 or 1????

_Bool Write_AIRs_CONTROL;
_Bool AIR_N_STATUS, AIR_P_STATUS;

uint32_t Timer_MAIN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AIRs_Check(void);
void IMD_ShortCircuitTo24VError_Handler(void);
void IMD_InsulationMeasurementError_Handler(void);
void IMD_UnderVoltageError_Handler(void);
void IMD_SpeedStartError_Handler(void);
void IMD_DeviceError_Handler(void);
void IMD_ConFaultEarthError_Handler(void);
void IMD_MalfunctionError_Handler(void);
void AIRs_Current_Measurment(void);
void MAIN_Status_Check(void);
void Set_ADC_Channel(uint32_t Channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	/*PWM input capture */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //main channel
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2); //indirect channel
	/*PWM input capture */

	Timer_MAIN = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (( HAL_GetTick() - Timer_MAIN ) > MAIN_TIMEOUT)
		{
			MAIN_Status_Check();
		}
		AIRs_Check();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/** HAL_TIM_IC_CaptureCallback
 * @brief Callback in which frame insulation resistance value is measured by IMD.
 * Basing on signal frequency and duty cycle, condition is checked
 * and reported via CAN if necessary.
 *
 * @param htim pointer to structure that contains htim configuration.
 *
 * @retval None.
 **/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //tim1, measuring IMD signal
{
	uint16_t Temp;
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // interrupt triggered by channel 1
	{
		//read IC value
		IC_Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (IC_Value != 0)
		{
			//calc duty cycle
			Duty_Cycle = ( HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * 100 ) / IC_Value; //duty cycle in %
			Frequency = 8000000 / IC_Value; //frequency of input signal, 8MHz - tim1 clock freq
			Temp = (uint16_t)( ( 90 * 1200 ) / ( Duty_Cycle - 5 ) - 1200 ); //in kOm
			Read_Ins_resistance[0] = Temp; //LSB
			Read_Ins_resistance[1] = ( Temp >> 8 ); //MSB
		}
		switch (Frequency)
		{
		case 0:
			if (HAL_GPIO_ReadPin(IMD_M_HS_uC_GPIO_Port, IMD_M_HS_uC_Pin) == GPIO_PIN_SET) IMD_ShortCircuitTo24VError_Handler();
			break;
		case 10:
			if (( (int8_t)Duty_Cycle >= 5 ) && ( (int8_t)Duty_Cycle <= 95 )) ; //all OK
			else IMD_InsulationMeasurementError_Handler();
			break;
		case 20:
			if (( (int8_t)Duty_Cycle >= 5 ) && ( (int8_t)Duty_Cycle <= 95 )) IMD_UnderVoltageError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 30:
			if (( (int8_t)Duty_Cycle >= 5 ) && ( (int8_t)Duty_Cycle <= 10 )) ; //all OK
			else if (( (int8_t)Duty_Cycle >= 90 ) && ( (int8_t)Duty_Cycle <= 95 )) IMD_SpeedStartError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 40:
			if (( (int8_t)Duty_Cycle >= 47.5 ) && ( (int8_t)Duty_Cycle <= 52.5 )) IMD_DeviceError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 50:
			if (( (int8_t)Duty_Cycle >= 47.5 ) && ( (int8_t)Duty_Cycle <= 52.5 )) IMD_ConFaultEarthError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		}
	}
}

/** IMD_ShortCircuitTo24VError_Handler
 * @brief Function to report  short circuit error via CAN
 *
 * @retval None.
 **/
void IMD_ShortCircuitTo24VError_Handler(void)
{
	CAN_ReportError(Error_IMD_Short_Circuit_24V_ID);
}

/** IMD_InsulationMeasurementError_Handler
 * @brief Function to report too low/too high frame insulation error via CAN
 *
 * @retval None.
 **/
void IMD_InsulationMeasurementError_Handler(void)
{
	CAN_ReportError(Error_IMD_Insulation_Measurement_ID);
}

/** IMD_UnderVoltageError_Handler
 * @brief Function to report error via CAN when Tractive System voltage drops below 300V
 *
 * @retval None.
 **/
void IMD_UnderVoltageError_Handler(void)
{
	CAN_ReportError(Error_IMD_Under_Voltage_ID);
}

/** IMD_SpeedStartError_Handler
 * @brief Function to report Speed start measurement failure error via CAN
 *
 * @retval None.
 **/
void IMD_SpeedStartError_Handler(void)
{
	CAN_ReportError(Error_IMD_Speed_Start_ID);
}

/** IMD_DeviceError_Handler
 * @brief Function to report Device error via CAN
 *
 * @retval None.
 **/
void IMD_DeviceError_Handler(void)
{
	CAN_ReportError(Error_IMD_Device_ID);
}

/** IMD_ConFaultEarthError_Handler
 * @brief Function to report fault on the earth connection error via CAN
 *
 * @retval None.
 **/
void IMD_ConFaultEarthError_Handler(void)
{
	CAN_ReportError(Error_IMD_Con_Fault_Earth_ID);
}

/** IMD_MalfunctionError_Handler
 * @brief Function to report IMD malfunction
 * based on incorrect Duty Cycle for respective frequency
 *
 * @retval None.
 **/
void IMD_MalfunctionError_Handler(void)
{
	CAN_ReportError(Error_IMD_Malfunction_ID);
}

/** AIRs_Current_Measurment
 * @brief Function which measures AIRs actual amperage
 * and calculate their average value
 *
 * @retval None.
 **/
void AIRs_Current_Measurment(void)
{
	/* AIRs current measurement and check BEGIN */
	Set_ADC_Channel(ADC_CHANNEL_8); //Switch to channel 8
	HAL_ADC_Start(&hadc); //start conversion

	if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK)
	{
		AIR_N_Current = HAL_ADC_GetValue(&hadc); // Read AIR_P current value
	}

	Set_ADC_Channel(ADC_CHANNEL_9); //Switch to channel 9
	HAL_ADC_Start(&hadc); //start conversion

	if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK)
	{
		AIR_P_Current = HAL_ADC_GetValue(&hadc); // Read AIR_N current value
	}

	//Current calc V(I) = 0.0034 *I + 2.5 -> I = (V(I) - 2.5) / 0.0034, equation from datasheet
	AIR_P_Current = (uint16_t)( ( (float)( AIR_P_Current - 2.5 ) ) / 0.0034 );
	AIR_N_Current = (uint16_t)( ( (float)( AIR_N_Current - 2.5 ) ) / 0.0034 );
	Read_AIR_AVG[0] = ( ( AIR_P_Current + AIR_N_Current ) / 2 ); 		  //LSB
	Read_AIR_AVG[1] = ( ( ( AIR_P_Current + AIR_N_Current ) / 2 ) >> 8 ); //MSB
}

/** AIRs_Check
 * @brief Function which monitor AIRs amperage
 * and reports error via CAN in case of:
 * -overcurrent
 * -current divergence
 * -not conducting current
 *
 * @retval None.
 **/
void AIRs_Check(void)
{
	AIRs_Current_Measurment();

	if (AIR_P_Current > MAX_CURRENT)
	{
		CAN_ReportError(Error_AIR_P_Overcurrent_ID);
	}
	if (AIR_N_Current > MAX_CURRENT)
	{
		CAN_ReportError(Error_AIR_N_Overcurrent_ID);
	}
	if (abs(AIR_P_Current - AIR_N_Current) > MAX_CURRENT_DIVERGENCE)
	{
		CAN_ReportError(Error_AIRs_Current_Divergence_ID);
	}

	Write_AIRs_CONTROL = HAL_GPIO_ReadPin(AIRs_CONTROL_uC_GPIO_Port, AIRs_CONTROL_uC_Pin); //AIR_N turned on/off
	AIR_N_STATUS = HAL_GPIO_ReadPin(AIR_N_STATUS_uC_GPIO_Port,
	AIR_N_STATUS_uC_Pin); // AIR_N conducting current

	AIR_P_STATUS = HAL_GPIO_ReadPin(AIR_P_STATUS_uC_GPIO_Port,
	AIR_P_STATUS_uC_Pin); // AIR_P conducting current

	//AIR_N state check, if AIR_N is turned on but doesn't conduct current, send error
	if (( Write_AIRs_CONTROL == GPIO_PIN_SET ) && ( AIR_N_STATUS == GPIO_PIN_RESET ))
	{
		CAN_ReportError(Error_AIR_P_ID);
	}
	//Both airs should conduct current at the same time
	if( AIR_N_STATUS != AIR_P_STATUS )
	{
		CAN_ReportError(Error_AIR_N_ID);
	}
}

/** MAIN_Status_Check
 * @brief Function which checks MAIN status
 * and turns OFF Tractive System if MAIN is not ON
 *
 * @retval None.
 **/
void MAIN_Status_Check(void)
{
	if (Write_MAIN_Status == 0) //MAIN always should be ON
	{
		HAL_GPIO_WritePin(AIRs_CONTROL_uC_GPIO_Port, AIRs_CONTROL_uC_Pin, Write_MAIN_Status);
	}
	else //reset main status
	{
		Write_MAIN_Status = 0;
	}
}

/** Set_ADC_Channel
 * @brief Function which switches used ADC channel
 *
 * @param Channel Number of channel which we want to use
 *
 * @retval None.
 **/
void Set_ADC_Channel(uint32_t Channel)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = Channel;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}
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
