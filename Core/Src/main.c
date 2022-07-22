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
#include "can_config.h"
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*IMD PWM */
uint32_t IC_Value = 0;
volatile uint32_t Frequency = 0;
float Duty_Cycle = 0;
uint16_t Ins_resistance;
/*IMD PWM */

//uint8_t IMD_State;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void IMD_State_Check(void);
void IMD_BMS_Check(void);
void AIR1_AIR2_Check(void);
void IMD_ShortCircuitTo24VError_Handler(void);
void IMD_InsulationMeasurementError_Handler(void);
void IMD_UnderVoltageError_Handler(void);
void IMD_SpeedStartError_Handler(void);
void IMD_DeviceError_Handler(void);
void IMD_ConFaultEarthError_Handler(void);
void IMD_MalfunctionError_Handler(void);

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	/*PWM input capture */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); //main channel
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2); //indirect channel
	/*PWM input capture */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		AIR1_AIR2_Check();
		IMD_BMS_Check();
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //tim1, measuring IMD signal
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // interrupt triggered by channel 1
	{
		//read IC value
		IC_Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if(IC_Value != 0)
		{
			//calc duty cycle
			Duty_Cycle = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * 100) / IC_Value; //duty cycle in %
			Frequency = 8000000 / IC_Value; //frequency of input signal, 8MHz - tim1 clock freq
			Ins_resistance = (90 * 1200) / (Duty_Cycle - 5) - 1200; //in kOm
			//TxData2[4] = ((Ins_resistance >> 0) & 0xFF); //lower 8 bits
			//TxData2[5] = ((Ins_resistance >> 8) & 0xFF); //higher 8 bits
		}
		switch(Frequency)
		{
		case 0:
			if(HAL_GPIO_ReadPin(IMD_PWM_IN_GPIO_Port, IMD_PWM_IN_Pin) == GPIO_PIN_SET) IMD_ShortCircuitTo24VError_Handler();
			break;
		case 10:
			if( ((int8_t)Duty_Cycle >= 5) && ((int8_t)Duty_Cycle <= 95) ); //all OK
			else IMD_InsulationMeasurementError_Handler();
			break;
		case 20:
			if( ((int8_t)Duty_Cycle >= 5) && ((int8_t)Duty_Cycle <= 95) ) IMD_UnderVoltageError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 30:
			if( ((int8_t)Duty_Cycle >= 5) && ((int8_t)Duty_Cycle <= 10) ); //all OK
			else if( ((int8_t)Duty_Cycle >= 90) && ((int8_t)Duty_Cycle <= 95) ) IMD_SpeedStartError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 40:
			if( ((int8_t)Duty_Cycle >= 47.5) && ((int8_t)Duty_Cycle <= 52.5) ) IMD_DeviceError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		case 50:
			if( ((int8_t)Duty_Cycle >= 47.5) && ((int8_t)Duty_Cycle <= 52.5) ) IMD_ConFaultEarthError_Handler();
			else IMD_MalfunctionError_Handler();
			break;
		}

//		if( (Frequency == 0) && (HAL_GPIO_ReadPin(IMD_PWM_IN_GPIO_Port, IMD_PWM_IN_Pin) == GPIO_PIN_SET) )
//			IMD_ShortCircuitTo24VError_Handler();
//		else if( ())
//		else if(Frequency == 20)
//			IMD_UnderVoltageError_Handler();
//		else if(Frequency == 50)
//			IMD_ConFaultEarthError_Handler();
	}
}

void IMD_ShortCircuitTo24VError_Handler(void)
{
	CAN_ReportError(IMD_Short_Circuit_24V_ERROR);
}

void IMD_InsulationMeasurementError_Handler(void)
{
	CAN_ReportError(IMD_Insulation_Measurement_ERROR);
}

void IMD_UnderVoltageError_Handler(void)
{
	CAN_ReportError(IMD_Under_Voltage_ERROR);
}

void IMD_SpeedStartError_Handler(void)
{
	CAN_ReportError(IMD_Speed_Start_ERROR);
}

void IMD_DeviceError_Handler(void)
{
	CAN_ReportError(IMD_Device_ERROR);
}

void IMD_ConFaultEarthError_Handler(void)
{
	CAN_ReportError(IMD_Con_Fault_Earth_ERROR);
}

void IMD_MalfunctionError_Handler(void) //Wrong Duty Cycle
{
	CAN_ReportError(IMD_Malfunction_ERROR);
}

void IMD_BMS_Check(void)
{
	_Bool IMD_Status = HAL_GPIO_ReadPin(IMD_STATUS_uC_GPIO_Port, IMD_STATUS_uC_Pin);
	if( (IMD_Status == 0) && (Frequency == 0))
	{
		CAN_ReportError(IMD_SDC_ERROR);
	}
	_Bool BMS_Status = HAL_GPIO_ReadPin(BMS_STATUS_uC_GPIO_Port, BMS_STATUS_uC_Pin);
	if (BMS_Status == 0)
	{
		CAN_ReportError(BMS_SDC_ERROR);
	}
}

void AIR1_AIR2_Check(void)
{
	//variables made for test purposes only (or not xD)
	_Bool AIR1_ON = HAL_GPIO_ReadPin(AIR1_ON_uC_GPIO_Port, AIR1_ON_uC_Pin);
	_Bool AIR1_STATUS = HAL_GPIO_ReadPin(AIR1_STATUS_uC_GPIO_Port, AIR1_STATUS_uC_Pin);

	_Bool AIR2_ON = HAL_GPIO_ReadPin(AIR2_ON_uC_GPIO_Port, AIR2_ON_uC_Pin);
	_Bool AIR2_STATUS = HAL_GPIO_ReadPin(AIR2_STATUS_uC_GPIO_Port, AIR2_STATUS_uC_Pin);

	//K1 state check, if Kilovac 1 is turned on but doesn't conduct current, send error
	if((AIR1_ON == GPIO_PIN_SET) && (AIR1_STATUS == GPIO_PIN_RESET))
	{
		CAN_ReportError(AIR1_ERROR);

	}
	//K2 state check, if Kilovac 2 is turned on but doesn't conduct current, send error
	if((AIR2_ON == GPIO_PIN_SET) && (AIR2_STATUS == GPIO_PIN_RESET))
	{
		CAN_ReportError(AIR2_ERROR);
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
