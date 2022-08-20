/*
 * can_config.c
 *
 *   Created on: 27.02.2022
 *  Modified on: 28.03.2022
 *       Author: Krystian Sosin
 *      Version: 1.0.1
 *  Last change: Fix minor bugs.
 */

#include "main.h"
#include "can_config.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "stdbool.h"


/* Variables and arrays needed to proper work of CAN */
CAN_FilterTypeDef    sFilterConfig;
CAN_TxHeaderTypeDef  TxHeader;
CAN_RxHeaderTypeDef  RxHeader;
uint8_t				 TxData[8];
uint8_t				 RxData[8];
uint32_t			 TxMailbox;

/* Externs used in configs */

extern _Bool Write_AIR1_ON;
extern _Bool Write_AIR2_ON;
extern _Bool Write_PreCharge_ON;
extern _Bool Write_MAIN_Status;

extern uint8_t Read_Ins_resistance[2];
extern uint8_t Read_AIR_AVG[2];

/** 
 * CAN READ MESSAGE FRAME
 *
 * | RxID  |  2  | 0x3D | ADDR  |
 * | StdID | DLC | READ | RegID |
 *
 *
 */
// Enum of ReadRegs defined in can_config.h. Nothing needs to be configured here.

/**
 *  CAN RESPONSE MESSAGE FRAME
 *
 * | TxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | RegID | DATA_1 | ... | DATA_N |
 *
 **/
ResponseMessageFrame ResponseMessage[NUMBER_OF_READ_REGS] =
{
		{ //AIR1 and AIR2 current value:
			.Response_DLC         = 3u,                         // Data length of response message
			.Read_ReactionHandler = ReadAIRsAmperageHandler,       // Handler of reaction to read request from MCU
			.Response_RegID       = Read_AIRs_Value_ID, // Address of regs which response refers
			.Response_Data1       = &Read_AIR_AVG[0],                 // AIR1 current value AVG LSB
			.Response_Data2       = &Read_AIR_AVG[1]                 // AIR2 current value AVG MSB
		},
		{//Insulation resistance value:
			.Response_DLC         = 3u,
			.Read_ReactionHandler = ReadInsulationResistanceValueHandler,
			.Response_RegID       = Read_Ins_R_Value_ID,
			.Response_Data1       = &Read_Ins_resistance[0],                // Insulation resistance LSB
			.Response_Data2       = &Read_Ins_resistance[1]                // Insulation resistance MSB
		}
};

/**
 *  CAN WRITE MESSAGE FRAME
 *
 * | RxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | WRITE | DATA_1 | ... | DATA_N |
 *
 **/
WriteMessageFrame WriteMessage[NUMBER_OF_WRITE_REGS] =
{
		{
			.Write_RegID           = Write_Tractive_System_State_ID, 	   // Reg which should be written by MCU command
			.Write_ReactionHandler = WriteTractiveSystemStateHandler,        // Handler of reaction to write request from MCU
			.Write_State1          = &Write_AIR1_ON,               // If this MCU command should change state of sth this pointer should point to variable which regards this state eg. if MCU want to light up brake light, this structure element should point to variable which contain the state of brake lights
			.Write_State2          = &Write_AIR2_ON,               // If this MCU command should change state of sth this pointer should point to variable which regards this state eg. if MCU want to light up brake light, this structure element should point to variable which contain the state of brake lights
			.Write_State3 		   = &Write_PreCharge_ON
		},
		{
			.Write_RegID           = Write_MAIN_State_ID, 					// Reg which should be written by MCU command
			.Write_ReactionHandler = WriteMainStatusHandler,       // Handler of reaction to write request from MCU
			.Write_State1          = &Write_MAIN_Status            // If this MCU command should change state of sth this pointer should point to variable which regards this state eg. if MCU want to light up brake light, this structure element should point to variable which contain the state of brake lights
		}
};

/**
 *  CAN ERROR MESSAGE FRAME
 *
 * | TxID  |  2  | 0x1D  |    ID   |
 * | StdID | DLC | ERROR | ErrorID |
 *
 */
// Enum of ErrorRegs defined in can_config.h. Nothing needs to be configured here.


/** CAN_Init
 * @brief Function to ensure proper work of CAN interface
          - configuration of filter and calling essential functions of CAN initialization
          Filter configured in accordance with E&S Team Project Guidelines.
 *
 * @retval None.
 **/
void CAN_Init(void)
{
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = Rx_ID << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFFF << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

//	CAN_Register_Callback(CAN_Error_Handler);

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.StdId = Tx_ID;
	TxHeader.ExtId = 0x0000;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
}

/** HAL_CAN_RxFifo0MsgPendingCallback
 * @brief HAL Callback to handle interuption from CAN new message
 * 
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 *
 * @retval None 
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Receive(hcan, &RxHeader, RxData);
	CAN_On_Receive(RxData);
}

/** CAN_On_Receive
 * @brief Function to procces received message - checking if message is read or write command and call specific handler
 * 
 * @param RxData pointer to uint8_t array which stores received data
 * 
 * @retval None
 **/
void CAN_On_Receive(uint8_t *RxData)
{
	if(Read_RequestMessage == RxData[ReadMessage])
	{
		CAN_Respond();
	}
	else
	{
		CAN_ProcessWriteCommand();
	}
}

/** CAN_Receive
 * @brief Function to receive data via CAN and, if neccessary, report error of CAN connection
 * 
 * @param RxHeader CAN_RxHeaderTypeDef pointer to structure that contains RxHeader configuration.
 * @param RxData uint8_t pointer to array that will contain received data.
 * 
 * @retval None.
 **/
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if(HAL_CAN_GetRxMessage(CANPointer, CAN_RX_FIFO0, RxHeader, RxData) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
};

/** CAN_Transmit
 * @brief Function to transmit data via CAN and, if neccessary, report error of CAN connection
 * 
 * @param TxHeader CAN_TxHeaderTypeDef pointer to structure that contains TxHeader configuration.
 * @param TxDLC uint8_t variable which contains Date Length of CAN message.
 * @param TxData uint8_t pointer to array that contains data to transmit.
 * @param TxMailbox uint32_t pointer to array that contains whole CAN message to transmit.
 * 
 * @retval None.
 **/
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox)
{
	TxHeader->DLC = TxDLC;
	if(HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, TxMailbox) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
}

/** CAN_Respond
 * @brief Function to respond in connection with read request from MCU
 * 
 * @retval None.
 **/
void CAN_Respond(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_READ_REGS; i++)
	{
		if (ResponseMessage[i].Response_RegID == RxData[ReadRegID])
		{
			ResponseMessage[i].Read_ReactionHandler();
		}
	}
}

/** CAN_ProcessWriteCommand
 * @brief Function to process write command
 * 
 * @retval None.
 **/
void CAN_ProcessWriteCommand(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_WRITE_REGS; i++)
	{
		if (WriteMessage[i].Write_RegID == RxData[WriteMessage_reg])
		{
			CAN_AcknowledgeWriteMessage(WriteMessage[i].Write_RegID);
			WriteMessage[i].Write_ReactionHandler();
		}
	}
}

/** CAN_AcknowledgeWriteMessage
 * @brief Function to send acknowledment received write instruction via CAN
 *
 * @param WriteReqID ID of received write instruction
 *
 * @retval None.
 **/
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID)
{
	TxData[AcknowledgmentMessage_reg] = Write_AcknowledgmentMessage; // 1st Data Byte: Standard Write Acknowledgment instruction
	TxData[WriteRegID] = WriteReqID;                                 // 2nd Data Byte: Acknowledged Received Write Command ReqID
	CAN_Transmit(&TxHeader, ACKNOWLEDMENT_DLC, TxData, &TxMailbox);  // Transmit Data
}

/** CAN_ReportError
 * @brief Function to report error via CAN
 * 
 * @param CANPointer pointer to a CAN_HandleTypeDef structure that contains
 *        the configuration information for the specified CAN.
 * @param ErrorID ID of reported Error Register]
 * 
 * @retval None.
 **/
void CAN_ReportError(ErrorRegsID ErrorID) //tego uzywam np. w IMD_Check
{
	TxData[ErrorMessage_reg] = Error_ReportMessage;             // 1st Data Byte: Standard Error Report instruction 
	TxData[ErrorRegID] = ErrorID;                            // 2nd Data Byte: Reported Error ID
	CAN_Transmit(&TxHeader, ERROR_DLC, TxData, &TxMailbox); // Transmit Data
}


/** CAN_Error_Handler
 * @brief General error handler of CAN connection and communication
 * 
 * @retval None.
 * */
void CANBUS_Error_Handler(void)
{
	__disable_irq();
	/*
	Put here behaviour of ECU when error will be occured.
	*/
}

/** ReadAIRsAmperageHandler
 * @brief Handler which reads and sends actual average value of AIRs amperage
 *
 * @retval None
 **/
void ReadAIRsAmperageHandler(void)
{
	AIR1_AIR2_Current_Measurment();

	TxData[ResponseRegID] = ResponseMessage[0].Response_RegID;             		  		// Response ID
	TxData[ResponseData1] = *( ResponseMessage[0].Response_Data1 );                		// AIR1 current value AVG LSB
	TxData[ResponseData2] = *( ResponseMessage[0].Response_Data2 ); 					// AIR1 current value AVG MSB
	CAN_Transmit(&TxHeader, ResponseMessage[0].Response_DLC, TxData, &TxMailbox); 		// Transmit Data
}

/** ReadInsulationResistanceValueHandler
 * @brief Handler which reads and sends actual frame insulation resistance value
 *
 * @retval None
 **/
void ReadInsulationResistanceValueHandler(void)
{
	TxData[ResponseRegID] = ResponseMessage[1].Response_RegID;             		  		// Response ID
	TxData[ResponseData1] = *( ResponseMessage[1].Response_Data1 );                    	// Insulation resistance LSB
	TxData[ResponseData2] = *( ResponseMessage[1].Response_Data2 ); 					// Insulation resistance MSB
	CAN_Transmit(&TxHeader, ResponseMessage[1].Response_DLC, TxData, &TxMailbox); 		// Transmit Data
}

/** WriteTractiveSystemStateHandler
 * @brief Handler which turns ON/OFF tractive system
 *
 * @retval None
 **/
void WriteTractiveSystemStateHandler(void)
{
	*( WriteMessage[0].Write_State1 ) = RxData[WriteData1];
	*( WriteMessage[0].Write_State2 ) = RxData[WriteData1];
	*( WriteMessage[0].Write_State3 ) = RxData[WriteData1];
	HAL_GPIO_WritePin(AIR1_ON_uC_GPIO_Port, AIR1_ON_uC_Pin, *( WriteMessage[0].Write_State1 ));
	HAL_GPIO_WritePin(Precharge_ON_GPIO_Port, Precharge_ON_Pin, *( WriteMessage[0].Write_State3 ));
	HAL_GPIO_WritePin(AIR2_ON_uC_GPIO_Port, AIR2_ON_uC_Pin, *( WriteMessage[0].Write_State2 ));
}

/** WriteMainStatusHandler
 * @brief Handler which update MAIN status to monitor whether its alive or not
 *
 * @retval None
 **/
void WriteMainStatusHandler(void)
{
	*( WriteMessage[1].Write_State1 ) = RxData[WriteData1];
}
