/*
 * can_config.h
 *
 *  Created on: 27.02.2022
 *      Author: Krystian Sosin
 *     Version: 1.0.0
 */

#ifndef INC_CAN_CONFIG_H_
#define INC_CAN_CONFIG_H_

#include "main.h"
#include "can.h"
#include "stdbool.h"

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF DEFINES----------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

#define NUMBER_OF_READ_REGS  (2U)
#define NUMBER_OF_WRITE_REGS (2U)
#define NUMBER_OF_ERROR_REGS (12U)
#define ERROR_DLC            (2U)
#define ACKNOWLEDMENT_DLC    (2U)
#define FIRST_ARRAY_ELEMENT  (0U)

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF DEFINES------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF ENUMS DEFINITIONS------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* Registers */
// All of Reg1 - RegN should be replaced by meaningful name of reg (don't delete prefixs)
typedef enum
{
	Read_AIRs_Value_ID = 0x01u, // ADC AIR_P AIR_N  current values channel 6, 7 read
	Read_Ins_R_Value_ID = 0x03u //Insulation resistance value
} ReadRegsID;

typedef enum
{
	Write_Tractive_System_State_ID = 0x02u, //AIRs and Pre-charge state
	Write_MAIN_Handshake_ID = 0xFFu
} WriteRegsID;

typedef enum
{
	Error_AIR_P_Overcurrent_ID = 0x001u,
	Error_AIR_N_Overcurrent_ID = 0x002u,
	Error_AIR_P_ID = 0x003u,
	Error_AIR_N_ID = 0x004u,
	Error_IMD_Short_Circuit_24V_ID = 0x005u,
	Error_IMD_Insulation_Measurement_ID = 0x006u,
	Error_IMD_Under_Voltage_ID = 0x007u,
	Error_IMD_Speed_Start_ID = 0x008u,
	Error_IMD_Device_ID = 0x009u,
	Error_IMD_Con_Fault_Earth_ID = 0x00Au,
	Error_IMD_Malfunction_ID = 0x00Bu,
	Error_AIRs_Current_Divergence_ID = 0x00Cu
} ErrorRegsID;

/* Enums to avoid magic numbers */
typedef enum
{
	ReadMessage,
	ReadRegID
} ReadFrameID;

typedef enum
{
	ResponseRegID,
	ResponseData1,
	ResponseData2
} ResponseFrame;

typedef enum
{
	WriteMessage_reg,
	WriteData1,
	WriteData2
} WriteFrame;

typedef enum
{
	ErrorMessage_reg,
	ErrorRegID
} ErrorFrame;

typedef enum
{
	AcknowledgmentMessage_reg,
	WriteRegID
} AcknowledgmentFrame;

typedef enum
{
	Error_ReportMessage = 0x1Du,
	Read_RequestMessage = 0x3Du,
	Write_AcknowledgmentMessage = 0x5Du
} CANStadardMessage;

typedef enum
{
	Rx_ID = 0x020Fu,
	Tx_ID = 0x020Au
} NodeAddress;


/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF ENUMS DEFINITIONS--------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF VARIABLES DEFINITIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

typedef void (*ReadReactionHandlerFuncPtr)(void);
typedef void (*WriteReactionHandlerFuncPtr)(void);

typedef struct
{
	WriteRegsID  Write_RegID;
	WriteReactionHandlerFuncPtr Write_ReactionHandler;
	bool*        Write_State1;
	bool*        Write_State2;
	bool*        Write_State3;
	uint8_t*     Write_Data1;
} WriteMessageFrame;

typedef struct
{
	uint8_t  Response_DLC;
	ReadRegsID  Response_RegID;
	ReadReactionHandlerFuncPtr Read_ReactionHandler;
	uint8_t* Response_Data1;
	uint8_t* Response_Data2;
} ResponseMessageFrame;

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF STRUCTURES DEFINITIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF FUNCTIONS DECLARATIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

void CAN_Init(void);
void CAN_On_Receive(uint8_t *RxData);
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox);
void CAN_Respond(void);
void CAN_ProcessWriteCommand(void);
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID);
void CAN_ReportError(ErrorRegsID ErrorID);
void CANBUS_Error_Handler(void);
void Read_AIRsAmperage_Handler(void);
void Read_InsulationResistanceValue_Handler(void);
void Write_TractiveSystemState_Handler(void);
void Write_MainHandshake_Handler(void);

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF FUNCTIONS DECLARATIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

#endif /* INC_CAN_CONFIG_H_ */
