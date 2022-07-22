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

#define NUMBER_OF_READ_REGS  (5U)
#define NUMBER_OF_WRITE_REGS (2U)
#define NUMBER_OF_ERROR_REGS (12U)
#define ERROR_DLC            (2U)

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
	K_Value = 0x01u, // ADC K1 K2  current values channel 6, 7 read
	K_State = 0x02u, // K1 K2 states (on/off)
	BMS_State = 0x04u, // BMS state
	IMD_State = 0x05u, //IMD
	Ins_R_Value = 0x06u //Insulation resistance value
} ReadRegsID;

typedef enum
{
	Fuse_State = 0x03u, //fuse state k1 k2
	MAIN_State = 0x07u
} WriteRegsID;

typedef enum
{
	Over_Current_ERROR = 0x001u,
	AIR1_ERROR = 0x002u,
	AIR2_ERROR = 0x003u,
	IMD_Short_Circuit_24V_ERROR = 0x004u,
	IMD_Insulation_Measurement_ERROR = 0x005u,
	IMD_Under_Voltage_ERROR = 0x006u,
	IMD_Speed_Start_ERROR = 0x007u,
	IMD_Device_ERROR = 0x008u,
	IMD_Con_Fault_Earth_ERROR = 0x009u,
	IMD_Malfunction_ERROR = 0x00Au,
	IMD_SDC_ERROR = 0x00Bu,
	BMS_SDC_ERROR = 0x00Cu,
	TSMS_Con_ERROR = 0x00Du,
	MSD_Con_ERROR = 0x00Eu
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
	Error_ReportMessage = 0x1Du,
	Read_RequestMessage = 0x3Du
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
void (* CAN_Error_Callback)(void);

typedef struct
{
	WriteRegsID  Write_RegID;
	WriteReactionHandlerFuncPtr Write_ReactionHandler;
	bool*        Write_State;
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
void CAN_ReportError(ErrorRegsID ErrorID);
void ReadReactionHandler1(void);
void WriteReactionHandler1(void);

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF FUNCTIONS DECLARATIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

#endif /* INC_CAN_CONFIG_H_ */
