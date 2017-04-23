#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"

#include "cmsis_os.h"

#include "framework_peripheraldefine.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"

//RxIOPool
NaiveIOPoolDefine(CMFLRxIOPool, {0});
NaiveIOPoolDefine(CMFRRxIOPool, {0});
NaiveIOPoolDefine(CMBLRxIOPool, {0});
NaiveIOPoolDefine(CMBRRxIOPool, {0});

NaiveIOPoolDefine(GMPITCHRxIOPool, {0});
NaiveIOPoolDefine(GMYAWRxIOPool, {0});

NaiveIOPoolDefine(AMUDFLRxIOPool, {0});
NaiveIOPoolDefine(AMUDFRRxIOPool, {0});
NaiveIOPoolDefine(AMUDBLRxIOPool, {0});
NaiveIOPoolDefine(AMUDBRRxIOPool, {0});
NaiveIOPoolDefine(AMPLATERxIOPool, {0});
NaiveIOPoolDefine(AMGETBULLETRxIOPool, {0});
//TxIOPool
#define DataPoolInit \
	{ \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(CMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(GMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(AM1TxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(AM2TxIOPool, DataPoolInit);
#undef DataPoolInit 

//NaiveIOPoolDefine(motorCanRxIOPool, {0});

//#define DataPoolInit \
//	{ \
//		{MOTORCM_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{MOTORGIMBAL_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
//	}
//NaiveIOPoolDefine(motorCanTxIOPool, DataPoolInit);
//#undef DataPoolInit 

///*****Begin define ioPool*****/
//#define DataPoolInit \
//	{ \
//		{MOTORCM_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{MOTORCM_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{MOTORGIMBAL_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{MOTORGIMBAL_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
//		{0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
//	}
//#define ReadPoolSize 2
//#define ReadPoolMap {MOTORCM_ID, MOTORGIMBAL_ID}
//#define GetIdFunc (data.StdId)
//#define ReadPoolInit {{0, Empty, 1}, {2, Empty, 3}}

//IOPoolDefine(motorCanTxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

//#undef DataPoolInit 
//#undef ReadPoolSize 
//#undef ReadPoolMap
//#undef GetIdFunc
//#undef ReadPoolInit
///*****End define ioPool*****/


uint8_t isRcanStarted = 0;

CanRxMsgTypeDef canRxMsg;
void motorInit(){
	//===wait for 820R
	
	
	CMGMMOTOR_CAN.pRxMsg = &canRxMsg;
	
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMGMMOTOR_CAN, &sFilterConfig);
	
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted = 1;
}

uint16_t pitchAngle = 0, yawAngle = 0;
uint32_t flAngle = 0, frAngle = 0, blAngle = 0, brAngle = 0;
uint16_t flSpeed = 0, frSpeed = 0, blSpeed = 0, brSpeed = 0;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	//CanRxMsgTypeDef *temp = IOPool_pGetWriteData(motorCanRxIOPool);
	if(hcan == &CMGMMOTOR_CAN){
		switch(canRxMsg.StdId){
			case CMFL_RXID:
				for(int i = 0; i != 8; ++i){
					IOPool_pGetWriteData(CMFLRxIOPool)->Data[i] = canRxMsg.Data[i];
				}
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				break;
			case CMBL_RXID:
				break;
			case CMBR_RXID:
				break;
			case GMYAW_RXID:
				break;
			case GMPITCH_RXID:
				break;
			default:
			fw_Error_Handler();
		}
	}else if(hcan == &AUXMOTOR_CAN){
		switch(canRxMsg.StdId){
			case AMUDFL_RXID:
				break;
			case AMUDFR_RXID:
				break;
			case AMUDBL_RXID:
				break;
			case AMUDBR_RXID:
				break;
			case AMPLATE_RXID:
				break;
			case AMGETBULLET_RXID:
				break;
			default:
			fw_Error_Handler();
		}
	}
	if(canRxMsg.StdId == CMFL_RXID){
		flAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
		flSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
	}else if(canRxMsg.StdId == CMFR_RXID){
		frAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
		frSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
	}else if(canRxMsg.StdId == CMBL_RXID){
		blAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
		blSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
	}else if(canRxMsg.StdId == CMBR_RXID){
		brAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
		brSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
	}else if(canRxMsg.StdId == GMPITCH_RXID){
		pitchAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
	}else if(canRxMsg.StdId == GMPITCH_RXID){
		yawAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
	}
	
//	
//	IOPool_getNextWrite(motorCanRxIOPool);
//	MOTOR_CAN.pRxMsg = IOPool_pGetWriteData(motorCanRxIOPool);
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		//fw_Warning();
		isRcanStarted = 0;
	}else{
		isRcanStarted = 1;
	}
	if(isInited == 1){
		osSemaphoreRelease(canrefreshGimbalSemaphoreHandle);
		//osSemaphoreRelease(motorCanReceiveSemaphoreHandle);
	}
}

int counttestsemwait = 0;
extern int counttestsemreleaseOk;
extern int counttestsemreleaseError;
void motorCanTransmitTask(void const * argument){
	//osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
	while(1){
//		fw_printfln("w%d rO%d rE%d", counttestsemwait, counttestsemreleaseOk, counttestsemreleaseError);
		osSemaphoreWait(motorCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
		counttestsemwait++;
		if(IOPool_hasNextRead(CMTxIOPool, 0)){
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(CMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
			}
		}
		if(IOPool_hasNextRead(GMTxIOPool, 0)){
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(GMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			HAL_StatusTypeDef test = HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN);
			if(test != HAL_OK){
				fw_printfln("t%d", test);
//				fw_printfln("h%d", motorCan.State);
//				fw_Warning();
				osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
			}
		}
		if(isRcanStarted == 0){
			if(CMGMMOTOR_CAN.State == HAL_CAN_STATE_BUSY_RX){
				CMGMMOTOR_CAN.State = HAL_CAN_STATE_READY;
			}
			if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
				fw_Warning();
				fw_printf("canstate=%x\r\n", CMGMMOTOR_CAN.State);
			}else{
				//fw_Warning();
				isRcanStarted = 1;
			}
		}
		//osDelay(1);
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
}
