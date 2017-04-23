#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"

#include "utilities_debug.h"
//#include "tasks_upper.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "application_pidfunc.h"
#include "application_motorcontrol.h"

#include "stdint.h"

//uint16_t yawAngle = 0, pitchAngle = 0;
//void printMotorTask(void const * argument){
//	while(1){
//		//printMotorTasktaskcount++;
////		osSemaphoreWait(motorCanReceiveSemaphoreHandle, osWaitForever);
////		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
////			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
////			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
////			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
////		}
////		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
////			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
////			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
////			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
////		}
////		osSemaphoreRelease(refreshGimbalSemaphoreHandle);
//	}
//}

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
PID_Regulator_t pitchPositionPID = PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t yawPositionPID = PID_INIT(30.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t pitchSpeedPID = PID_INIT(50.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
PID_Regulator_t yawSpeedPID = PID_INIT(70.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);

extern float gYroX, gYroY, gYroZ;
//extern int forPidDebug;

int counttestsemreleaseOk = 0;
int counttestsemreleaseError = 0;

//extern uint16_t testcanrxiopool;
extern uint16_t pitchAngle, yawAngle;
extern uint32_t flAngle, frAngle, blAngle, brAngle;
extern uint16_t flSpeed, frSpeed, blSpeed, brSpeed;
uint8_t flUpDown = 0, frUpDown = 0, blUpDown = 0, brUpDown = 0, allUpDown = 0;
void controlMotorTask(void const * argument){
	float yawAngleTarget = 0.0, pitchAngleTarget = 8.0;
	int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;//1075,710 | 750,6821
	//uint16_t yawAngle = 0, pitchAngle = 0;
	float yawRealAngle = 0.0, pitchRealAngle = 0.0;
	//uint16_t lastYawAngle = 0, lastPitchAngle = 0;
	
	int16_t pitchIntensity = 0, yawIntensity = 0;
	
	//uint8_t isLastAngleError = 0;
	//uint8_t pitchReady = 0, yawReady = 0;
	
	
	int16_t flIntensity = 0, frIntensity = 0, blIntensity = 0, brIntensity = 0;
	while(1){
		//controlMotorTaskTasktaskcount++;
		
		//osSemaphoreWait(imurefreshGimbalSemaphoreHandle, osWaitForever);
		osSemaphoreWait(canrefreshGimbalSemaphoreHandle, osWaitForever);
		
//		if(IOPool_hasNextRead(upperGimbalIOPool, 0)){
//			IOPool_getNextRead(upperGimbalIOPool, 0);
//			yawAngleTarget += IOPool_pGetReadData(upperGimbalIOPool, 0)->yawAdd;
//			pitchAngleTarget += IOPool_pGetReadData(upperGimbalIOPool, 0)->pitchAdd;
//		}
		
		
		
		if(IOPool_hasNextRead(GMYAWRxIOPool, 0)){
			IOPool_getNextRead(GMYAWRxIOPool, 0);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(GMYAWRxIOPool, 0);
			//yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
			
			yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8192.0;
			yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
			yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
			MINMAX(yawAngleTarget, -45, 45);
			//position		
			yawPositionPID.target = yawAngleTarget;
			yawPositionPID.feedback = yawRealAngle;
			yawPositionPID.Calc(&yawPositionPID);
			//speed
			yawSpeedPID.target = yawPositionPID.output;
			yawSpeedPID.feedback = -gYroZ;
			yawSpeedPID.Calc(&yawSpeedPID);
			yawIntensity = (int16_t)yawSpeedPID.output;
			
			//yawReady = 1;
		}
		if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(GMPITCHRxIOPool, 0);
			//pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
			
			pitchRealAngle = -(pitchAngle - pitchZeroAngle) * 360 / 8192.0;
			pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
			pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
			MINMAX(pitchAngleTarget, -25, 25);
			//position		
			pitchPositionPID.target = pitchAngleTarget;
			pitchPositionPID.feedback = pitchRealAngle;
			pitchPositionPID.Calc(&pitchPositionPID);
			//speed
			pitchSpeedPID.target = pitchPositionPID.output;
			pitchSpeedPID.feedback = -gYroY;
			pitchSpeedPID.Calc(&pitchSpeedPID);
			pitchIntensity = -(int16_t)pitchSpeedPID.output;
			
			//pitchReady = 1;
		}
		
//		if(isLastAngleError >= 0){
//			isLastAngleError = 0;
//		}else if(/*isLastYawAngleInited && isLastPitchAngleInited 
//			&&(*/lastYawAngle - yawAngle > 100 || lastYawAngle - yawAngle < -100
//			|| lastPitchAngle - pitchAngle > 100 || lastPitchAngle - pitchAngle < -100/*)*/
//		){
//			isLastAngleError++;
//			continue;
//		}
//		lastYawAngle = yawAngle;
//		lastPitchAngle = pitchAngle;
//		isLastYawAngleInited = isLastPitchAngleInited = 1;
		
		static int countwhile = 0;
//		static int yIoutCount = 0, pIoutCount = 0, totalCount = 0;
//		totalCount++;
//		if(yawIntensity == 4900 | yawIntensity == -4900){
//			yIoutCount++;
//		}
//		if(pitchIntensity == 4900 | pitchIntensity == -4900){
//			pIoutCount++;
//		}
		if(countwhile >= 500){
			countwhile = 0;
			
			//if(forPidDebug == 1){
//				fw_printf("yIoc = %d\t", yIoutCount);
//				fw_printf("pIoc = %d\t", pIoutCount);
//				fw_printf("count = %d\t", totalCount);
//				yIoutCount = 0, pIoutCount = 0, totalCount = 0;
				
//				fw_printf("ya = %d\t", yawAngle);
//				fw_printf("yra = %f\t", yawRealAngle);
//				fw_printf("yI = %5d\t", yawIntensity);
//				fw_printf("gYroZ = %f\t", gYroZ);
//				
//				fw_printf("pa = %d\t", pitchAngle);
//				fw_printf("pra = %f\t", pitchRealAngle);
//				fw_printf("pitI = %5d\t", pitchIntensity);
//				fw_printf("gYroY = %f \r\n", gYroY);
				
				fw_printfln("flAngle %d flSpeed %d", flAngle, flSpeed);
			//}
		}else{
			countwhile++;
		}
		
		//fw_printf("%d\r\n", testcanrxiopool);
		//yawIntensity = 0;
		//pitchIntensity = -700;
		
			int16_t testIntensity = 500;
			
			setMotor(CMFL, testIntensity);

		
	}
}
