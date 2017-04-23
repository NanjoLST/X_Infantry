#include "drivers_imu_low.h"

#include "utilities_debug.h"
#include "utilities_tim.h"

#include "cmsis_os.h"

#include "drivers_imu_mpu6500_reg.h"
#include "drivers_imu_IST8310_reg.h"

#include "spi.h"

#include "tim.h"
#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)


uint8_t MPU_id = 0;
uint8_t datasc = 1;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};

IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
	datasc = data;
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

//Initialize the IST8310
uint8_t IST8310_Init(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

//Get 6 axis data from MPU6500
void IMU_Get_Data()
{
  uint8_t mpu_buff[22];
//	uint8_t ist_buff[6];
//	uint8_t a,b;
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 22);
  
  imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
  imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
  imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
  
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
  imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
  imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
  imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
  
	imu_data.mx=mpu_buff[15]<<8 |mpu_buff[14];
	imu_data.my=mpu_buff[17]<<8 |mpu_buff[16];
  imu_data.mz=mpu_buff[19]<<8 |mpu_buff[18];
	
}


//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }

  return 0;
}
/*****
// Quaternion
*****/
//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

#include "math.h"
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float gx, gy, gz, ax, ay, az, mx, my, mz;
float gYroX, gYroY, gYroZ;
float angles[3];
int16_t maxx = 203, maxy = -45, maxz = 421;
int16_t minx = -90, miny = -321, minz = 134;
void updateQuaternion()
{
			float mygetqval[9];
			mygetqval[0] = imu_data.ax;
			mygetqval[1] = imu_data.ay;
			mygetqval[2] = imu_data.az;
			
			mygetqval[3] = imu_data.gx / 32.8f;
			mygetqval[4] = imu_data.gy / 32.8f;
			mygetqval[5] = imu_data.gz / 32.8f;
			
			mygetqval[6] = (imu_data.mx - (maxx + minx) / 2.0) / (maxx - minx) * 2;
			mygetqval[7] = (imu_data.my - (maxy + miny) / 2.0) / (maxy - miny) * 2;
			mygetqval[8] = (imu_data.mz - (maxz + minz) / 2.0) / (maxz - minz) * 2;
			
			gYroX = mygetqval[3];
			gYroY = mygetqval[4];
			gYroZ = mygetqval[5];
			
#define Kp 2.0f
#define Ki 0.01f 
#define M_PI  (float)3.1415926535
			static uint64_t lastUpdate, now;
			static float exInt, eyInt, ezInt;

			float norm;
			float hx, hy, hz, bx, bz;
			float vx, vy, vz, wx, wy, wz;
			float ex, ey, ez, halfT;
			float tempq0,tempq1,tempq2,tempq3;

			float q0q0 = q0*q0;
			float q0q1 = q0*q1;
			float q0q2 = q0*q2;
			float q0q3 = q0*q3;
			float q1q1 = q1*q1;
			float q1q2 = q1*q2;
			float q1q3 = q1*q3;
			float q2q2 = q2*q2;   
			float q2q3 = q2*q3;
			float q3q3 = q3*q3;   
			
			//halfT=2.5/1000;
			now = fw_getTimeMicros();  //读取时间 单位是us   
			halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//			if((now-lastUpdate)<100)
//			{
//				//halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//				return;
//			}
//			else	
//			{
//					halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//			}
			lastUpdate = now;	//更新时间
			
			gx = mygetqval[3] * M_PI/180;
			gy = mygetqval[4] * M_PI/180;
			gz = mygetqval[5] * M_PI/180;
			ax = mygetqval[0];
			ay = mygetqval[1];
			az = mygetqval[2];
			mx = mygetqval[6];
			my = mygetqval[7];
			mz = mygetqval[8];

			//快速求平方根算法
			norm = invSqrt(ax*ax + ay*ay + az*az);       
			ax = ax * norm;
			ay = ay * norm;
			az = az * norm;
			//把加计的三维向量转成单位向量。
			norm = invSqrt(mx*mx + my*my + mz*mz);          
			mx = mx * norm;
			my = my * norm;
			mz = mz * norm; 
			// compute reference direction of flux
			hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
			hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
			hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
			bx = sqrt((hx*hx) + (hy*hy));
			bz = hz; 
			// estimated direction of gravity and flux (v and w)
			vx = 2.0f*(q1q3 - q0q2);
			vy = 2.0f*(q0q1 + q2q3);
			vz = q0q0 - q1q1 - q2q2 + q3q3;
			wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
			wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
			wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay*vz - az*vy) + (my*wz - mz*wy);
			ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
			ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

			if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
			{
					exInt = exInt + ex * Ki * halfT;
					eyInt = eyInt + ey * Ki * halfT;	
					ezInt = ezInt + ez * Ki * halfT;
					// 用叉积误差来做PI修正陀螺零偏
					gx = gx + Kp*ex + exInt;
					gy = gy + Kp*ey + eyInt;
					gz = gz + Kp*ez + ezInt;
			}
			// 四元数微分方程
			tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
			tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
			tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
			tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

			// 四元数规范化
			norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
			q0 = tempq0 * norm;
			q1 = tempq1 * norm;
			q2 = tempq2 * norm;
			q3 = tempq3 * norm;
			
			float q[4];
			q[0] = q0; //返回当前值
			q[1] = q1;
			q[2] = q2;
			q[3] = q3;
			//float angles[3];
			angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
			angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
			angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  

			static int countPrint = 0;
			if(countPrint > 50)
			{
				countPrint = 0;
				
//				fw_printf("mx max = %d | min = %d\r\n", mymaxmx, myminmx);
//				fw_printf("my max = %d | min = %d\r\n", mymaxmy, myminmy);
//				fw_printf("mz max = %d | min = %d\r\n", mymaxmz, myminmz);
//				fw_printf("========================\r\n");
				
//				fw_printf("now = %d \r\n", now);
//				fw_printf("xxx = %d \r\n", 2147483647);
//				fw_printf("halfT = %f \r\n", halfT);

//				fw_printf("angles0 = %f | ", angles[0]);
//				fw_printf("angles1 = %f | ", angles[1]);
//				fw_printf("angles2 = %f\r\n", angles[2]);
//				fw_printf("========================\r\n");
//				
//				fw_printf("mx = %d | ",imu_data.mx);
//				fw_printf("my = %d | ",imu_data.my);
//				fw_printf("mz = %d\r\n",imu_data.mz);
//				fw_printf("========================\r\n");
				
//				fw_printf("mx = %f | ",mx);
//				fw_printf("my = %f | ",my);
//				fw_printf("mz = %f\r\n",mz);
//				fw_printf("========================\r\n");
			}
			else
			{
				countPrint++;
			}
}

void printIMUTask(void const * argument){
//	int16_t tmaxx, tmaxy, tmaxz;
//	int16_t tminx, tminy, tminz;
//	IMU_Get_Data();
//	tmaxx = tminx = imu_data.mx;
//	tmaxy = tminy = imu_data.my;
//	tmaxz = tminz = imu_data.mz;
	while(1){
		IMU_Get_Data();
		updateQuaternion();
//		fw_printfln("axyz %d %d %d", imu_data.ax, imu_data.ay, imu_data.az);
//		fw_printfln("gxyz %d %d %d", imu_data.gx, imu_data.gy, imu_data.gz);
//		fw_printfln("mxyz %d %d %d", imu_data.mx, imu_data.my, imu_data.mz);
		fw_printfln("xyz %f %f %f", angles[0], angles[1], angles[2]);
//		fw_printfln("mxyz %f %f %f", 
//			(imu_data.mx - (maxx + minx) / 2.0) / (maxx - minx) * 2,
//			(imu_data.my - (maxy + miny) / 2.0) / (maxy - miny) * 2,
//			(imu_data.mz - (maxz + minz) / 2.0) / (maxz - minz) * 2
//		);
//		if(imu_data.mx > tmaxx)tmaxx = imu_data.mx;
//		if(imu_data.mx < tminx)tminx = imu_data.mx;
//		if(imu_data.my > tmaxy)tmaxy = imu_data.my;
//		if(imu_data.my < tminy)tminy = imu_data.my;
//		if(imu_data.mz > tmaxz)tmaxz = imu_data.mz;
//		if(imu_data.mz < tminz)tminz = imu_data.mz;
//		fw_printfln("max %d %d %d", tmaxx, tmaxy, tmaxz);
//		fw_printfln("min %d %d %d", tminx, tminy, tminz);
		
		//fw_printfln("time %f", fw_getTimeMicros() / 1000000.0);
		fw_printfln("=============");
		osDelay(500);
	}
}
