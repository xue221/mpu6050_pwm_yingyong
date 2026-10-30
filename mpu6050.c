#include "app_mpu6050.h"
#include "i2c.h"
#include "delay.h"
#include "task.h"
#include "math.h"

static float ax ,ay ,az;
static float gx ,gy ,gz;
static float yaw, pitch, roll;
static void reg_write(uint8_t reg , uint8_t value);
static uint8_t reg_read(uint8_t reg);

// 对mpu6050继续初始化

void App_MPU6050_Init(void)
{
	//  1 初始化I2C总线
	//#1. IO引脚初始化
	
	//对I2C1进行重映射
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
	
	//对PB7和PB9进行初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	// #2 .初始化I2C1模块
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	
	I2C_InitTypeDef I2C_InitStruct;
	
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_Init(I2C1,&I2C_InitStruct);
	
	I2C_Cmd(I2C1,ENABLE);
	
	// 2 设置mpu6050的参数
	reg_write(0x6b ,0x80); //复位
	Delay(100);
	
	reg_write(0x6b ,0x00); //唤醒mpu6050
	
	reg_write(0x1b ,0x18); //将陀螺仪量程设置为+-2000°/s
	
	reg_write(0x1c ,0x00); //将加速度设置为+-2g
}


 //通过陀螺仪计算欧拉角
void App_MPU6050_Proc(void)
{
	PERIODIC(5)
	
	App_MPU6050_Updata();
	
	float yaw_g = yaw + gz * 0.005;
	float pitch_g = pitch + pitch * 0.005;
	float roll_g = roll - gy * 0.005;
	
	float pitch_a = atan2(ay , az) / 3.1415927 *180.0f;
	float roll_a = atan2(ax , az) / 3.1415927 *180.0f;
	
	yaw = yaw_g;
	pitch = 0.95238 *pitch_g + (1-0.95238) * pitch_a;
	roll = 0.95238 *roll_g + (1-0.95238) * roll_a;
}


 // 更新mpu6050的值
void App_MPU6050_Updata(void)
{
	int16_t ax_raw =(int16_t)((reg_read(0x3b)<< 8) + reg_read(0x3c)); //ax的原始数据
	int16_t ay_raw =(int16_t)((reg_read(0x3d)<< 8) + reg_read(0x3e)); //ay的原始数据
	int16_t az_raw =(int16_t)((reg_read(0x3f)<< 8) + reg_read(0x40)); //az的原始数据
	
	ax = ax_raw * 6.1035e-5f;
	ay = ay_raw * 6.1035e-5f;
	az = az_raw * 6.1035e-5f;
	
	int16_t gx_raw =(int16_t)((reg_read(0x43)<< 8) + reg_read(0x44)); //gx的原始数据
	int16_t gy_raw =(int16_t)((reg_read(0x45)<< 8) + reg_read(0x46)); //gy的原始数据
	int16_t gz_raw =(int16_t)((reg_read(0x47)<< 8) + reg_read(0x48)); //gz的原始数据
	
	gx = gx_raw * 6.1035e-2f;
	gy = gy_raw * 6.1035e-2f;
	gz = gz_raw * 6.1035e-2f;
}

 // 向寄存器写值
 // reg  要写入寄存器的地址
 // value  要写入的值
static void reg_write(uint8_t reg , uint8_t value)
{
	uint8_t bytesToSend[] = { reg , value};
	
	My_I2C_SendBytes(I2C1 , 0xd0 ,bytesToSend, 2);
}

 // 读取寄存器的值
 // reg  要读取寄存器的地址
 // 返回值 ：表示读取到的值


static uint8_t reg_read(uint8_t reg)
{
	My_I2C_SendBytes(I2C1 , 0xd0 ,&reg ,1);
	
	uint8_t regValue;
	
	My_I2C_ReceiveBytes(I2C1, 0xd0 ,&regValue, 1);
	
	return regValue;
}

float App_MPU6050_GetAx(void)
{
	return ax;
}
float App_MPU6050_GetAy(void)
{
	return ay;
}
float App_MPU6050_GetAz(void)
{
	return az;
}

float App_MPU6050_GetGx(void)
{
	return gx;
}
float App_MPU6050_GetGy(void)
{
	return gy;
}
float App_MPU6050_GetGz(void)
{
	return gz;
}

float App_MPU6050_GetYaw(void)
{
	return yaw;
}
float App_MPU6050_GetPitch(void)
{
	return pitch;
}
float App_MPU6050_GetRoll(void)
{
	return roll;
}
