/******************** (C) COPYRIGHT 2016 MiaowLabs **************************
 * 文件名  ：main.c
 * 描述    ：喵呜实验室平衡小车Mwbalanced STM32       
 * 实验平台：Mwbalanced STM32
 * 库版本  ：ST3.5.0
 *
 * 作者    ：MiaowLabs 
 * 论坛    ：https://bbs.miaowlabs.com/
 * 淘宝    ：https://miaowlabs.taobao.com/
**********************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "usart.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "i2c.h"
#include "motor.h"
#include "SysTick.h"
#include "upstandingcar.h"
#include "outputdata.h"
#include "ADC.h"
#include "communicate.h"
#include "dataflash.h"
#include "common.h"

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */


unsigned short encoder_num1,encoder_num2;

uint16_t RunTime=0;
uint16_t BatVol;

//秒级任务
void SecTask()
{
	static char LED=0;
	if(SoftTimer[0])return;
	else{
		SoftTimer[0] = 1000;
	}
	RunTime++; // 记录运行时间
	BatVol = GetBatVol(); // 读取电池电压

	if(StatusFlag)ResponseStatus();
	
	LED = ~LED;
	if(LED)LED1(ON);
	else LED1(OFF);
}

int main(void)
{	
	LED_GPIO_Config();	//LED配置
	USART1_Config();	//UART1配置-->MicroUSB用
	USART3_Config(0);	//UART3配置-->与蓝牙模块通信
	delay_nms(20);
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+BAUD8\r\n"); //发送蓝牙模块指令
	USART3_Config(1);	//更改UART3波特率为115200
	delay_nms(20);
	SetBlueToothName();	//配置蓝牙模块名称
	
	NVIC_Configuration();//中断优先级配置
	TIM2_PWM_Init(); //PWM初始化
	MOTOR_GPIO_Config();//电机方向控制GPIO初始化
	ADC_Config();//ADC初始化
	TIM3_External_Clock_CountingMode();	 //脉冲测速初始化
	TIM4_External_Clock_CountingMode();	 //脉冲测速初始化
	i2cInit();	 //初始化I2C
	delay_nms(10);
	MPU6050_Init();//初始化MPU6050
	PIDInit(); //初始化PID
	SysTick_Init();	//初始化定时器
	
	CarUpstandInit(); //初始化系统参数

	GPIO_ResetBits(GPIOB, GPIO_Pin_4);

	// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	
	while (1)
	{
		SecTask();

		if(SoftTimer[1]==0){// 每隔50ms上传一次姿态数据
			SoftTimer[1] = 50;
			ResponseIMU();
			Parse(Uart3Buffer);
		}
	
#if 0 /*调试用 预编译命令*/

	  
   OutData[0] = Roll;
   OutData[1] = gyro[0];
   OutData[2] = accel[1] ;
   //OutData[3] = g_iAccelInputVoltage_X_Axis;
   
   OutPut_Data();
#endif	  

#if 0
		encoder_num1=TIM_GetCounter(TIM3);
		TIM3->CNT = 0;
		encoder_num2=TIM_GetCounter(TIM4);
		TIM4->CNT = 0;
		//printf("\r\n this is a printf demo \r\n");
	    //printf("\r\n 欢迎使用两轮自平衡小车BasicBalance主控板:) \r\n");		
		printf("\r\n---------编码器1---------%d \r\n",encoder_num1);
		printf("\r\n---------编码器2---------%d \r\n",encoder_num2);
#endif	  
		/*
		printf("\r\n---------加速度X轴原始数据---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------加速度Y轴原始数据---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------加速度Z轴原始数据---------%d \r\n",GetData(ACCEL_ZOUT_H));	
		printf("\r\n---------陀螺仪X轴原始数据---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------陀螺仪Y轴原始数据---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------陀螺仪Z轴原始数据---------%d \r\n",GetData(GYRO_ZOUT_H));
		delay_ms(10);     									   */
	}
}


/******************* (C) COPYRIGHT 2016 MiaowLabs Team *****END OF FILE************/


