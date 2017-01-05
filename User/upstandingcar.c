
#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"
#include "communicate.h"

u8 g_u8LEDCount; 
u8 g_u8MainEventCount;
u8 g_u8SpeedControlCount;
u8 g_u8SpeedControlPeriod;
u8 g_u8DirectionControlPeriod;
u8 g_u8DirectionControlCount;
//s32 g_liAccAdd;
//s32 g_liGyroAdd;
/******电机控制参数******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;

float g_fCarAngle;

/******速度控制参数******/
//s32   g_s32LeftMotorPulse;
//s32   g_s32RightMotorPulse;
s16   g_s16LeftMotorPulse;
s16	  g_s16RightMotorPulse;

s32   g_s32LeftMotorPulseOld;
s32   g_s32RightMotorPulseOld;
s32   g_s32LeftMotorPulseSigma;
s32   g_s32RightMotorPulseSigma;

float g_fCarSpeed;
float g_iCarSpeedSet;
float g_fCarSpeedOld;
//float g_fCarSpeedOld;
float g_fCarPosition;
//float fSpeedCtrlPeriod=100.0;
/*-----角度环和速度环PID控制参数-----*/
PID_t g_tCarAnglePID={60.7, 0.0, 18.8};	//
PID_t g_tCarSpeedPID={29.97, 1.08, 0.0};	//{22.29, 2.7, 0.0}

/******蓝牙控制参数******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;


void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}

/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void CarUpstandInit(void)
{
	//g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
	g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
	g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

	g_fCarSpeed = g_fCarSpeedOld = 0;
	g_fCarPosition = 0;
	g_fCarAngle    = 0;
	//g_fGyroAngleSpeed = 0;
	//g_fGravityAngle   = 0;
	//g_fGyroscopeAngleIntegral = 0;
	g_fAngleControlOut = g_fSpeedControlOut = g_fBluetoothDirectionOut = 0;
	g_fLeftMotorOut    = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed  = g_fBluetoothDirection = 0;
	g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

  	g_u8MainEventCount=0;
	g_u8SpeedControlCount=0;
 	 g_u8SpeedControlPeriod=0;
}

/***************************************************************
** 函数名称: SampleInputVoltage
** 功能描述: 采样函数             
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/

/***************************************************************
** 函数名称: AngleControl
** 功能描述: 角度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void AngleControl(void)	 
{

	g_fCarAngle = Roll - CAR_ZERO_ANGLE;
	g_fAngleControlOut =  g_fCarAngle * g_tCarAnglePID.P+ \
	gyro[0] * (g_tCarAnglePID.D/100);

}
/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速及方向控制函数             
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	u16 u16LeftMotorValue;
	u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)
    {	
		GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      	GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      	s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      	GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    
      	GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      	//s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {	
      	GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      	GPIO_ResetBits(GPIOB, GPIO_Pin_13 );
      	s16RightVoltage = (-s16RightVoltage);
    }
    else
    {
		GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
		GPIO_ResetBits(GPIOB, GPIO_Pin_12 );	      
      	//s16RightVoltage = s16RightVoltage;
    }
	 u16RightMotorValue= (u16)s16RightVoltage;
	 u16LeftMotorValue = (u16)s16LeftVoltage;

	if(u16RightMotorValue>MOTOR_OUT_MAX)  
	{
		u16RightMotorValue=MOTOR_OUT_MAX;
	}
	if(u16LeftMotorValue>MOTOR_OUT_MAX)
	{
	   u16LeftMotorValue=MOTOR_OUT_MAX;
	}
	TIM_SetCompare3(TIM2,u16RightMotorValue);
	TIM_SetCompare4(TIM2,u16LeftMotorValue);

#if 1	 /*判断车辆是否跌倒，调试用*/

	if(g_fCarAngle > 70 || g_fCarAngle < (-70))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
	}

#endif
}

/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com 
** 日　期:   2014年08月01日
***************************************************************/
void MotorOutput(void)
{
	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection  ;	
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection ;

	/*增加死区常数*/
	if(g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if(g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if(g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if(g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理，防止超出PWM范围*/		
	if((s16)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)g_fLeftMotorOut,(s16)g_fRightMotorOut);
}

void GetMotorPulse(void)  //采集电机速度脉冲
{ 
 
  g_s16LeftMotorPulse = TIM_GetCounter(TIM3);     
  g_s16RightMotorPulse= TIM_GetCounter(TIM4);
  TIM3->CNT = 0;
  TIM4->CNT = 0;   //清零
 
  if(!MOTOR_LEFT_SPEED_POSITIVE)  
  {
  	g_s16LeftMotorPulse  = (-g_s16LeftMotorPulse) ; 
  }
  if(!MOTOR_RIGHT_SPEED_POSITIVE) 
  {
  	g_s16RightMotorPulse = (-g_s16RightMotorPulse);
  }
  #if 0
		//g_s32LeftMotorPulse=TIM_GetCounter(TIM3);		
		//g_s32RightMotorPulse=TIM_GetCounter(TIM4);		
		//printf("\r\n this is a printf demo \r\n");
	    //printf("\r\n 欢迎使用两轮自平衡小车BasicBalance主控板:) \r\n");		
		printf("\r\n---编码器1:%d \r\n",g_s16LeftMotorPulse);
		printf("\r\n---编码器2:%d \r\n",g_s16RightMotorPulse);
#endif	 
  g_s32LeftMotorPulseSigma +=  g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse; 
}
/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/

void SpeedControl(void)
{
  	float fP,fI;   	
	float fDelta;
	
	
	g_fCarSpeed = (g_s32LeftMotorPulseSigma  + g_s32RightMotorPulseSigma ) * 0.5 ;
    g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零
    	
	g_fCarSpeed = 0.5 * g_fCarSpeedOld + 0.5 * g_fCarSpeed ;
	g_fCarSpeedOld = g_fCarSpeed;

	fDelta = CAR_SPEED_SET;
	fDelta -= g_fCarSpeed;   
	
	fP = fDelta * (g_tCarSpeedPID.P);
  	fI = fDelta * (g_tCarSpeedPID.I);

	g_fCarPosition += fI;
	g_fCarPosition += g_fBluetoothSpeed;	  
//积分上限设限
	if((s16)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
	if((s16)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  	g_fSpeedControlOutNew = fP + g_fCarPosition;
//	g_fSpeedControlOut =  fP + g_fCarPosition;

}

void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;
  g_fSpeedControlOut = fValue * (g_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld; 
}

float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

void steer(enum Command command){
//	char temp[32],i;
	if(command == stop){
		g_fBluetoothSpeed = 0;
		g_fBluetoothDirection = 0;
		g_iCarSpeedSet= 0;
	}
	else if(command == joystick){
		if(sppData2>0)//前进
		g_fBluetoothSpeed = scale(sppData2, 0, 1, 0, 80);
		else if(sppData2<0)//后退
		g_fBluetoothSpeed = -scale(sppData2, 0, -1, 0, 80);
		if (sppData1 > 0) // 右转
      	g_fBluetoothDirection = scale(sppData1, 0, 1, 0, 160);
    	else if (sppData1 < 0) // 左转
      	g_fBluetoothDirection = -scale(sppData1, 0, -1, 0, 160);
						
	}else if(command == imu){
		if(sppData1>0)//前进
		g_fBluetoothSpeed = scale(sppData1, 0, 20, 0, 250);
		else if(sppData1<0)//后退
		g_fBluetoothSpeed = -scale(sppData1, 0, -20, 0, 250);
		if (sppData2 > 0) // 右转
      	g_fBluetoothDirection = scale(sppData2, 0, 40, 0, 200);
    	else if (sppData2 < 0) // 左转
      	g_fBluetoothDirection = -scale(sppData2, 0, -40, 0, 200);
		}
}





