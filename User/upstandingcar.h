#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
} ;
typedef struct _PID_t{
	float P;
	float I;
	float D;
}PID_t;

#define    CAR_ANGLE_SET 0
#define    CAR_ANGULARSPEED_SET 0
#define    GRAVITY_OFFSET (1500)       //加速度零点偏移值 
#define    GYRO_OFFSET      0          //陀螺仪零点偏移值
#define    GRAVITY_X_MIN       (-16384)
#define    GRAVITY_X_MAX         16384     
#define    CAR_ANGLE_RANGE	  	 90
#define    GRAVITY_ANGLE_RATIO	 ((float)CAR_ANGLE_RANGE * 2.0 /((float)GRAVITY_X_MAX - (float)GRAVITY_X_MIN))

#define    CAR_ZERO_ANGLE   (3)
#define    GYROSCOPE_ANGLE_RATIO     0.120 //0.7	 0.080
#define    GYROSCOPE_ANGLE_SIGMA_FREQUENCY	100	 //1/0.005=200
#define    GRAVITY_ADJUST_TIME_CONSTANT 4 //1
/******速度控制相关宏定义******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         g_iCarSpeedSet
#define MOTOR_LEFT_SPEED_POSITIVE  (g_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (g_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  334	//光电码盘刻度槽
#define SPEED_CONTROL_PERIOD	 50	    //速度环控制周期
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)
#define CAR_POSITION_MAX	MOTOR_OUT_MAX
#define CAR_POSITION_MIN	MOTOR_OUT_MIN
/******电机控制相关宏定义******/
#define MOTOR_OUT_DEAD_VAL       0	   //死区值8
#define MOTOR_OUT_MAX           999	   //占空比正最大值
#define MOTOR_OUT_MIN         (-999)   //占空比负最大值

#define	MOTOR_LEFT_AIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))
#define	MOTOR_LEFT_AIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_14))
#define	MOTOR_LEFT_AIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_15))
#define	MOTOR_LEFT_AIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_15))

#define	MOTOR_RIGHT_BIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))
#define	MOTOR_RIGHT_BIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_12))
#define	MOTOR_RIGHT_BIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_13))
#define	MOTOR_RIGHT_BIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_13))

extern float g_fCarAngle;

extern PID_t g_tCarAnglePID;
extern PID_t g_tCarSpeedPID;

extern float g_fBluetoothSpeed;
extern float g_fBluetoothDirection;
extern u8 g_u8MainEventCount;
extern u8 g_u8SpeedControlCount;
extern u8 g_u8SpeedControlPeriod;
extern u8 g_u8DirectionControlPeriod;
extern u8 g_u8DirectionControlCount;
extern u8 g_u8LEDCount; 

void delay_nms(u16 time);
void CarUpstandInit(void);
//void SampleInputVoltage(void);
void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void BluetoothControl(void)	;
void GetMotorPulse(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void DirectionControl(void); 
void steer(enum Command command);
float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);

#endif
