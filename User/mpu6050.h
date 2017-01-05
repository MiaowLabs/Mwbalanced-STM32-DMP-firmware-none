#ifndef __MPU6050_H__
#define __MPU6050_H__

#define q30  1073741824.0f

extern float Pitch,Roll,Yaw;
extern short gyro[3], accel[3];

void MPU6050_Init(void);
void MPU6050_Pose(void);

#endif

