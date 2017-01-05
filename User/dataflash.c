#include "stm32f10x_flash.h"
#include "upstandingcar.h"
#include "dataflash.h"

static char FlashWBuffer[128]; 

#define FLASH_START_ADDR	(0x08000000+1024*60)//

#define original_PID_Addr	0x0
#define Angle_PID_Addr	0x32
#define Speed_PID_Addr	0x48


/*
	操作顺序:
	- 每次写操作之前将整叶数据读取到缓冲区
	- 将要存储的数据按指定地址写到缓冲区
	- 叶擦除
	- 将整个缓冲区数据写到flash

	缓冲区大小为128字节，因此实际有效存储地址范围0x00-0x7F
	如果要增加存储容量，可通过增大缓冲区实现
*/
void ProgramFlash(uint32_t addr, char* data, uint16_t len)
{
	uint8_t i=0;
	
	FLASH_Unlock();  //
     	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//

	for(i=0; i<128; i ++){
		FlashWBuffer[i] = *((char*)(FLASH_START_ADDR+i));
	}

	for(i=0; i< len; i++){
		FlashWBuffer[addr+i] = *data++;
	}

	FLASH_ErasePage(FLASH_START_ADDR); 	//
	for(i=0; i<128; i+=4)
		FLASH_ProgramWord(FLASH_START_ADDR+i, *((uint32_t*)&FlashWBuffer[i])); //

	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//
	FLASH_Lock();    //
}

void ReadFlash(uint32_t addr, char* data, uint16_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)
	 	*data++ = *((char*)(FLASH_START_ADDR+addr+i));
}


void PIDInit()
{
	char flag[2];
	
	ReadFlash(original_PID_Addr, flag, 2);
	if((flag[0]==0xa5)&&(flag[1]==0x5a))
	{// 非初次运行
		PIDRead();
	}
	else{// 初次运行
		flag[0]=0xa5,flag[1]=0x5a;
		ProgramFlash(original_PID_Addr, flag, 2);
		ProgramFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));// 保存角度环默认pid参数
		ProgramFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));// 保存速度环默认PID参数
	}
	
}

/*
	保存修改的PID参数
*/
void PIDWrite(char flag)
{
	char i=1;
	if(flag==0){
		ProgramFlash(Angle_PID_Addr, &i, 1);	
		ProgramFlash(Angle_PID_Addr+1,(char*)&g_tCarAnglePID, sizeof(PID_t));
	}
	else{
		ProgramFlash(Speed_PID_Addr, &i ,1);	
		ProgramFlash(Speed_PID_Addr+1,(char*)&g_tCarSpeedPID, sizeof(PID_t));
	}
		
}

/*
	读取PID参数，参数没有更新则读取默认值
*/
void PIDRead()
{
	char i;
	
	ReadFlash(Angle_PID_Addr, &i, 1);
	if(i==1)      
		ReadFlash(Angle_PID_Addr+1, (char*)&g_tCarAnglePID, sizeof(PID_t));
	else
		ReadFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));
	
	ReadFlash(Speed_PID_Addr, &i, 1);
	if(i==1)      
		ReadFlash(Speed_PID_Addr+1, (char*)&g_tCarSpeedPID, sizeof(PID_t));
	else
		ReadFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));
}
/*
	将PID参数恢复到默认值
*/
void PIDReset(char flag)
{
	char i=0;
	if(flag==0){
		ProgramFlash(Angle_PID_Addr,  &i, 1);
		ReadFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));
	}
	else{
		ProgramFlash(Speed_PID_Addr, &i, 1);
		ReadFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));
	}
}


