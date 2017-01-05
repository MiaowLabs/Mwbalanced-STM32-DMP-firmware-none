/******************** (C) COPYRIGHT 2014 MiaoW Labs ***************************
** 文件名称：led.c
** 功能描述: led 应用函数库            
** 实验平台：两轮自平衡小车BasicBalance主控板
** 硬件连接：----------------- 
**	   		|   PB3 - LED1     |
**			|   PB4 - LED2     |
**			 ----------------- 
** 库版本  ：ST3.5.0 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
**********************************************************************************/
#include "led.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*先开启GPIOA 和 AFIO 的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);

	/*改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能*/
    	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 

	/*选择要控制的GPIOB引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	
	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOA, &GPIO_InitStructure);			  

	/* 打开所有led灯	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12); 	 
}



/******************* (C) COPYRIGHT 2014 MiaoW Labs *****END OF FILE************/
