/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：秋雨魂,茶不思
 * 文件名  ：AnoCopter.cpp
 * 描述    ：匿名挑战者V3微型六轴飞行器
 * 代码版本：V1.0
 * 时间		 ：2015/9/6
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "ANO_Config.h"

int main(void)
{
	//初始化飞控板的硬件设置
	ANO_Hexacopter_board_Init();
	
	//初始化参数
	param.Init();
	
	//初始化IMU（惯性测量单元）
	imu.Init();	
	
	nav.Init();
	
	while(1)
	{
		ANO_Loop();
	}
	
	return 0;
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
