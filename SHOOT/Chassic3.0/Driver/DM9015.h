#ifndef __DM9015_H_
#define __DM9015_H_
#include "stdint.h"
#include "usart.h"



#define RATE_BUF_SIZE 6

typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
}DM_Encoder;

void DMfold_Res_Task(void);
void DM_SendPower(int16_t power);
void DM_SendAngularSpeed(int32_t angularspeed);
void DM_SendAngle(int64_t angle);
void DM_SendPosition(int64_t angle,int32_t angularspeed);

#endif