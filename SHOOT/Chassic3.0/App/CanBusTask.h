#ifndef  __CANBUSTASK_H
#define  __CANBUSTASK_H

#include "main.h"
#include "can.h"
#include "cmsis_os.h"

//can1
#define CM1_ID           0x201
#define CM2_ID           0x202 
#define CM3_ID           0x203
#define CM4_ID           0x204
#define GMLittlePitch_ID          0x205
#define GMPitch_ID      0x206

#define TM_ID           0x207




#define RATE_BUF_SIZE 6
typedef struct{
	int16_t	 	speed_rpm;          //
	int16_t  	real_current;       //
	int16_t  	Temperature;        //
	int16_t 	angle;              //
}Measure;

typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
}Encoder;


extern Measure CM1_Measure;
extern Measure CM2_Measure;
extern Measure CM3_Measure;
extern Measure CM4_Measure;
extern Measure Turntable_Measure ;

extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder TurntableEncoder;

void CanReceiveMsgProcess(CanRxMsgTypeDef * msg);			//���յ�CAN�ź� ��Ϣ����
void CanReceiveMsgProcess_2(CanRxMsgTypeDef *msg);
void Set_Gimbal_Current(CAN_HandleTypeDef *CANx,int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);
void Set_Up_Current(CAN_HandleTypeDef *CANx, int16_t gimbal_pitch_iq);
void Set_CM_Speed(CAN_HandleTypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);


//9015
void DM_EncoderProcess(Encoder *v);

#endif

