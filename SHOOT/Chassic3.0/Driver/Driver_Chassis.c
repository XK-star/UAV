#include "Driver_Chassis.h"
#include "Remote.h"
#include "pid.h"
#include "can.h"
#include "CanBusTask.h"
#include "StatusMachine.h"
#include "Ramp.h"
#include "IOTask.h"


ChassisDataTypeDef ChassisData;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse�����ƶ�б��
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseǰ���ƶ�б��
RampGen_t MBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseè���ƶ�б�� 

static void CalculateWheelSpeed(float vx, float vy, float omega, float radian, uint16_t maxspeed);
void Key2Speed(int16_t FB, int16_t LR)
{
	int16_t tmp_FB = 0;
	int16_t tmp_LR = 0;

	tmp_FB = FB ;
	tmp_LR = LR ;


	if(Remote_CheckJumpKey(KEY_W) == 1)
	{
		ChassisData.ChassisSpeedRef.Y = tmp_FB*FBSpeedRamp.Calc(&FBSpeedRamp);
	}
	else if(Remote_CheckJumpKey(KEY_S) == 1)
	{
		ChassisData.ChassisSpeedRef.Y = -tmp_FB*FBSpeedRamp.Calc(&FBSpeedRamp);
	}
	else
	{
		FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		ChassisData.ChassisSpeedRef.Y = 0;
	}

	if(Remote_CheckJumpKey(KEY_D) == 1)
	{
		ChassisData.ChassisSpeedRef.X = tmp_LR*LRSpeedRamp.Calc(&LRSpeedRamp);
	}
	else if(Remote_CheckJumpKey(KEY_A) == 1)
	{
		ChassisData.ChassisSpeedRef.X = -tmp_LR*LRSpeedRamp.Calc(&LRSpeedRamp);
	}
	else
	{
		LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		ChassisData.ChassisSpeedRef.X = 0;
	}
}

/**
  * @brief  �����˶�����
  * @param  None
  * @retval None
  */
static void UnderpanMoveControl(void)
{	
	switch(GetChassis_MoveMode())
	{	
		case Chassis_Stop:
		{
			ChassisData.ChassisSpeedRef.Y = 0;
			ChassisData.ChassisSpeedRef.X = 0;                                                       
			ChassisData.ChassisSpeedRef.Omega  = 0;
			ChassisData.ChassisAngle = 0;//��ֵ ǰ��������ʱ��ת
		}break;
		
		case Chassis_RC_Mode:                                    
		{
			
				ChassisData.ChassisSpeedRef.Y		= RC_CtrlData.rc.ch1;
				ChassisData.ChassisSpeedRef.X   	= RC_CtrlData.rc.ch0; 
				ChassisData.ChassisSpeedRef.Omega	= PID_Task(&CMRotatePID, 0, GMYawEncoder.ecd_angle); 	
				ChassisData.ChassisAngle 			= 0;
	
		}break; 
		
		case Chassis_Mouse_Mode:
		{ 
			switch(Get_AttackMode())
			{
			  case Attack_Normal:
				{
					if(Remote_CheckJumpKey(KEY_SHIFT) == 1)//Shift ����ģʽ
					{
						Key2Speed(HIGH_FORWARD_BACK_SPEED, HIGH_LEFT_RIGHT_SPEED);
					}
					else if(Remote_CheckJumpKey(KEY_CTRL) == 1)//Ctrl����ȡ�� ����ģʽ
					{
						Key2Speed(LOW_FORWARD_BACK_SPEED, LOW_LEFT_RIGHT_SPEED);
					}				
					else
					{
						Key2Speed(NORMAL_FORWARD_BACK_SPEED, NORMAL_LEFT_RIGHT_SPEED);
					}
				}
				case Attack_Dragon:
				{
					
				}
				case Attack_CatWalk:
				{
					
				}
				case Attack_Slow:
				{
					
				}
				case Attack_45Degree:
				{
					
				}
				case Attack_None:
				{
					
				}
			}
		}	
		default:
			break;
	}
	  ChassisData.ChassisSpeedRef.X = RC_CtrlData.rc.ch0;
		ChassisData.ChassisSpeedRef.Y = RC_CtrlData.rc.ch1;
		ChassisData.ChassisSpeedRef.Omega = RC_CtrlData.rc.ch2;
		ChassisData.ChassisAngle =  0;
	
CalculateWheelSpeed(ChassisData.ChassisSpeedRef.X, \
						ChassisData.ChassisSpeedRef.Y, \
						ChassisData.ChassisSpeedRef.Omega, \
						Ang2Rad(ChassisData.ChassisAngle), \
						660);
}
/**
  * @brief  ����3���ٶ�ֵ�������ӵ��ٶ�
  * @param  float vx X�����ϵ��ٶ�
  * @param  float vy Y�����ϵ��ٶ�
  * @param  float omega ��ת���ٶ�
  * @param  float radian ��ʱ���������ĽǶ�(����)��˳ʱ��Ϊ����
  * @param  int16_t maxspped  ����ٶ�
  * @retval None
  */
static void CalculateWheelSpeed(float vx, float vy, float omega, float radian, uint16_t maxspeed)
{
	float   fWheelSpd[5];
	float	Chassis_forward_back_ref;		//��ս������ϵ�Ĳο��ٶ�
	float	Chassis_left_right_ref;
	float 	fMaxSpd = 0;
	int16_t s16_WheelSpd[5];
	Chassis_forward_back_ref = vy*cos(radian)+vx*sin(radian);		
	Chassis_left_right_ref   = vx*cos(radian)-vy*sin(radian);
	//���ֽ���
	fWheelSpd[0] = -Chassis_forward_back_ref + Chassis_left_right_ref + omega;	//���Ͻǵ����ʼ��ʱ����
	fWheelSpd[1] =  Chassis_forward_back_ref + Chassis_left_right_ref + omega;
	fWheelSpd[2] =  Chassis_forward_back_ref - Chassis_left_right_ref + omega;
	fWheelSpd[3] = -Chassis_forward_back_ref - Chassis_left_right_ref + omega;
	  
	//���� �ҵ��ٶ����ֵ
	fMaxSpd = fabs(fWheelSpd[0]);		
	if(fabs(fWheelSpd[1]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[1]);
	if(fabs(fWheelSpd[2]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[2]);
	if(fabs(fWheelSpd[3]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[3]);
  
	//�����������ٶ�
	if(fMaxSpd > maxspeed)
	{
		s16_WheelSpd[0]   = (int16_t)(fWheelSpd[0]*(maxspeed/fMaxSpd));
		s16_WheelSpd[1]   = (int16_t)(fWheelSpd[1]*(maxspeed/fMaxSpd));
		s16_WheelSpd[2]   = (int16_t)(fWheelSpd[2]*(maxspeed/fMaxSpd));
		s16_WheelSpd[3]   = (int16_t)(fWheelSpd[3]*(maxspeed/fMaxSpd));
	}
	else
	{
		s16_WheelSpd[0]   = (int16_t)fWheelSpd[0];
		s16_WheelSpd[1]   = (int16_t)fWheelSpd[1];
		s16_WheelSpd[2]   = (int16_t)fWheelSpd[2];
		s16_WheelSpd[3]   = (int16_t)fWheelSpd[3];
	}
	memcpy((void*)ChassisData.ChassisWheelSpeedRef, (void*)s16_WheelSpd, 8);
	//Power_Limit_OutputCal();
	CM_PID_Cal();
}
/**
  * @brief  ����PID���
  * @param  None
  * @retval None
  */
void CM_PID_Cal(void)
{	
	PID_Task(&CM1SpeedPID, ChassisData.ChassisWheelSpeedRef[0], CM1_Measure.speed_rpm/10.0);
	PID_Task(&CM2SpeedPID, ChassisData.ChassisWheelSpeedRef[1], CM2_Measure.speed_rpm/10.0);
	PID_Task(&CM3SpeedPID, ChassisData.ChassisWheelSpeedRef[2], CM3_Measure.speed_rpm/10.0);
	PID_Task(&CM4SpeedPID, ChassisData.ChassisWheelSpeedRef[3], CM4_Measure.speed_rpm/10.0);
	
	if(CM1SpeedPID.output>-700 && CM1SpeedPID.output<700 ) CM1SpeedPID.output=0;
	if(CM2SpeedPID.output>-700 && CM2SpeedPID.output<700 ) CM2SpeedPID.output=0;
	if(CM3SpeedPID.output>-700 && CM3SpeedPID.output<700 ) CM3SpeedPID.output=0;
	if(CM4SpeedPID.output>-700 && CM4SpeedPID.output<700 ) CM4SpeedPID.output=0;
}

/**
  * @brief  ���õ������
  * @param  None
  * @retval None
  */

int a = 0;
void CM_SetOutput(void)
{
	if (GetGimbal_MoveMode() == STOP)    //|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
	{
		Set_CM_Speed(&hcan1, 0,0,0,0);	
	}
	else
	{
		Set_CM_Speed(&hcan1,  CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, \
							CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, \
							CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, \
							CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);	
	}
}
/**
  * @brief  ʹ����λ������PIDֵ
  * @param  None
  * @retval None
  */
static void PID_Calibration(void)
{
	CMRotatePID.kp = AppParamRealUsed.RotateSpeedPID.kp_offset;
	CMRotatePID.ki = AppParamRealUsed.RotateSpeedPID.ki_offset/1000.0;
	CMRotatePID.kd = AppParamRealUsed.RotateSpeedPID.kd_offset;
	CM1SpeedPID.kp = CM2SpeedPID.kp  = CM3SpeedPID.kp = CM4SpeedPID.kp = AppParamRealUsed.ChassisSpeedPID.kp_offset;
	CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
	CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = AppParamRealUsed.ChassisSpeedPID.kd_offset;

	CM1SpeedPID.err[2] =  CM2SpeedPID.err[2] = CM3SpeedPID.err[2] = CM4SpeedPID.err[2] = 0;//��һ����������Ҳ���㣿
	CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
}
/**
  * @brief  ���̿���������
  * @param  None
  * @retval None
  */

void CMControlLoop(void)
{
//	  PID_Calibration();//ʹ����λ��
//  	UnderpanMoveControl();	//��ͬģʽ �õ�����ֵ
////	Power_Limit_OutputCal();
//	  CM_SetOutput();
}
