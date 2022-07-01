#include "motor.h"
#include "delay.h"
#include "rc.h"

extern RC_PacketStructure rc_packet;


int main(void)
{
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	
	/*��ʼ������*/
	SteerMotor_Init();
	RubMotorInit();
	PullBulletMotorTimer_Init();
	DCMotorTimer_Init();
	RemoteControl_Init();	
	/*��ʼ������*/
	
	
	
	//SetFrontAngle(180);
	SetXAngle(90);//90����ǰ��  ��ת���140��  ��ת30��
	//SetYAngle(125);//125����ǰ��    152���������ֵ  94�ȸ������ֵ
	while(1){
		
		
		
	}
}

