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
	//SetFrontAngle(0);

	/*��ʼ������*/
	//SetFrontAngle(180);
	
	while(1){
		
		

	}
}