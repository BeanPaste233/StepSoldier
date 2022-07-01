#include "motor.h"
#include "delay.h"
#include "rc.h"

extern RC_PacketStructure rc_packet;


int main(void)
{
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	
	/*初始化流程*/
	SteerMotor_Init();
	RubMotorInit();
	PullBulletMotorTimer_Init();
	DCMotorTimer_Init();
	RemoteControl_Init();	
	/*初始化流程*/
	
	
	
	//SetFrontAngle(180);
	SetXAngle(90);//90度正前方  左转最大140度  右转30度
	//SetYAngle(125);//125度正前方    152度仰角最大值  94度俯角最大值
	while(1){
		
		
		
	}
}

