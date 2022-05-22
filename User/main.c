#include "motor.h"
#include "delay.h"
#include "rc.h"



RC_PacketStructure rc_packet;

uint8_t rc_data[RC_FRAME_LENGTH];


int main(void)
{
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	
	/*初始化流程*/
	/*SteerMotor_Init();
	RubMotorInit();
	PullBulletMotorTimer_Init();*/
	//DCMotorTimer_Init();
	RubMotorInit();
	delay_ms(1000);
	/*SetLeftSpeed(1100);
	SetRightSpeed(1100);*/
	TIM_SetCompare2(TIM2,1100);
	TIM_SetCompare3(TIM2,1100);
	
	/*初始化流程*/

	//delay_ms(3000);

	while(1){
		
		
		/*PullMotor_Forward(10000);
		delay_ms(2000);
		PullMotor_Back(10000);
		delay_ms(2000);
		*/
	}
}