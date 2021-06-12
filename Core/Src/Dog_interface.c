#include "Dog_interface.h"

#include "stdio.h"
#include "string.h"

#include "FreeRTOS.h" 
#include "task.h"
#include "cmsis_os.h"

#include "OLED.h"

#include "test_workspace.h"

uint8_t state_change_flag = 0;

enum Flag{
	Flag_trot = 0,
	Flag_walk,
	Flag_stand,
	Flag_balance,
	Flag_bend,
	Flag_Walk_bend,
	Flag_dance,
	Flag_rest
};

uint8_t motion_Flag[Flag_rest] = {0};

void bluetoothController(uint8_t motion_state)
{
	
	memset(motion_Flag,0,sizeof(motion_Flag));
	switch(motion_state)
	{
		case '0': 
			motion_Flag[Flag_trot] = 1;
			printf("trot\n\r");
		break;
		case '1': 
			motion_Flag[Flag_walk] = 1;
			printf("walk\n\r");
		break;
		case '2': 
			motion_Flag[Flag_balance] = 1;
			printf("balance\n\r");
		break;
	}
	
	state_change_flag = 1;
	
}

void Gait_Controller(void)
{
	OLED_ShowStr(0, 0, "System Operating!", 1);
	if(state_change_flag)
	{
		state_change_flag = 0;
		OLED_Clear();
	}
	if(motion_Flag[Flag_trot])		Trot_run();
	if(motion_Flag[Flag_walk])		Walk_run();
	if(motion_Flag[Flag_balance])	Balance_run();
}
