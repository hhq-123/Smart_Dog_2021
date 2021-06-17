#include "Dog_interface.h"

#include "stdio.h"
#include "string.h"

#include "FreeRTOS.h" 
#include "task.h"
#include "cmsis_os.h"

#include "Quadrudep_huaner.h"

#include "OLED.h"

#include "test_workspace.h"

uint8_t state_change_flag = 0;
int speed = 10;

extern uint16_t LSCControlPeriod;

enum Flag{
	Flag_trot = 0,
	Flag_walk,
	Flag_stand,
	Flag_dance,
	Flag_bend,
	Flag_balance,
	Flag_walk_bend,
	Flag_crab_L,
	Flag_crab_R,
	Flag_round_L,
	Flag_round_R,
	Flag_stand_up,
	Flag_power_down,
	Flag_rest
};

uint8_t motion_Flag[Flag_rest] = {0};

void bluetoothTranslater(uint8_t BTdata[])
{
	switch(BTdata[0])
	{
		case 'M':
			bluetoothStateController(BTdata[1]);
			break;
		case 'V':
			
			break;
	}
}

void bluetoothDataController(uint8_t motion_state)
{
//	case '+': 
//		if(LSCControlPeriod > 50)
//			LSCControlPeriod-=speed;
//		break;
//		case '-': 
//		if(LSCControlPeriod < 500)
//			LSCControlPeriod+=speed;
//		break;
}

void bluetoothStateController(uint8_t motion_state)
{
	memset(motion_Flag,0,sizeof(motion_Flag));
	switch(motion_state)
	{
		case '0': 
			LSCControlPeriod = 50;
			motion_Flag[Flag_trot] = 1;
			printf("trot\n\r");
		break;
		case '1': 
			LSCControlPeriod = 150;
			motion_Flag[Flag_walk] = 1;
			printf("walk\n\r");
		break;
		case '2': 
			motion_Flag[Flag_stand] = 1;
			printf("stand\n\r");
		break;
		case '3': 
			motion_Flag[Flag_dance] = 1;
			printf("dance\n\r");
		break;
		case '4': 
			motion_Flag[Flag_bend] = 1;
			printf("bend\n\r");
		break;
		case '5': 
			motion_Flag[Flag_balance] = 1;
			printf("balance\n\r");
		break;
		case '6': 
			motion_Flag[Flag_walk_bend] = 1;
			printf("walk_bend\n\r");
		break;
		case '7': 
			LSCControlPeriod = 50;
			motion_Flag[Flag_crab_L] = 1;
			printf("crab_L\n\r");
		break;
		case '8': 
			LSCControlPeriod = 50;
			motion_Flag[Flag_crab_R] = 1;
			printf("crab_R\n\r");
		break;
		case '9': 
			LSCControlPeriod = 50;
			motion_Flag[Flag_round_L] = 1;
			printf("round_L\n\r");
		break;
		case 'a': 
			LSCControlPeriod = 50;
			motion_Flag[Flag_round_R] = 1;
			printf("round_R\n\r");
		break;
		case 'b': 
			motion_Flag[Flag_stand_up] = 1;
			printf("stand_up\n\r");
		break;
	}
	state_change_flag = 1;
}

void Gait_Controller(void)
{
	//OLED_ShowStr(0, 0, "System Operating!", 1);
	if(state_change_flag)
	{
		state_change_flag = 0;
		OLED_Clear();
	}
	if(motion_Flag[Flag_trot])			Trot_run();
	if(motion_Flag[Flag_walk])			Walk_run();
	if(motion_Flag[Flag_stand])			Stand_run();
	if(motion_Flag[Flag_dance])			Dance_run();
	if(motion_Flag[Flag_bend])			Bend_run();
	if(motion_Flag[Flag_balance])		Balance_run();
	if(motion_Flag[Flag_walk_bend])	Walk_Bend_run();
	if(motion_Flag[Flag_crab_L])		Crab_L_run();
	if(motion_Flag[Flag_crab_R])		Crab_R_run();
	if(motion_Flag[Flag_round_L])		Round_L_run();
	if(motion_Flag[Flag_round_R])		Round_R_run();
	if(motion_Flag[Flag_stand_up])	Stand_Up_run();
	if(motion_Flag[Flag_power_down])	Power_down();
}
