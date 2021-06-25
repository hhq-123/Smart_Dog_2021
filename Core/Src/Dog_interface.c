#include "Dog_interface.h"

#include "stdio.h"
#include "string.h"

#include "FreeRTOS.h" 
#include "task.h"
#include "cmsis_os.h"

#include "Quadruped_huaner.h"

#include "OLED.h"

#include "test_workspace.h"

uint8_t state_change_flag = 0;
int speed = 10;

uint8_t Dog_command_state = 0;

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
int Dog_state[11] = {0};//第一状态 第二状态 八个参数

void bluetoothTranslater(uint8_t BTdata[])
{
	printf("bluetoothTranslater\n\r");
	//printf("%d %d %d\n\r", BTdata[0],BTdata[1], BTdata[2]);
	if(BTdata[0]==0x55&&BTdata[1]==0x55)
		Dog_command_state = BTdata[3];
		switch(BTdata[3])
		{
			case 0:
				//bluetoothStateController(&BTdata[3]);
				printf("Error\n\r");
				break;
			case 1:
				bluetoothStateController(BTdata);
				printf("StateController\n\r");
				break;
			case 2:
				bluetoothMoveController(BTdata);
				break;
		}
}

void bluetoothStateController(uint8_t BTCommand[])
{
	if(BTCommand[2]==0x0B)
		//printf("%s", BTCommand);
	{
		Dog_state[0] = BTCommand[3];
		Dog_state[1] = BTCommand[4];
		sscanf(BTCommand+5,"%d %d %d %d %d %d %d %d", &Dog_state[2], &Dog_state[3], &Dog_state[4],&Dog_state[5], &Dog_state[6], &Dog_state[7],&Dog_state[8],&Dog_state[9]);
	}
	else
		printf("State Command Error!");
	
	printf("%d %d %d %d %d %d %d %d\n\r", Dog_state[2],Dog_state[3], Dog_state[4], Dog_state[5], Dog_state[6],Dog_state[7], Dog_state[8], Dog_state[9]);
}

void bluetoothDataController(uint8_t BTCommand[])
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

void bluetoothMoveController(uint8_t BTCommand[])
{
	if(BTCommand[2]==0x03)
	{
		Dog_state[0] = BTCommand[3];
		Dog_state[1] = BTCommand[4];
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
	switch(Dog_state[0])
	{
		case 0:
			osDelay(10);
			break;
			
		case 1:
			switch(Dog_state[1])
			{
				case 0:
					//printf("trot\n\r");
					Trot_state(Dog_state[2], Dog_state[3],Dog_state[4],Dog_state[5],Dog_state[6],Dog_state[7],Dog_state[8],Dog_state[9]);
				break;
				case 1:
					Walk_state(Dog_state[2], Dog_state[3],Dog_state[4],Dog_state[5],Dog_state[6],Dog_state[7],Dog_state[8],Dog_state[9]);
					//printf("walk\n\r");
				break;
				case 2:
					//printf("crab\n\r");
					Crab_state(Dog_state[2], Dog_state[3],Dog_state[4],Dog_state[5],Dog_state[6],Dog_state[7],Dog_state[8],Dog_state[9]);
				break;
				case 3:
					//printf("round\n\r");
					Round_state(Dog_state[2], Dog_state[3],Dog_state[4],Dog_state[5],Dog_state[6],Dog_state[7],Dog_state[8],Dog_state[9]);
				//Step_state(Dog_state[2], Dog_state[3],Dog_state[4]);
				break;
			}
			break;
		case 2:
			switch(Dog_state[1])
			{
			 case 0:
				Trot_run();
			 //printf("Trot_run\n\r");
			 break;
			 case 1:
				Walk_run();
			 //printf("Walk_run\n\r");
			 break;
			 case 2:
				Step_run();
			 //printf("Stand_run\n\r");
			 break;
			 case 3:
				Stand_run();
			 //printf("Dance_run\n\r");
			 break;
			 case 4:
				Crab_L_run();
			 break;
			 case 5:
				Round_L_run();
			 break;
			 case 6:
				Round_R_run();
			 break;
			 case 7:
				Crab_R_run();
			 break;
			 case 8:
				Balance_run();
			 break;
			 case 9:
				Dance_run();
			 break;
			 case 10:
				Move_zero(1000);
			 break;
			 case 11:
				Power_down();
			 break;
			 default:
				osDelay(10);
			}
		break;
		case 3:
			osDelay(10);
			break;
		default:
			osDelay(10);
	}
}
