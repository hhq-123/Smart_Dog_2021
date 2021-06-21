#ifndef DOG_INTERFACE_H
#define DOG_INTERFACE_H
#include "stm32f401xc.h"

void bluetoothTranslater(uint8_t BTdata[]);

void bluetoothStateController(uint8_t BTCommand[]);
void bluetoothMoveController(uint8_t BTCommand[]);
void bluetoothDataController(uint8_t BTCommand[]);
void Gait_Controller(void);

#endif
