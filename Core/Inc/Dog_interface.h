#ifndef DOG_INTERFACE_H
#define DOG_INTERFACE_H
#include "stm32f401xc.h"

void bluetoothTranslater(uint8_t BTdata[]);
void bluetoothDataController(uint8_t motion_state);
void bluetoothStateController(uint8_t motion_state);

void Gait_Controller(void);

#endif
