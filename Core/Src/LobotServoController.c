/*******************************************************************************
* �ļ����� LobotServoController.c
* ���ߣ� �����ֻ������Ƽ�
* ���ڣ�20160806
* LSCϵ�ж�����ư���ο���ʾ��
*******************************************************************************/
#include "stm32f401xc.h"
#include "LobotServoController.h"
#include <stdarg.h>
#include <string.h>

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//�꺯�� ���A�ĵͰ�λ
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//�꺯�� ���A�ĸ߰�λ

extern bool isUartRxCompleted;

uint8_t LobotTxBuf[128];  //���ͻ���
uint8_t LobotRxBuf[16];
uint16_t batteryVolt;

/*********************************************************************************
 * Function:  moveServo
 * Description�� ���Ƶ������ת��
 * Parameters:   sevoID:���ID��Position:Ŀ��λ��,Time:ת��ʱ��
                    ���IDȡֵ:0<=���ID<=31,Timeȡֵ: Time > 0
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0)) {  //���ID���ܴ���31,�ɸ��ݶ�Ӧ���ư��޸�
		return;
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //���֡ͷ
	LobotTxBuf[2] = 8;
	LobotTxBuf[3] = CMD_SERVO_MOVE;           //���ݳ���=Ҫ���ƶ����*3+5���˴�=1*3+5//������ƶ�ָ��
	LobotTxBuf[4] = 1;                        //Ҫ���ƵĶ������
	LobotTxBuf[5] = GET_LOW_BYTE(Time);       //ȡ��ʱ��ĵͰ�λ
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //ȡ��ʱ��ĸ߰�λ
	LobotTxBuf[7] = servoID;                  //���ID
	LobotTxBuf[8] = GET_LOW_BYTE(Position);   //ȡ��Ŀ��λ�õĵͰ�λ
	LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //ȡ��Ŀ��λ�õĸ߰�λ
	HAL_UART_Transmit( &huart6 , LobotTxBuf , 10, 0xFFFF);
	//uartWriteBuf(LobotTxBuf, 10);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description�� ���ƶ�����ת��
 * Parameters:   servos[]:����������飬Num:�������,Time:ת��ʱ��
                    0 < Num <= 32,Time > 0
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
	uint8_t index = 7;
	uint8_t i = 0;

	if (Num < 1 || Num > 32 || !(Time > 0)) {
		return;                                          //���������Ϊ��ʹ���32��ʱ�䲻��Ϊ��
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //���֡ͷ
	LobotTxBuf[2] = Num * 3 + 5;                       //���ݳ��� = Ҫ���ƶ����*3+5
	LobotTxBuf[3] = CMD_SERVO_MOVE;                    //������ƶ�ָ��
	LobotTxBuf[4] = Num;                               //Ҫ���ƵĶ������
	LobotTxBuf[5] = GET_LOW_BYTE(Time);                //ȡ��ʱ��ĵͰ�λ
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //ȡ��ʱ��ĸ߰�λ

	for (i = 0; i < Num; i++) {                        //ѭ�������ID�Ͷ�ӦĿ��λ��
		LobotTxBuf[index++] = servos[i].ID;              //�����ID
		LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //���Ŀ��λ�õͰ�λ
		LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//���Ŀ��λ�ø߰�λ
	}
	
	HAL_UART_Transmit( &huart6 , LobotTxBuf , LobotTxBuf[2] + 2, 0xFFFF);
	//uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);             //����
}

/*********************************************************************************
 * Function:  moveServos
 * Description�� ���ƶ�����ת��
 * Parameters:   Num:�������,Time:ת��ʱ��,...:���ID,ת���ǣ����ID,ת���Ƕ� �������
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr;  //

	va_start(arg_ptr, Time); //ȡ�ÿɱ�����׵�ַ
	if (Num < 1 || Num > 32) {
		return;               //���������Ϊ��ʹ���32��ʱ�䲻��С��0
	}
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //���֡ͷ
	LobotTxBuf[2] = Num * 3 + 5;                //���ݳ��� = Ҫ���ƶ���� * 3 + 5
	LobotTxBuf[3] = CMD_SERVO_MOVE;             //����ƶ�ָ��
	LobotTxBuf[4] = Num;                        //Ҫ���ƶ����
	LobotTxBuf[5] = GET_LOW_BYTE(Time);         //ȡ��ʱ��ĵͰ�λ
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //ȡ��ʱ��ĸ߰�λ

	for (i = 0; i < Num; i++) {//�ӿɱ������ȡ�ò�ѭ�������ID�Ͷ�ӦĿ��λ��
		temp = va_arg(arg_ptr, int);//�ɲ�����ȡ�ö��ID
		LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);  //�ɱ������ȡ�ö�ӦĿ��λ��
		LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //���Ŀ��λ�õͰ�λ
		LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//���Ŀ��λ�ø߰�λ
	}

	va_end(arg_ptr);  //�ÿ�arg_ptr
	
	HAL_UART_Transmit( &huart6 , LobotTxBuf , LobotTxBuf[2] + 2, 0xFFFF);
	//uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);    //����
}


/*********************************************************************************
 * Function:  runActionGroup
 * Description�� ����ָ��������
 * Parameters:   NumOfAction:���������, Times:ִ�д���
 * Return:       �޷���
 * Others:       Times = 0 ʱ����ѭ��
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  //���֡ͷ
	LobotTxBuf[2] = 5;                      //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ5
	LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //������ж���������
	LobotTxBuf[4] = numOfAction;            //���Ҫ���еĶ������
	LobotTxBuf[5] = GET_LOW_BYTE(Times);    //ȡ��Ҫ���д����ĵͰ�λ
	LobotTxBuf[6] = GET_HIGH_BYTE(Times);   //ȡ��Ҫ���д����ĸ߰�λ

	HAL_UART_Transmit( &huart6 , LobotTxBuf , 7 , 0xFFFF);
	//uartWriteBuf(LobotTxBuf, 7);            //����
}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description�� ֹͣ����������
 * Parameters:   Speed: Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
	LobotTxBuf[0] = FRAME_HEADER;     //���֡ͷ
	LobotTxBuf[1] = FRAME_HEADER;
	LobotTxBuf[2] = 2;                //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ2
	LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //���ֹͣ���ж���������

	HAL_UART_Transmit( &huart6 , LobotTxBuf , 4 , 0xFFFF);
	//uartWriteBuf(LobotTxBuf, 4);      //����
}
/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description�� �趨ָ��������������ٶ�
 * Parameters:   NumOfAction: ��������� , Speed:Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   //���֡ͷ
	LobotTxBuf[2] = 5;                       //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ5
	LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  //������ö������ٶ�����
	LobotTxBuf[4] = numOfAction;             //���Ҫ���õĶ������
	LobotTxBuf[5] = GET_LOW_BYTE(Speed);     //���Ŀ���ٶȵĵͰ�λ
	LobotTxBuf[6] = GET_HIGH_BYTE(Speed);    //���Ŀ������ĸ߰�λ

	HAL_UART_Transmit( &huart6 , LobotTxBuf , 7 , 0xFFFF);
	//uartWriteBuf(LobotTxBuf, 7);             //����
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description�� �������ж�����������ٶ�
 * Parameters:   Speed: Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
	setActionGroupSpeed(0xFF, Speed);  //���ö������ٶ��趨�����Ϊ0xFFʱ������������ٶ�
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description�� ���ͻ�ȡ��ص�ѹ����
 * Parameters:   Timeout�����Դ���
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
	LobotTxBuf[0] = FRAME_HEADER;  //���֡ͷ
	LobotTxBuf[1] = FRAME_HEADER;
	LobotTxBuf[2] = 2;             //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ2
	LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //����ȡ��ص�ѹ����

	HAL_UART_Transmit( &huart6 , LobotTxBuf , 4 , 0xFFFF);
	//uartWriteBuf(LobotTxBuf, 4);   //����
}

void receiveHandle()
{
	//���Ը��ݶ��ο����ֲ��������ָ��
	if (isUartRxCompleted) {
		isUartRxCompleted = false;
		switch (LobotRxBuf[3]) {
		case CMD_GET_BATTERY_VOLTAGE: //��ȡ��ѹ
			batteryVolt = (((uint16_t)(LobotRxBuf[5])) << 8) | (LobotRxBuf[4]);
			break;
		default:
			break;
		}
	}
}
