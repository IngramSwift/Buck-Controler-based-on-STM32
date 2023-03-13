#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f0xx.h"
#include "pwm.h"

#define CV_MODE						0x00	//��ѹģʽ
#define CC_MODE						0x01	//����ģʽ

#define PWM_FRE						PWM_FREQUENCY	//Hz.����Ƶ��

#define RESET_FAULT_TIME 			15000u	//�������ֹͣʱ��
#define PW_VOLTAGV_IN_MAX			56.5f	//V.��������ѹ	
#define PW_VOLTAGV_IN_MIN			11.0f	//V.��С�����ѹ	
#define PW_VOLTAGV_OUT_MAX			50.5f	//V.��������ѹ	
#define PW_CURRENT_IN_MAX			8.50f	//A.����������
#define PW_CURRENT_OUT_MAX			8.50f	//A.����������

#define VOLTAGE_CFRE				(0.2*PWM_FRE)	//Hz.PID����Ƶ��,Ϊ����Ƶ�ʵ�0.05~0.2���ȽϺ���
#define CURRENT_CFRE				(0.2*PWM_FRE)	//Hz.PID����Ƶ��,Ϊ����Ƶ�ʵ�0.05~0.2���ȽϺ���

typedef struct  
{
	uint32_t recoveryCnt; //���ϼ���
	uint32_t faultTime;   //�������ʱ��
	float controlPWM;  //pwm����
	uint8_t controlMode;  //���ģʽ
	uint8_t iovp;         //�����ѹ����
	uint8_t iuvp;         //����Ƿѹ����
	uint8_t iocp;         //�����������
	uint8_t oocp;         //�����������
	uint8_t oovp;         //�����ѹ����
	uint8_t outputState;
}_uDeviceFault_Type;
extern _uDeviceFault_Type	device;


void PID_Config(void);
void ProtectReset(void);

#endif















