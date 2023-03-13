#include "calibration.h"
#include "delay.h"
#include "pwm.h"
#include "led.h"
#include "oled.h"
#include "adc.h"
#include "key.h"

_uCalibration_Type	calib;

/**
  * @brief  This function is...
  * @param  None
  * @retval None
  */
void CalibrationOutput(void)
{
	uint8_t calibFlag=0;
	float v=0.0f;
	float i=0.0f;
	uint8_t keyCode=0;
	
	//���У׼ģʽ
	//��ѹ15.000VУ׼
	//���������ѹʹ�����ѹ�ȶ���15.000V,��"�л�"������У׼��ѹֵ,���������У׼
	//����1.000AУ׼
	//ʹ�õ��Ӹ��غ������1.000A,��"�л�"������У׼����ֵ,�������ѹ�͵�����У׼ֵ��flash
	
	OLED_ClearAll();
	OLED_ShowString(24, 0, "Calib Mode", 10, 16);
	OLED_ShowString(0, 3, "Buck Vout 15V", 13, 16);
	
	PWM_Start();
	PWM_PULSE = (uint16_t)(0.75f*pwm.period);
	
	while(1)
	{
		i = i*0.7f + 0.3f*power.channel[1];
		v = v*0.7f + 0.3f*power.channel[2];

		keyCode = Key_Scan();
		if(keyCode==KEY_VI_FLAG)
		{
			BUZZER_ON;
			if(calibFlag==0)
			{	//�����ѹУ׼ֵ������
				calibFlag = 1;
				calib.voRatio = CALIBRATION_V / v;
				OLED_ShowString(0, 3, "Buck Iout 0A ", 13, 16);
			}
			else if(calibFlag==1)
			{	//����������У׼ֵ������
				calibFlag = 2;
				calib.ioZero = i;
				OLED_ShowString(0, 3, "Buck Iout 2A ", 13, 16);
			}
			else if(calibFlag==2)
			{	//����1A����У׼ֵ������
				calibFlag = 3;
				calib.ioRatio = CALIBRATION_I / (i-calib.ioZero);
				break;
			}
			delay_us(120000);
		}
		delay_us(500);
	}
	
	//
	PWM_PULSE = 0;
	
	delay_us(500000);
	//����У׼����
	Save_CalibData();
	//��λ
	NVIC_SystemReset();
	while(1)
	{;}
	
}

/**
  * @brief  This function is...
  * @param  None
  * @retval None
  */
void Save_CalibData(void)
{
	uint8_t cnt=0;
	uint32_t temp=0;
	
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	
	if(FLASH_ErasePage(FLASH_USER_START_ADDR)!= FLASH_COMPLETE)
    {	//��������
		while(1)
		{	
			BUZZER_ON;
			LED_OUT_TOGGLE;
			delay_us(800000);
		}
    }
	
	//����У׼��ѹ����
	temp = (uint32_t)(100000 * calib.voRatio);
	if(FLASH_ProgramWord(FLASH_USER_START_ADDR, temp)==FLASH_COMPLETE)
	{
		cnt++;
	}
	
	//����������
	temp = (uint32_t)(100000 * calib.ioZero);
	if(FLASH_ProgramWord(FLASH_USER_START_ADDR+4, temp)==FLASH_COMPLETE)
	{
		cnt++;
	}
	
	//����У׼��������
	temp = (uint32_t)(100000 * calib.ioRatio);
	if(FLASH_ProgramWord(FLASH_USER_START_ADDR+8, temp)==FLASH_COMPLETE)
	{
		cnt++;
	}
	
	FLASH_Lock();
	
	if(cnt==3)
	{	//������ȷ
		BUZZER_ON;
		delay_us(120000);
		BUZZER_ON;
		delay_us(120000);
	}
	else
	{	//�������
		while(1)
		{	
			BUZZER_ON;
			LED_OUT_TOGGLE;
			delay_us(800000);
		}
	}
	
}

/**
  * @brief  This function is...
  * @param  None
  * @retval None
  */
void Read_CalibData(void)
{
	uint32_t temp=0;
	uint32_t address=FLASH_USER_START_ADDR;
	
	//����ѹ
	temp = *(__IO uint32_t *)address;
	calib.voRatio = (float)temp / 100000;
	calib.viRatio = calib.voRatio;

	//��������
	address += 4;
	temp = *(__IO uint32_t *)address;
	calib.ioZero = (float)temp / 100000;
	
	//������
	address += 4;
	temp = *(__IO uint32_t *)address;
	calib.ioRatio = (float)temp / 100000;
}












