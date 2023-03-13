#if USE_USART_PRINTF
#include "upload.h"
#include "stdarg.h"	 
#include "string.h"	





/**********************************************************
**��buf����õ���cx���������ڵ�λ��
**����ֵ:0~0XFE,����������λ�õ�ƫ��.
**       0XFF,�������ڵ�cx������
*/
uint8_t My_Comma_Pos(uint8_t *buf, uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z') { return 0XFF; }//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',') { cx--; }
		buf++;
	}
	return buf-p;	 
}

/*********************************************************
**m^n����
**����ֵ:m^n�η�.
*/
uint32_t My_Pow(uint8_t m, uint8_t n)
{
	uint32_t result=1;	 
	while(n--) { result *= m; }    
	return result;
}

/*********************************************************
**strת��Ϊ����,��','����'*'����
**buf:���ִ洢��
**dx:С����λ��,���ظ����ú���
**����ֵ:ת�������ֵ
*/
int My_Str2num(uint8_t *buf, uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-') { mask|=0X02; p++; }//�Ǹ���
		if(*p==','||(*p=='*')) { break; }//����������
		if(*p=='.') { mask|=0X01; p++; }//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01) { flen++; }
		else { ilen++; }
		p++;
	}
	if(mask&0X02) { buf++; }	//ȥ������
	for(i=0; i<ilen; i++)	//�õ�������������
	{  
		ires += My_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5) { flen=5; } //���ȡ5λС��
	*dx = flen;	 		//С����λ��
	for(i=0; i<flen; i++)	//�õ�С����������
	{  
		fres += My_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res = ires * My_Pow(10,flen)+fres;
	if(mask&0X02) { res = -res; }		   
	return res;
}	  	

/*********************************************************
**����CalibMsg��Ϣ
**msg:У׼��Ϣ�ṹ��
**buf:���յ���GPS���ݻ������׵�ַ
*/
void Calib_Analysis(_CalibMsgType *msg, uint8_t *buf)
{
	uint8_t *p1, dx;			 
	uint8_t posx;    
	p1 = (uint8_t*)strstr((const char *)buf,"$CALIB");
	posx = My_Comma_Pos(p1, 1);
	if(posx!=0XFF)
	{
		msg->iBuckOffset = My_Str2num(p1+posx, &dx);
	}
	posx = My_Comma_Pos(p1,2);
	if(posx!=0XFF)
	{
		msg->iBoostOffset = My_Str2num(p1+posx, &dx);
	}
}
#endif




































