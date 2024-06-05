#ifndef __TOUCH_H__
#define __TOUCH_H__
//#include "sys.h"
#include "ft6336.h"

#define TP_PRES_DOWN 0x80  //����������	  
#define TP_CATH_PRES 0x40  //�а��������� 	  
										    
//������������
typedef struct
{
	uint8_t (*init)(void);			//��ʼ��������������
	uint8_t (*scan)(void);			//ɨ�败����.0,��Ļɨ��;1,��������;	 
	uint16_t x[CTP_MAX_TOUCH]; 		//��ǰ����
	uint16_t y[CTP_MAX_TOUCH];		//�����������5������,����������x[0],y[0]����:�˴�ɨ��ʱ,����������,��
								//x[4],y[4]�洢��һ�ΰ���ʱ������. 
	uint8_t  sta;					//�ʵ�״̬ 
								//b7:����1/�ɿ�0; 
	                            //b6:0,û�а�������;1,�а�������. 
								//b5:����
								//b4~b0:���ݴ��������µĵ���(0,��ʾδ����,1��ʾ����)
}_m_tp_dev;

extern _m_tp_dev tp_dev;	 	//������������touch.c���涨��
    
uint8_t TP_Init(void);								//��ʼ��
 		  
#endif