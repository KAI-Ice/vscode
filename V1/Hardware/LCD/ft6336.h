//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F407ZGT6,����ԭ��Explorer STM32F4������,��Ƶ168MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V      //��Դ
//      GND          ��          GND          //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//    SDI(MOSI)      ��          PB5          //Һ����SPI��������д�ź�
//    SDO(MISO)      ��          PB4          //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 					      STM32��Ƭ�� 
//       SCK         ��          PB3          //Һ����SPI����ʱ���ź�
//       LED         ��          PE9         //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
//      DC/RS        ��          PE10         //Һ��������/��������ź�
//       RST         ��          PE8         //Һ������λ�����ź�
//       CS          ��          PE11        //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//     CTP_INT       ��          PE2          //���ݴ������ж��ź�
//     CTP_SDA       ��          PB10         //���ݴ�����IIC�����ź�
//     CTP_RST       ��          PG15          //���ݴ�������λ�ź�
//     CTP_SCL       ��          PB11          //���ݴ�����IICʱ���ź�
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/
#ifndef __FT6336_H
#define __FT6336_H	
//#include "sys.h"	
#include <stdint.h>

#define CTP_MAX_TOUCH 2

//����ݴ��������ӵ�оƬ����(δ����IIC����) 
//IO��������	 
#define FT_RST(n)    	HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, n)	//PCout(5)	//FT6636��λ����
//#define FT_INT(n)    				//PBin(1)		//FT6636�ж�����	

//I2C��д����	
#define FT_CMD_WR 				0X70    	//д����
#define FT_CMD_RD 				0X71		//������
  
//FT5426 ���ּĴ������� 
#define FT_DEVIDE_MODE 			0x00   		//FT6336ģʽ���ƼĴ���
#define FT_REG_NUM_FINGER   	0x02		//����״̬�Ĵ���

#define FT_TP1_REG 				0X03	  	//��һ�����������ݵ�ַ
#define FT_TP2_REG 				0X09		//�ڶ������������ݵ�ַ

#define FT_ID_G_CIPHER_MID    	0x9F      	//оƬ���ţ����ֽڣ� Ĭ��ֵ0x26
#define FT_ID_G_CIPHER_LOW    	0xA0      	//оƬ���ţ����ֽڣ� 0x01: Ft6336G  0x02: Ft6336U 
#define	FT_ID_G_LIB_VERSION		0xA1		//�汾		
#define FT_ID_G_CIPHER_HIGH   	0xA3      	//оƬ���ţ����ֽڣ� Ĭ��0x64 
#define FT_ID_G_MODE 			0xA4   		//FT6636�ж�ģʽ���ƼĴ���
#define FT_ID_G_FOCALTECH_ID  	0xA8      	//VENDOR ID Ĭ��ֵΪ0x11
#define FT_ID_G_THGROUP			0x80   		//������Чֵ���üĴ���
#define FT_ID_G_PERIODACTIVE	0x88   		//����״̬�������üĴ���


uint8_t FT6336_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
void FT6336_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
uint8_t FT6336_Init(void);
uint8_t FT6336_Scan(void);

#endif

















