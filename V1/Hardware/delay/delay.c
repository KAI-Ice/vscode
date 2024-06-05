/****************************************************************************************************
//=========================================delay��ʱ================================================//
��ʽһ��ϵͳ�δ�ʱ��
�ŵ㣺ȫϵ��ͨ�ã�ֻ��Ҫ���궨��CPU_FREQUENCY_MHZ����ʱ����Ƶ�޸ļ��ɡ�
ȱ�㣺ϵͳ�δ�ʱ����HAL���ʼ���ģ��ұ�����HAL���ʼ����
**************************************************************************************************/

#include "delay.h"
#define CPU_FREQUENCY_MHZ    168	// STM32ʱ����Ƶ

void delay_us(__IO uint32_t xus)
{
    int last, curr, val;
    int temp;

    while (xus != 0)
    {
        temp = xus > 900 ? 900 : xus;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        xus -= temp;
    }
}


/**
  * @brief  ���뼶��ʱ
  * @param  xms ��ʱʱ������Χ��0~4294967295
  * @retval ��
  */
void delay_ms(__IO uint32_t xms)
{
	while(xms--)
	{
		delay_us(1000);
	}
}


/**
  * @brief  �뼶��ʱ
  * @param  xs ��ʱʱ������Χ��0~4294967295
  * @retval ��
  */
void delay_s(__IO uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
}