/****************************************************************************************************
//=========================================delay延时================================================//
方式一：系统滴答定时器
优点：全系列通用，只需要将宏定义CPU_FREQUENCY_MHZ根据时钟主频修改即可。
缺点：系统滴答定时器是HAL库初始化的，且必须有HAL库初始化。
**************************************************************************************************/

#include "delay.h"
#define CPU_FREQUENCY_MHZ    168	// STM32时钟主频

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
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_ms(__IO uint32_t xms)
{
	while(xms--)
	{
		delay_us(1000);
	}
}


/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_s(__IO uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
}