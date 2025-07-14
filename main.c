#include "Myfun.h"
#include "HD44780.h"
extern volatile uint16_t led7seg_value;
int16_t pos;
float angle;
int main(void)
{
	SysTick_Config(4000000 / 1000);
	Led_Conf();
	Joy_Conf();
	Led7seg_Conf();
	LedRGB_PwmConf();
	Servodrive_Conf();
	DCMotor_Conf();
	Encoder_Conf();
	while(1)
	{
		pos = TIM3->CNT;
		angle = (float)pos / 64.0 * 360.0 / 19.0;
	}
}
