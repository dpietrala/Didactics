#include "Myfun.h"
volatile uint32_t tick = 0;
volatile uint8_t led7seg_pos = 0;
volatile uint16_t led7seg_value;

volatile uint16_t val = 0;
volatile uint16_t tab[200];

uint8_t bufwrite[100];
volatile uint16_t rp;
volatile uint16_t rpmax;
uint8_t bufread[100];
volatile uint16_t wp;

uint8_t dmabufwrite[100];
uint8_t dmabufread[100];

uint32_t cnt_rxne = 0;
uint32_t cnt_idle = 0;
uint32_t cnt_cmf = 0;

uint8_t whoacc;
int16_t accx;
int16_t accy;
int16_t accz;
double accx_g;
double accy_g;
double accz_g;

uint8_t whomag;
int16_t magx;
int16_t magy;
int16_t magz;
double magx_g;
double magy_g;
double magz_g;

uint32_t HCSR04_Time = 0;
double HCSR04_Distancemm = 0.0;

void delay_ms(uint32_t ms)
{
	tick = 0;
	while(tick < ms);
}
void Led_Conf(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	
	GPIOC->MODER &= ~GPIO_MODER_MODE6;
	GPIOC->MODER |= GPIO_MODER_MODE6_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODE7;
	GPIOC->MODER |= GPIO_MODER_MODE7_0;
	GPIOC->MODER &= ~GPIO_MODER_MODE8;
	GPIOC->MODER |= GPIO_MODER_MODE8_0;
	GPIOC->MODER &= ~GPIO_MODER_MODE9;
	GPIOC->MODER |= GPIO_MODER_MODE9_0;
	GPIOE->MODER &= ~GPIO_MODER_MODE4;
	GPIOE->MODER |= GPIO_MODER_MODE4_0;
	GPIOD->MODER &= ~GPIO_MODER_MODE3;
	GPIOD->MODER |= GPIO_MODER_MODE3_0;
	GPIOE->MODER &= ~GPIO_MODER_MODE5;
	GPIOE->MODER |= GPIO_MODER_MODE5_0;
	GPIOE->MODER &= ~GPIO_MODER_MODE6;
	GPIOE->MODER |= GPIO_MODER_MODE6_0;
}
void Led_OnOff(uint8_t num, eLed state)
{
	switch(num)
	{
		case 0:
			if(state == LedOff)	GPIOC->ODR &= ~GPIO_ODR_OD6;
			if(state == LedOn)	GPIOC->ODR |= GPIO_ODR_OD6;
			if(state == LedTog)	GPIOC->ODR ^= GPIO_ODR_OD6;
		break;
		
		case 1:
			if(state == LedOff)	GPIOC->ODR &= ~GPIO_ODR_OD7;
			if(state == LedOn)	GPIOC->ODR |= GPIO_ODR_OD7;
			if(state == LedTog)	GPIOC->ODR ^= GPIO_ODR_OD7;
		break;
		
		case 2:
			if(state == LedOff)	GPIOC->ODR &= ~GPIO_ODR_OD8;
			if(state == LedOn)	GPIOC->ODR |= GPIO_ODR_OD8;
			if(state == LedTog)	GPIOC->ODR ^= GPIO_ODR_OD8;
		break;
		
		case 3:
			if(state == LedOff)	GPIOC->ODR &= ~GPIO_ODR_OD9;
			if(state == LedOn)	GPIOC->ODR |= GPIO_ODR_OD9;
			if(state == LedTog)	GPIOC->ODR ^= GPIO_ODR_OD9;
		break;
		
		case 4:
			if(state == LedOff)	GPIOE->ODR &= ~GPIO_ODR_OD4;
			if(state == LedOn)	GPIOE->ODR |= GPIO_ODR_OD4;
			if(state == LedTog)	GPIOE->ODR ^= GPIO_ODR_OD4;
		break;
		
		case 5:
			if(state == LedOff)	GPIOD->ODR &= ~GPIO_ODR_OD3;
			if(state == LedOn)	GPIOD->ODR |= GPIO_ODR_OD3;
			if(state == LedTog)	GPIOD->ODR ^= GPIO_ODR_OD3;
		break;
		
		case 6:
			if(state == LedOff)	GPIOE->ODR &= ~GPIO_ODR_OD5;
			if(state == LedOn)	GPIOE->ODR |= GPIO_ODR_OD5;
			if(state == LedTog)	GPIOE->ODR ^= GPIO_ODR_OD5;
		break;
		
		case 7:
			if(state == LedOff)	GPIOE->ODR &= ~GPIO_ODR_OD6;
			if(state == LedOn)	GPIOE->ODR |= GPIO_ODR_OD6;
			if(state == LedTog)	GPIOE->ODR ^= GPIO_ODR_OD6;
		break;
	}
}
void SysTick_Handler(void)
{
	tick++;
	Led7seg_WriteNumber();
}
void Joy_Conf(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	GPIOE->MODER &= ~GPIO_MODER_MODE0; //konfiguracja jako cyfrowe wejscie
	GPIOE->MODER &= ~GPIO_MODER_MODE1;
	GPIOE->MODER &= ~GPIO_MODER_MODE2;
	GPIOE->MODER &= ~GPIO_MODER_MODE3;
	GPIOE->MODER &= ~GPIO_MODER_MODE15;
}
eJoy Joy_Read(void)
{
	eJoy state = JoyNull;
	if((GPIOE->IDR & GPIO_IDR_ID0) == RESET)		state = Right;
	if((GPIOE->IDR & GPIO_IDR_ID1) == RESET)		state = Left;
	if((GPIOE->IDR & GPIO_IDR_ID2) == RESET)		state = Down;
	if((GPIOE->IDR & GPIO_IDR_ID3) == RESET)		state = Up;
	if((GPIOE->IDR & GPIO_IDR_ID15) == RESET)		state = Center;
	return state;
}
void Led7seg_Conf(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	PWR->CR2 |= PWR_CR2_IOSV;
	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
	
	GPIOB->MODER &= ~GPIO_MODER_MODE2;
	GPIOB->MODER |= GPIO_MODER_MODE2_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE3;
	GPIOB->MODER |= GPIO_MODER_MODE3_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE4;
	GPIOB->MODER |= GPIO_MODER_MODE4_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	GPIOB->MODER |= GPIO_MODER_MODE5_0;
	
	GPIOG->MODER &= ~GPIO_MODER_MODE0;
	GPIOG->MODER |= GPIO_MODER_MODE0_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE1;
	GPIOG->MODER |= GPIO_MODER_MODE1_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE2;
	GPIOG->MODER |= GPIO_MODER_MODE2_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE3;
	GPIOG->MODER |= GPIO_MODER_MODE3_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE4;
	GPIOG->MODER |= GPIO_MODER_MODE4_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE5;
	GPIOG->MODER |= GPIO_MODER_MODE5_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE6;
	GPIOG->MODER |= GPIO_MODER_MODE6_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE9;
	GPIOG->MODER |= GPIO_MODER_MODE9_0;
}
void Led7seg_WriteDigit(uint8_t pos, uint8_t num)
{
	GPIOB->ODR &= ~GPIO_ODR_OD2;
	GPIOB->ODR &= ~GPIO_ODR_OD3;
	GPIOB->ODR &= ~GPIO_ODR_OD4;
	GPIOB->ODR &= ~GPIO_ODR_OD5;
	GPIOG->ODR &= ~GPIO_ODR_OD0;
	GPIOG->ODR &= ~GPIO_ODR_OD1;
	GPIOG->ODR &= ~GPIO_ODR_OD2;
	GPIOG->ODR &= ~GPIO_ODR_OD3;
	GPIOG->ODR &= ~GPIO_ODR_OD4;
	GPIOG->ODR &= ~GPIO_ODR_OD5;
	GPIOG->ODR &= ~GPIO_ODR_OD6;
	GPIOG->ODR &= ~GPIO_ODR_OD9;
	
	switch(pos)
	{
		case 0:	GPIOB->ODR |= GPIO_ODR_OD5;	break;
		case 1:	GPIOB->ODR |= GPIO_ODR_OD4;	break;
		case 2:	GPIOB->ODR |= GPIO_ODR_OD3;	break;
		case 3:	GPIOB->ODR |= GPIO_ODR_OD2;	break;
	}
	switch(num)
	{
		case 0:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | 
										GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5;
			break;
		case 1:
			GPIOG->ODR |= GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2;
			break;
		case 2:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_3 |
										GPIO_ODR_ODR_4 | GPIO_ODR_ODR_6;
			break;
		case 3:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 |
										GPIO_ODR_ODR_3 | GPIO_ODR_ODR_6;
			break;
		case 4:
			GPIOG->ODR |= GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_5 |
										GPIO_ODR_ODR_6;
			break;
		case 5:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3 |
										GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6;
			break;
		case 6:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3 |
										GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6;
			break;
		case 7:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2;
			break;
		case 8:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 |
										GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 |
										GPIO_ODR_ODR_6;
			break;
		case 9:
			GPIOG->ODR |= GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 |
										GPIO_ODR_ODR_3 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6;
			break;
	}
}
void Led7seg_WriteNumber(void)
{
	uint8_t tab[4];
	tab[0] = (led7seg_value % 10) / 1;
	tab[1] = (led7seg_value % 100) / 10;
	tab[2] = (led7seg_value % 1000) / 100;
	tab[3] = (led7seg_value % 10000) / 1000;
	Led7seg_WriteDigit(led7seg_pos, tab[led7seg_pos]);
	led7seg_pos++;
	if(led7seg_pos >= 4)
		led7seg_pos = 0;
}
void LPUART1_Conf_Basic(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER0 & ~GPIO_MODER_MODE1;
	GPIOC->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODE1_1;
	GPIOC->AFR[0] |= 0x00000088;
	
	LPUART1->BRR = (256 * 4000000) / 57600;
	LPUART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}
void LPUART1_Conf_Interrupt(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	
	GPIOC->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1;
	GPIOC->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
	GPIOC->AFR[0] |= 0x00000088;
	
	LPUART1->CR2 |= 'A' << USART_CR2_ADD_Pos;
	LPUART1->BRR = (256 * 4000000) / 57600;
	LPUART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | 
									USART_CR1_IDLEIE | USART_CR1_CMIE;
	
	NVIC_EnableIRQ(LPUART1_IRQn);
}
void LPUART1_IRQHandler(void)
{
	if((LPUART1->ISR & USART_ISR_TXE) != RESET)
	{
		LPUART1->TDR = bufwrite[rp];
		if(rp++ >= rpmax)
			LPUART1->CR1 &= ~USART_CR1_TXEIE;
		//Ewentualny dodatkowy kod
	}
	if((LPUART1->ISR & USART_ISR_RXNE) != RESET)
	{
		bufread[wp] = LPUART1->RDR;
		if(wp++ >= 100)
			wp = 0;
		Led_OnOff(7, LedTog);
		cnt_rxne++;
	}
	if((LPUART1->ISR & USART_ISR_IDLE) != RESET)
	{
		cnt_idle++;
		wp = 0;
		LPUART1_ReinitDMA();
		LPUART1->ICR |= USART_ICR_IDLECF; //kasowanie flagi przerwania
	}
	if((LPUART1->ISR & USART_ISR_CMF) != RESET)
	{
		cnt_cmf++;
		LPUART1->ICR |= USART_ICR_CMCF; //kasowanie flagi przerwania
	}
}
void LPUART1_SendWithInterrupt(const char* str, uint16_t len)
{
	for(int i=0;i<len;i++)
		bufwrite[i] = str[i];
	rp = 0;
	rpmax = len-1;
	LPUART1->CR1 |= USART_CR1_TXEIE;
}
void ComSendChar(USART_TypeDef *USARTx, char c)
{
	while((USARTx->ISR & USART_ISR_TXE) == RESET){;}
		USARTx->TDR = c;
}
void ComPuts(USART_TypeDef* USARTx, const char* str)
{
	while(*str)
		ComSendChar(USARTx, *str++);
}
void ADC_ConfBasic(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADCEN;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL;
	GPIOC->MODER |= GPIO_MODER_MODER2;
	ADC3->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
	ADC3->CFGR |= ADC_CFGR_DISCEN;
	ADC3->SQR1 |= (3<< ADC_SQR1_SQ1_Pos);
	ADC3->SMPR1 |= ADC_SMPR1_SMP3;
	ADC3->ISR |= ADC_ISR_EOC;
	ADC3->CR |= ADC_CR_ADSTART;
}
void ADC_ConfInterrupt(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADCEN;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL;
	
	GPIOC->MODER |= GPIO_MODER_MODER2;
	
	ADC3->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
	ADC3->SQR1 |= (3<<ADC_SQR1_SQ1_Pos);
	ADC3->SMPR1 |= ADC_SMPR1_SMP3;
	ADC3->CFGR |= ADC_CFGR_CONT;
	ADC3->IER |= ADC_IER_EOCIE;
	NVIC_EnableIRQ(ADC3_IRQn);
	ADC3->CR |= ADC_CR_ADSTART;
}
void ADC3_IRQHandler(void)
{
	if((ADC3->ISR & ADC_ISR_EOC) != RESET)
	{
		val = ADC3->DR;
		led7seg_value = val;
	}
}
void ADC_Conf_DMA_2Channels(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL;
	
	GPIOC->MODER |= GPIO_MODER_MODER2;
	
	DMA1_Channel3->CPAR = (uint32_t)&ADC3->DR;
	DMA1_Channel3->CMAR = (uint32_t)tab;
	DMA1_Channel3->CNDTR = (uint16_t)2;
	DMA1_CSELR->CSELR = (0 << DMA_CSELR_C3S_Pos);
	DMA1_Channel3->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 |	DMA_CCR_MSIZE_0 | DMA_CCR_EN;
	
	ADC123_COMMON->CCR |= ADC_CCR_TSEN;
	ADC3->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
	ADC3->SQR1 = (1<<ADC_SQR1_L_Pos) | (3<<ADC_SQR1_SQ1_Pos) | (17<<ADC_SQR1_SQ2_Pos);
	ADC3->SMPR1 = ADC_SMPR1_SMP3;
	ADC3->SMPR2 = ADC_SMPR2_SMP17;
	ADC3->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN | ADC_CFGR_CONT;
	ADC3->CR |= ADC_CR_ADSTART;
}
void ADC_Conf_DMA_2ChannelsMean(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL;
	
	GPIOC->MODER |= GPIO_MODER_MODER2;
	
	DMA1_Channel3->CPAR = (uint32_t)&ADC3->DR;
	DMA1_Channel3->CMAR = (uint32_t)tab;
	DMA1_Channel3->CNDTR = (uint16_t)200;
	DMA1_CSELR->CSELR = (0 << DMA_CSELR_C3S_Pos);
	DMA1_Channel3->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 |	DMA_CCR_MSIZE_0 | DMA_CCR_EN;
	
	ADC123_COMMON->CCR |= ADC_CCR_TSEN;
	ADC3->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
	ADC3->SQR1 = (1<<ADC_SQR1_L_Pos) | (3<<ADC_SQR1_SQ1_Pos) | (17<<ADC_SQR1_SQ2_Pos);
	ADC3->SMPR1 = ADC_SMPR1_SMP3;
	ADC3->SMPR2 = ADC_SMPR2_SMP17;
	ADC3->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN | ADC_CFGR_CONT;
	ADC3->CR |= ADC_CR_ADSTART;
}
void I2C3_ConfBasic(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	PWR->CR2 |= PWR_CR2_IOSV;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
	
	GPIOG->MODER &= ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE7;
	GPIOG->MODER |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE7_1;
	GPIOG->AFR[0] = 0x40000000;
	GPIOG->AFR[1] |= 0x00000004;
	
	I2C3->TIMINGR = 0x04;
	I2C3->CR1 = I2C_CR1_PE;
}
void LSM303_ACC_ReadWhoAmI(void)
{
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) {;}
	I2C3->TXDR = 0x0f;
	while((I2C3->ISR & I2C_ISR_TC) == RESET) {;}
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_RXNE) == RESET) {;}
	whoacc = I2C3->RXDR;
}
void LSM303_ACC_WriteConfig(void)
{
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x20;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x67;
	
	delay_ms(10);
	
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x23;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x04;
}
void LSM303_ACC_Read(void)
{
	uint8_t tabacc[6];
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) {;}
	I2C3->TXDR = 0x28;
	while((I2C3->ISR & I2C_ISR_TC) == RESET) {;}
	
	I2C3->CR2 = (6 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1d << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	for(int i=0;i<6;i++)
	{
		while((I2C3->ISR & I2C_ISR_RXNE) == RESET) {;}
		tabacc[i] = I2C3->RXDR;
	}

	accx = (int16_t)(((uint16_t)tabacc[1] << 8) + ((uint16_t)tabacc[0] << 0));
	accy = (int16_t)(((uint16_t)tabacc[3] << 8) + ((uint16_t)tabacc[2] << 0));
	accz = (int16_t)(((uint16_t)tabacc[5] << 8) + ((uint16_t)tabacc[4] << 0));
	accx_g = (double)accx / 16384.0;
	accy_g = (double)accy / 16384.0;
	accz_g = (double)accz / 16384.0;
}
void LSM303_MAG_ReadWhoAmI(void)
{
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) {;}
	I2C3->TXDR = 0x0f;
	while((I2C3->ISR & I2C_ISR_TC) == RESET) {;}
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_RXNE) == RESET) {;}
	whomag = I2C3->RXDR;
}
void LSM303_MAG_WriteConfig(void)
{
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x20;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0xfe;
	delay_ms(10);
	
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x21;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x60;
	delay_ms(10);
	
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x22;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x00;
	delay_ms(10);
	
	I2C3->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x23;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) ;
	I2C3->TXDR = 0x0c;
	delay_ms(10);
}
void LSM303_MAG_Read(void)
{
	uint8_t tabmag[6];
	I2C3->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C3->ISR & I2C_ISR_TXIS) == RESET) {;}
	I2C3->TXDR = 0x28;
	while((I2C3->ISR & I2C_ISR_TC) == RESET) {;}
	
	I2C3->CR2 = (6 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (0x1e << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	for(int i=0;i<6;i++)
	{
		while((I2C3->ISR & I2C_ISR_RXNE) == RESET) {;}
		tabmag[i] = I2C3->RXDR;
	}

	magx = (int16_t)(((uint16_t)tabmag[1] << 8) + ((uint16_t)tabmag[0] << 0));
	magy = (int16_t)(((uint16_t)tabmag[3] << 8) + ((uint16_t)tabmag[2] << 0));
	magz = (int16_t)(((uint16_t)tabmag[5] << 8) + ((uint16_t)tabmag[4] << 0));
	magx_g = (double)magx / 2048.0;
	magy_g = (double)magy / 2048.0;
	magz_g = (double)magz / 2048.0;
}
void LPUART1_Conf_DMA(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	GPIOC->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1;
	GPIOC->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
	GPIOC->AFR[0] |= 0x00000088;
	
	LPUART1->BRR = (256 * 4000000) / 57600;
	LPUART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	LPUART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(LPUART1_IRQn);
	
	DMA2_Channel6->CPAR = (uint32_t)&LPUART1->TDR;
	DMA2_Channel6->CMAR = (uint32_t)dmabufwrite;
	DMA2_Channel6->CNDTR = (uint16_t)100;
	DMA2_CSELR->CSELR |= 0x00400000; //Channel 6
	DMA2_Channel6->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
	NVIC_EnableIRQ(DMA2_Channel6_IRQn);
	
	DMA2_Channel7->CPAR = (uint32_t)&LPUART1->RDR;
	DMA2_Channel7->CMAR = (uint32_t)dmabufread;
	DMA2_Channel7->CNDTR = (uint16_t)100;
	DMA2_CSELR->CSELR |= 0x04000000; //Channel 7
	DMA2_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
}
void LPUART1_SendWithDMA(const char *str, uint16_t len)
{
	for(uint16_t i=0;i<len;i++)
		dmabufwrite[i] = str[i];
	DMA2_Channel6->CCR &= ~DMA_CCR_EN;
	DMA2_Channel6->CNDTR = (uint16_t)len;
	DMA2_Channel6->CCR |= DMA_CCR_EN;
}
void DMA2_Channel6_IRQHandler(void)
{
	if((DMA2->ISR & DMA_ISR_TCIF6) != RESET)
	{
		DMA2->IFCR |= DMA_IFCR_CTCIF6;
		Led_OnOff(6, LedTog);
	}
}
void LPUART1_ReinitDMA(void)
{
	DMA2_Channel7->CCR &= ~DMA_CCR_EN;
	DMA2_Channel7->CNDTR = 100;
	DMA2_Channel7->CCR |= DMA_CCR_EN;
}
void Joy_Interrupt_Conf(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	GPIOE->MODER &= ~GPIO_MODER_MODER0;
	GPIOE->MODER &= ~GPIO_MODER_MODER1;
	GPIOE->MODER &= ~GPIO_MODER_MODER2;
	GPIOE->MODER &= ~GPIO_MODER_MODER3;
	GPIOE->MODER &= ~GPIO_MODER_MODER15;
	
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PE;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PE;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PE;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PE;
	
	EXTI->IMR1 |= EXTI_IMR1_IM0;
	EXTI->IMR1 |= EXTI_IMR1_IM1;
	EXTI->IMR1 |= EXTI_IMR1_IM2;
	EXTI->IMR1 |= EXTI_IMR1_IM3;
	EXTI->IMR1 |= EXTI_IMR1_IM15;
	
	EXTI->FTSR1 |= EXTI_FTSR1_FT0;
	EXTI->FTSR1 |= EXTI_FTSR1_FT1;
	EXTI->FTSR1 |= EXTI_FTSR1_FT2;
	EXTI->FTSR1 |= EXTI_FTSR1_FT3;
	EXTI->FTSR1 |= EXTI_FTSR1_FT15;
	
	EXTI->RTSR1 |= EXTI_RTSR1_RT0;
	EXTI->RTSR1 |= EXTI_RTSR1_RT1;
	EXTI->RTSR1 |= EXTI_RTSR1_RT2;
	EXTI->RTSR1 |= EXTI_RTSR1_RT3;
	EXTI->RTSR1 |= EXTI_RTSR1_RT15;
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void EXTI0_IRQHandler(void)
{
	if((GPIOE->IDR & GPIO_IDR_ID0) == RESET)
		Led_OnOff(0, LedOn);
	else
		Led_OnOff(0, LedOff);
	
	led7seg_value++;
	EXTI->PR1 |= EXTI_PR1_PIF0;
}
void EXTI1_IRQHandler(void)
{
	if((GPIOE->IDR & GPIO_IDR_ID1) == RESET)
		Led_OnOff(1, LedOn);
	else
		Led_OnOff(1, LedOff);
	
	led7seg_value++;
	EXTI->PR1 |= EXTI_PR1_PIF1;
}
void EXTI2_IRQHandler(void)
{
	if((GPIOE->IDR & GPIO_IDR_ID2) == RESET)
		Led_OnOff(2, LedOn);
	else
		Led_OnOff(2, LedOff);
	
	led7seg_value++;
	EXTI->PR1 |= EXTI_PR1_PIF2;
}
void EXTI3_IRQHandler(void)
{
	if((GPIOE->IDR & GPIO_IDR_ID3) == RESET)
		Led_OnOff(3, LedOn);
	else
		Led_OnOff(3, LedOff);
	
	led7seg_value++;
	EXTI->PR1 |= EXTI_PR1_PIF3;
}
void EXTI15_10_IRQHandler(void)
{
	if((EXTI->PR1 & EXTI_PR1_PIF15) != RESET)
	{
		if((GPIOE->IDR & GPIO_IDR_ID15) == RESET)
			Led_OnOff(7, LedOn);
		else
			Led_OnOff(7, LedOff);
		
		led7seg_value++;
		EXTI->PR1 |= EXTI_PR1_PIF15;
	}
}
void HCSR04_Conf(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	
	GPIOF->MODER &= ~GPIO_MODER_MODER3;
	GPIOF->MODER |= GPIO_MODER_MODER3_0;
	GPIOF->MODER &= ~GPIO_MODER_MODER4;
	
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PF;
	EXTI->IMR1 |= EXTI_IMR1_IM4;
	EXTI->FTSR1 |= EXTI_FTSR1_FT4;
	EXTI->RTSR1 |= EXTI_RTSR1_RT4;
	NVIC_EnableIRQ(EXTI4_IRQn);
	
	HCSR04_SendTriger();
}
void HCSR04_SendTriger(void)
{
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->PSC = 4-1;
	TIM7->ARR = 10-1;
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
	GPIOF->ODR |= GPIO_ODR_OD3;
}
void HCSR04_ReadEcho(void)
{
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->CNT = 0;
	TIM7->PSC = 4-1;
	TIM7->ARR = 0xffff;
	TIM7->DIER &= ~TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
}

void EXTI4_IRQHandler(void)
{
	if((GPIOF->IDR & GPIO_IDR_ID4) != RESET)
	{
		HCSR04_ReadEcho();
	}
	else
	{
		HCSR04_Time = TIM7->CNT;
		HCSR04_Distancemm = (double)HCSR04_Time / 2.0 * 0.340;
		led7seg_value = HCSR04_Distancemm;
		HCSR04_SendTriger();
	}
	EXTI->PR1 |= EXTI_PR1_PIF4;
}
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		TIM7->CR1 &= ~TIM_CR1_CEN;
		GPIOF->ODR &= ~GPIO_ODR_OD3;
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
void LedRGB_PwmConf(void)
{
	//Red GPIOD.13, Green GPIOB.8, Blue GPIOD.12
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~GPIO_MODER_MODER8;
	GPIOB->MODER |= GPIO_MODER_MODER8_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_1;
	GPIOB->AFR[1] |= 0x00000002;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	GPIOD->MODER &= ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13;
	GPIOD->MODER |= GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD12_1 | GPIO_PUPDR_PUPD13_1;
	GPIOD->AFR[1] |= 0x00220000;
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	TIM4->PSC = 4-1;
	TIM4->ARR = 256-1;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM4->CR1 |= TIM_CR1_CEN;
}
void LedRGB_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
	TIM4->CCR1 = blue;
	TIM4->CCR2 = red;
	TIM4->CCR3 = green;
}
void Servodrive_Conf(void)
{
	//Servodrive GPIOA.5
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_1;
	GPIOA->AFR[0] |= 0x00100000;

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	TIM2->PSC = 4-1;
	TIM2->ARR = 20000-1;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM Mode
	TIM2->CCR1 = 1500; //7.5 %; 3% = 600imp; 12% = 2400imp
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CR1 |= TIM_CR1_CEN;
}
void Serwodrive_SetPosition(double angle)
{
	if(angle < -90.0 || angle > 90.0)
		return;
	
	TIM2->CCR1 = 1500 + angle * 10;
}
void DCMotor_Conf(void)
{
	//DC MOTOR GPIOC.10, GPIOC.11, GPIOA.8
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODER10 & ~GPIO_MODER_MODER11;
	GPIOC->MODER |= GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD8_0;
	GPIOA->AFR[1] |= 0x00000001;
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->PSC	= 4-1;
	TIM1->ARR	= 1000-1;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCR1 = 0;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CR1	|= TIM_CR1_CEN;
	
	GPIOC->ODR |= GPIO_ODR_OD11; //enable
	GPIOC->ODR |= GPIO_ODR_OD10; //enable
}
void Encoder_Conf(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7;
	GPIOC->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
	GPIOC->AFR[0] |= 0x22000000;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	TIM3->ARR = 0xffff;
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM3->CR1 |= TIM_CR1_CEN;
}












