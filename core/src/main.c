/**
 * @file main.c
 * @author Egor Ryazanov (711jorik@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "stm32f4xx.h"

#define PLL_M 	4
#define PLL_N 	180
#define PLL_P 	2  

void system_clock_init();
void gpioa_init();
void toggle_led();

int main(void)
{
    system_clock_init();
    gpioa_init();
    while (1)
    {
        toggle_led();
    }
    return 0;
}

void system_clock_init() {
    /* Определяем внешний генератор */
    RCC->CR |= RCC_CR_HSEON;
    /* Ждем пока HSE стабилизируется. Определяется по флагу RCC_CR_HSERDY 
    (флаг готовности внешнего высокоскоростного тактирования) */
    while(!(RCC->CR & RCC_CR_HSERDY)) {} 
    /* настройки voltage regulator  */
    PWR->CR |=PWR_CR_VOS;
    /* включение режима power over drive */
    PWR->CR |=PWR_CR_ODEN;
    /************************* Configure flash *****************************/
    /* разрешение предварительной выборки инструкций */
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    /* разрешение кэша инструкций */
    FLASH->ACR |= FLASH_ACR_ICEN;
    /*  разрешение кэша данных */
    FLASH->ACR |= FLASH_ACR_DCEN;
    /* Эти биты представляют соотношение между периодом тактов CPU и 
    временем доступа к памяти Flash (количество циклов ожидания wait state). */
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    /***************** Configure the PRESCALARS HCLK, PCLK1, PCLK2 ***********/
    /* AHB prescalar */
    RCC->CFGR &= ~(RCC_CFGR_HPRE);
    /* APB1 prescalar */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; 
    /* APB2 prescalar */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; 
    /****************** Configure the MAIN PLL (ФАПЧ) *******************************/
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Pos) ;
    RCC->PLLCFGR |= (PLL_M << RCC_PLLCFGR_PLLM_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_Pos); 
    RCC->PLLCFGR |= (PLL_N << RCC_PLLCFGR_PLLN_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= (PLL_P << RCC_PLLCFGR_PLLP_Pos);

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;
    /******************* Enable the PLL *****************************************/
    RCC->CR |= RCC_CR_PLLON;
    /*  */
    while(!(RCC->CR & RCC_CR_PLLRDY)) {}
    /******* Select the Clock Source and wait for it to be set ******************/
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);

}

void gpioa_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  // Enable the GPIOG clock
    /********** PG13 ***********/
    GPIOG->MODER |= (GPIO_MODER_MODER13_0);  // pin PG13(bits 27:26) as Output (01)
    GPIOG->OTYPER &= ~(GPIO_OTYPER_OT13);  // bit 13=0 --> Output push pull
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED13_1);  // Pin PG13 (bit  s 27:26) as Fast Speed (1:0)
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD13);  // Pin PG13 (bits 27:26) are 0:0 --> no pull up or pulldown
    /*********** PG14 ************/
    GPIOG->MODER |= (GPIO_MODER_MODER14_0);  // pin PG13(bits 29:28) as Output (01)
    GPIOG->OTYPER &= ~(GPIO_OTYPER_OT14);  // bit 14 = 0 --> Output push pull
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED14_1);  // Pin PG14 (bit  s 27:26) as Fast Speed (1:0)
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD14);  // Pin PG14 (bits 29:28) are 0:0 --> no pull up or pulldown
}

void toggle_led() {

	GPIOG->ODR |= GPIO_IDR_ID13;
    GPIOG->ODR = ~(GPIO_IDR_ID14);
	for(int i = 500000; i > 0; --i) {}
	GPIOG->ODR &= ~(GPIO_IDR_ID13);
    GPIOG->ODR |= GPIO_IDR_ID14;
	for(int i = 500000; i > 0; --i) {}
}
