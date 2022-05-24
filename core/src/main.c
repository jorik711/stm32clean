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
#include "led.h"

#define PLL_M 	    4
#define PLL_N 	    180
// #define PLL_P 	2  
#define SYSCLOCK    180000000UL

#define TIM2_PSC     8999
#define TIM2_ARR     9999

#define TIM3_PSC     8999
#define TIM3_ARR     49

__IO uint32_t systick_cnt = 0;
static __IO uint32_t flag = 0;
__IO uint32_t tim2_cnt = 0;

uint32_t buttonState = 0;
uint32_t debounceCount = 0;
uint32_t result = 0;
uint32_t flagDebounce = 0;
/******************** dynamic indication ***************/
__IO uint32_t tim3_cnt = 1; /* выбор зардяда дисплея */
extern uint8_t num1, num2, num3, num4; /* номер индикатора */
extern uint16_t showNumber; /* выводимое число */
uint32_t counter; /* переменная для счета */
/******************** init func ************************/
void system_clock_init();
void led_gpio_init();
void systick_init();
void button_gpio_init();
static void tim2_init();
static void tim3_init();
/******************** user func ***********************/
void toggle_led();
void delay_ms(uint32_t delay);
uint32_t debounce_handler(uint32_t buttonState);

/***************** main func *******************************/
int main(void)
{
    system_clock_init();
    led_gpio_init();
    systick_init();
    button_gpio_init();
    /*************** timer2 init and start ******************/
    tim2_init();
    /* update interrupt enable */
    TIM2->DIER |= TIM_DIER_UIE;
    /* start timer */
    TIM2->CR1 |= TIM_CR1_CEN;
    /*************** timer3 init and start ******************/
    tim3_init();
    /* update interrupt enable */
    TIM3->DIER |= TIM_DIER_UIE;
    /* start timer */
    TIM3->CR1 |= TIM_CR1_CEN;
    while (1)
    {
        for (counter = 0; counter < 10000; ++counter ) {
            ledstream(counter);
            delay_ms(500);
        }
        // buttonState = GPIOA->IDR & GPIO_IDR_IDR_0;
        // result = debounce_handler(buttonState);
        // if (result == 1 && flagDebounce == 0) {
        //     flagDebounce = 1;
        //     GPIOG->ODR ^= GPIO_ODR_ODR_14;
        // }
        // else if (result == 0 && flagDebounce == 1) {
        //     flagDebounce = 0;
        // }
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
    //PWR->CR |=PWR_CR_ODEN;
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
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM) ;
    RCC->PLLCFGR |= (PLL_M << RCC_PLLCFGR_PLLM_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); 
    RCC->PLLCFGR |= (PLL_N << RCC_PLLCFGR_PLLN_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
    //RCC->PLLCFGR |= (PLL_P << RCC_PLLCFGR_PLLP_Pos);

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;
    /******************* Enable the PLL *****************************************/
    RCC->CR |= RCC_CR_PLLON;
    /*  */
    while(!(RCC->CR & RCC_CR_PLLRDY)) {}
    /******* Select the Clock Source and wait for it to be set ******************/
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
    SystemCoreClockUpdate();
}
void led_gpio_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  /* Enable the GPIOG clock */
    /********** PG13 ***********/
    GPIOG->MODER |= (GPIO_MODER_MODER13_0);  // pin PG13(bits 27:26) as Output (01)
    GPIOG->OTYPER &= ~(GPIO_OTYPER_OT13);  // bit 13=0 --> Output push pull
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED13_1);  // Pin PG13 (bit  s 27:26) as Fast Speed (1:0)
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD13);  // Pin PG13 (bits 27:26) are 0:0 --> no pull up or pulldown
    /*********** PG14 ************/
    GPIOG->MODER |= (GPIO_MODER_MODER14_0);  // pin PG13(bits 29:28) as Output (01)
    GPIOG->OTYPER &= ~(GPIO_OTYPER_OT14);  // bit 14 = 0 --> Output push pull
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED14_1);  // Pin PG14 (bit  s 29:28) as Fast Speed (1:0)
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD14);  // Pin PG14 (bits 29:28) are 0:0 --> no pull up or pulldown
    /********** dynamic indication *************************/
    /* pins PG0 - PG11 as Output (01) */
    GPIOG->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 |
                    GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 |
                    GPIO_MODER_MODER10_0 |GPIO_MODER_MODER11_0);
    /* pins PG0 - PG11 Output push pull (00) */ 
    GPIOG->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 |  GPIO_OTYPER_OT5 |
                GPIO_OTYPER_OT6 |  GPIO_OTYPER_OT7 |  GPIO_OTYPER_OT8 |  GPIO_OTYPER_OT9 |  GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
    /* pins PG0 - PG11 as fast speed (10) */
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1 | GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED4_1 |
                    GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED6_1 | GPIO_OSPEEDR_OSPEED7_1 | GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED9_1 |
                    GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED11_1);
    /* pins PG0 - PG11 no pull up or pulldown (00) */
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD0 |  GPIO_PUPDR_PUPD1 |  GPIO_PUPDR_PUPD2 |  GPIO_PUPDR_PUPD3 |  GPIO_PUPDR_PUPD4 |
                    GPIO_PUPDR_PUPD5 |  GPIO_PUPDR_PUPD6 |  GPIO_PUPDR_PUPD7 |  GPIO_PUPDR_PUPD8 |  GPIO_PUPDR_PUPD9 |
                    GPIO_PUPDR_PUPD10 |  GPIO_PUPDR_PUPD11);
    /* pins PG0 - PG11 reset */
    GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 |
                    GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7 | GPIO_ODR_ODR_8 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10 | GPIO_ODR_ODR_11);
}
void toggle_led() {
    GPIOG->BSRR |= GPIO_BSRR_BS13;
    //GPIOG->BSRR |= GPIO_BSRR_BR14;
	delay_ms(1000);
    //GPIOG->BSRR |= GPIO_BSRR_BS14;
    GPIOG->BSRR |= GPIO_BSRR_BR13;
	delay_ms(1000);
}
void systick_init() {
    SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk;
    SysTick->LOAD = SYSCLOCK / 1000 - 1;
    SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
void SysTick_Handler(void) {
    if (systick_cnt > 0) {
        systick_cnt--;
    } 
}
void delay_ms(uint32_t delay) {
    SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
    SysTick->VAL = SYSCLOCK / 1000 - 1; /* SystemCoreClock */
    systick_cnt = delay;
    while (systick_cnt) {
        /* empty cycle */
    }
}
/* PA0 button stm32f4 - discovery */
void button_gpio_init(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable the GPIOA clock
    GPIOA->MODER &= ~(GPIO_MODER_MODER0); /* set bits as input (0:0) */
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1; /* (1:0) pull up */

    /* Включить прерывание */
    NVIC_EnableIRQ(EXTI0_IRQn);
    /* Разрешить прерывание */
    EXTI->IMR |= EXTI_IMR_IM0;
    /* Выбор прерывани по нарастающему фронту */
    EXTI->RTSR |= EXTI_RTSR_TR0;
    /* Запрещаем по спадающему фронту */
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
}
void EXTI0_IRQHandler(void) {
    /* сбрасываем флаг прерывания  */
    EXTI->PR |= EXTI_PR_PR0;
}
uint32_t debounce_handler(uint32_t buttonState) {

    if (buttonState != 0) {  // если кнопка нажата
        if (debounceCount < 1000) { // если счетчик дребезга меньше "n"
            debounceCount++;
            return 0;
        } else {
            return 1; // кнопка нажата;
        } 
    } else {
        if (buttonState == 0) { // кнопка отжата 
            if (debounceCount > 0) {
                debounceCount--;
                return 1;
            } else {
                return 0; // кнопка отжата
            }
        }
    }
}
static  void tim2_init(void){
    /* enable clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    /* enable interrupt */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->PSC = TIM2_PSC;
    TIM2->ARR = TIM2_ARR;

}
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        GPIOG->ODR ^= GPIO_ODR_ODR_13;
    }
}
static void tim3_init() {
    /* enable clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    /* enable interrupt */
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->PSC = TIM3_PSC;
    TIM3->ARR = TIM3_ARR;
}
void TIM3_IRQHandler() {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        if (tim3_cnt == 1) {
            GPIOG->ODR |= GPIO_ODR_ODR_0;
            GPIOG->ODR &= ~(GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3);
            segchar(num1);
            GPIOG->ODR &= ~(GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3);
        }
        if (tim3_cnt == 2) {
            GPIOG->ODR |= GPIO_ODR_ODR_1;
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3);
            segchar(num2);
            if(showNumber < 10) {
                GPIOG->ODR &= ~(GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7 | 
                                GPIO_ODR_ODR_8 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10 | GPIO_ODR_ODR_11);
            }
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3);
        }
        if (tim3_cnt == 3) {
            GPIOG->ODR |= GPIO_ODR_ODR_2;
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_3);
            segchar(num3);
            if(showNumber < 100) {
                GPIOG->ODR &= ~(GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7 | 
                                GPIO_ODR_ODR_8 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10 | GPIO_ODR_ODR_11);
            }
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_3);
        }
        if (tim3_cnt == 4) {
            GPIOG->ODR |= GPIO_ODR_ODR_3;
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2);
            segchar(num4);
            if(showNumber < 1000) {
                GPIOG->ODR &= ~(GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7 | 
                                GPIO_ODR_ODR_8 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10 | GPIO_ODR_ODR_11);
            }
            GPIOG->ODR &= ~(GPIO_ODR_ODR_0 | GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2);
        }
        tim3_cnt++;
        if (tim3_cnt > 4) {
            tim3_cnt = 0;
        }
    }
}