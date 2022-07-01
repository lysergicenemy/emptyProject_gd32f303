/*!
    \file  main.c
    \brief systick LED demo
    
    \version 2021-03-23, V2.0.0, demo for GD32F30x
*/


#include "gd32f30x.h"
#include "systick.h"

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* configure systick */
    systick_config();
    
    /* enable the LEDs GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);

    /* configure LED1 LED2 GPIO port */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); // SWD/CJTAG only
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    
    /* reset LED1 LED2 GPIO pin */
    gpio_bit_reset(GPIOC, GPIO_PIN_13);

    while(1){
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(750);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(250);
    }
}
