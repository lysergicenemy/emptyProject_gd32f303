/*!
    \file  main.c
    \brief systick LED demo

    \version 2021-03-23, V2.0.0, demo for GD32F30x
*/

#include "mcuConfig.h"

system_t mcu;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* config and start peripheral */
    systemStart(&mcu);
    /* reset LED1 LED2 GPIO pin */
    gpio_bit_reset(GPIOC, GPIO_PIN_13);

    while (1)
    {
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(750);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(250);
    }
}
