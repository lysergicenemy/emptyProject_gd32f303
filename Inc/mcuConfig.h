/*!
    \file  mcuConfig.h
    \brief System and peripherial configuration
*/

#include "gd32f30x.h"
#include "systick.h"

#ifndef MCUCONFIG_H
#define MCUCONFIG_H


#define FMC_WRITE_START_ADDR ((uint32_t)0x0803F800U)

//
// MCU cycles counter (programm execution time capture)
//
struct cycCntr_s
{
    uint32_t startPoint;
    uint32_t stopPoint;
    uint32_t cpuTime;    // time in clock cycles
    uint32_t cpuTimeMax; // max time in clock cycles
    uint32_t cpuTimeMin; // min time in clock cycles
    uint8_t ignoreFlag;  // ignore data if counter has been reset
};
typedef volatile struct cycCntr_s cycCntr_t;


//
// System data struct
//
struct system_s
{
    cycCntr_t cc;
    volatile uint8_t uart_tx_buff[4];
    volatile uint8_t uart_rx_buff[4];
    uint32_t uart_baud;
    void (*cc_start)(cycCntr_t *p);
    void (*cc_stop)(cycCntr_t *p);
};
typedef struct system_s system_t;

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void rcu_config(void)
{
    /* enable GPIO clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);
    /* enable DMA clock*/
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);
    /* Enable cycles counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

/*!
    \brief      configure the NVIC
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    /* UART_DMA_RX ISR */
    nvic_irq_enable(DMA0_Channel4_IRQn, 2, 0);
    /* UART_DMA_TX ISR */
    nvic_irq_enable(DMA0_Channel3_IRQn, 2, 0);
}

/*!
    \brief      configure the GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void gpio_config(void)
{
    /* configure LED1 GPIO port */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
    /* Remap */
    gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);
    gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); // Disable JTAG pins, enable SWD only
    /* GPIO */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    /* connect pins to USART */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);        // USART0 TX
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10); // USART0 RX
}


/*!
    \brief      configure the DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void uart_dma_config(system_t *p)
{
    dma_parameter_struct dma_init_struct;
    /* initialize USART */

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, p->uart_baud);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* deinitialize DMA channel3(USART0 tx) */
    dma_deinit(DMA0, DMA_CH3);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)p->uart_tx_buff;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = sizeof(p->uart_tx_buff);
    dma_init_struct.periph_addr = ((uint32_t)&USART_DATA(USART0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    /* deinitialize DMA channel4 (USART0 rx) */
    dma_deinit(DMA0, DMA_CH4);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)p->uart_rx_buff;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = sizeof(p->uart_rx_buff);
    dma_init_struct.periph_addr = ((uint32_t)&USART_DATA(USART0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    dma_circulation_disable(DMA0, DMA_CH4);
    /* USART DMA enable for transmission and reception */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
    dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF);
    /* enable DMA channel3 */
    dma_channel_enable(DMA0, DMA_CH3);
    dma_channel_enable(DMA0, DMA_CH4);
}


/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void fmc_erase_page(uint32_t page_address)
{

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    fmc_page_erase(page_address);
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void fmc_program(uint32_t address, void *data, uint32_t data_size)
{
    uint32_t *ptr = (uint32_t *)data;
    /* unlock the flash program/erase controller */
    fmc_unlock();
    for (uint32_t i = 0; i < data_size; i += 4)
    {
        /* program flash */
        fmc_word_program((address + i), *(ptr++));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void fmc_read(uint32_t address, void *data, uint32_t data_size)
{
    uint32_t *ptr= (uint32_t *)data;

      for (uint32_t i = 0; i < data_size; i += 4)
    {
        *(ptr++) = *(uint32_t *)(address + i);
    }
}


/*!
    \brief      send new packet via UART using DMA
    \param[in]  buff: data buffer
    \param[out] none
    \retval     none
*/
static inline void usart_dma_startTx(uint32_t buff, uint32_t size)
{
    /* re-enable transmition */
    usart_transmit_config(USART0, USART_TRANSMIT_DISABLE);
    dma_memory_address_config(DMA0, DMA_CH3, buff);
    dma_transfer_number_config(DMA0, DMA_CH3, size);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    dma_channel_enable(DMA0, DMA_CH3);
}

/*!
    \brief      receive new packet via UART using DMA
    \param[in]  buff: data buffer
    \param[out] none
    \retval     none
*/
static inline void usart_dma_startRx(uint32_t buff)
{
    /* re-enable transmition */
    usart_receive_config(USART0, USART_RECEIVE_DISABLE);
    dma_memory_address_config(DMA0, DMA_CH4, buff);
    dma_transfer_number_config(DMA0, DMA_CH4, sizeof(buff));
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    dma_channel_enable(DMA0, DMA_CH4);
}

/*!
    \brief      Capture execution time of code section in CPU cycles. start point
    \param[in]  p: pointer to cycCntr data
    \param[out] none
    \retval     none
*/
static inline void cc_captureTimeStart(cycCntr_t *p)
{
    p->startPoint = DWT->CYCCNT;
}

/*!
    \brief      Capture execution time of code section in CPU cycles. stop point
    \param[in]  p: pointer to cycCntr data
    \param[out] none
    \retval     none
*/
static inline void cc_captureTimeStop(cycCntr_t *p)
{
    p->stopPoint = DWT->CYCCNT;
    p->ignoreFlag = (p->startPoint > p->stopPoint) ? 1 : 0;
    if (p->ignoreFlag == 0)
    {
        p->cpuTime = (p->stopPoint - p->startPoint);
        p->cpuTimeMax = (p->cpuTime > p->cpuTimeMax) ? p->cpuTime : p->cpuTimeMax;
        p->cpuTimeMin = (p->cpuTime < p->cpuTimeMin && p->cpuTime != 0) ? p->cpuTime : p->cpuTimeMin;
    }
}

/*!
    \brief      config all system modules
    \param[in]  p: pointer to system data struct
    \param[out] none
    \retval     none
*/
static inline void systemStart(system_t *p)
{
    /* link function pointers */
    p->cc_start = cc_captureTimeStart;
    p->cc_stop = cc_captureTimeStop;
    /* enable peripherial */
    rcu_config();
    nvic_config();
    systick_config();
    gpio_config();
    uart_dma_config(p);
    /* start peripherial */
    usart_dma_startRx((uint32_t)p->uart_rx_buff);
}

#endif /* MCUCONFIG_H */

