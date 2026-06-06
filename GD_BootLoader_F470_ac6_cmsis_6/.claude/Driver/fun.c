#include "fun.h"

/* USART DMA RX buffer. */
uint8_t usart0_rxbuffer[512];
uint8_t usart1_rxbuffer[256];

void bsp_wkup_key_exti_init(void)
{
    rcu_periph_clock_enable(WK_UP_CLK_PORT);
    rcu_periph_clock_enable(RCU_SYSCFG);

    gpio_mode_set(WK_UP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, WK_UP_PIN);

    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN0);
    exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_0);
    nvic_irq_enable(EXTI0_IRQn, 1U, 0U);
}

static void bsp_usart_disable_for_deepsleep(void)
{
    usart_interrupt_disable(USART0, USART_INT_IDLE);
    nvic_irq_disable(USART0_IRQn);
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_DISABLE);
    dma_channel_disable(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL);
    usart_disable(USART0);

    usart_interrupt_disable(USART1, USART_INT_IDLE);
    nvic_irq_disable(USART1_IRQn);
    usart_dma_receive_config(USART1, USART_RECEIVE_DMA_DISABLE);
    dma_channel_disable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
    usart_disable(USART1);
}

static void bsp_gpio_enter_deepsleep_state(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RS485_CS_PORT_RCU);

    gpio_mode_set(USART0_TX_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, USART0_TX_PIN);
    gpio_mode_set(USART0_RX_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, USART0_RX_PIN);
    gpio_mode_set(USART1_TX_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, USART1_TX_PIN);
    gpio_mode_set(USART1_RX_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, USART1_RX_PIN);
    gpio_mode_set(RS485_CS_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, RS485_CS_PIN);
    gpio_mode_set(WK_UP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, WK_UP_PIN);
}

static void bsp_deepsleep_reinit_after_wakeup(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    systick_config();
    bsp_usart_init();
}

void bsp_enter_deepsleep(void)
{
    rcu_periph_clock_enable(RCU_PMU);

    __disable_irq();

    bsp_usart_disable_for_deepsleep();
    bsp_gpio_enter_deepsleep_state();
    bsp_wkup_key_exti_init();

    pmu_flag_clear(PMU_FLAG_RESET_WAKEUP);
    pmu_flag_clear(PMU_FLAG_RESET_STANDBY);

    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    __enable_irq();

    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, PMU_LOWDRIVER_ENABLE, WFI_CMD);

    bsp_deepsleep_reinit_after_wakeup();
}

/*!
    \brief      configure USART0
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_usart0_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;
    
    rcu_periph_clock_enable(RCU_DMA1);
    
    dma_deinit(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL);
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr = (uint32_t)usart0_rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number = sizeof(usart0_rxbuffer);
    dma_init_struct.periph_addr = USART0_RDATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL, &dma_init_struct);
    
    dma_circulation_disable(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL);
    dma_channel_subperipheral_select(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL, USART0_RX_DMA_SUBPERI);
    dma_channel_enable(USART0_RX_DMA_PERIPH, USART0_RX_DMA_CHANNEL);
    
    /* enable GPIO clock */
    rcu_periph_clock_enable(USART0_CLK_PORT);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);
    
    /* connect port to USARTx_Tx */
    gpio_af_set(USART0_PORT, AF_USART0, USART0_TX);

    /* connect port to USARTx_Rx */
    gpio_af_set(USART0_PORT, AF_USART0, USART0_RX);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(USART0_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_TX);
    gpio_output_options_set(USART0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART0_TX);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(USART0_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_RX);
    gpio_output_options_set(USART0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART0_RX);

    /* configure USART */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
    usart_enable(USART0);
    
    nvic_irq_enable(USART0_IRQn, 0, 0);
    
    usart_interrupt_enable(USART0, USART_INT_IDLE);
}

void bsp_usart_init(void)
{
    bsp_usart0_init();
    bsp_usart1_init();
}

void bsp_usart1_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(USART_RS_PORT_RCU);
    rcu_periph_clock_enable(RS485_CS_PORT_RCU);
    rcu_periph_clock_enable(USART_RS_RCU);

    dma_deinit(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr = (uint32_t)usart1_rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number = sizeof(usart1_rxbuffer);
    dma_init_struct.periph_addr = USART1_RDATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL, &dma_init_struct);

    dma_circulation_disable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
    dma_channel_subperipheral_select(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL, USART1_RX_DMA_SUBPERI);
    dma_channel_enable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);

    gpio_af_set(USART_RS_PORT, AF_USART1, USART_RS_TX_PIN | USART_RS_RX_PIN);
    gpio_mode_set(USART_RS_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART_RS_TX_PIN | USART_RS_RX_PIN);
    gpio_output_options_set(USART_RS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, USART_RS_TX_PIN | USART_RS_RX_PIN);

    gpio_mode_set(RS485_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RS485_CS_PIN);
    gpio_output_options_set(RS485_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, RS485_CS_PIN);
    RS485_CS_SET(0);

    usart_deinit(RS232_RS485_USART);
    usart_baudrate_set(RS232_RS485_USART, 115200U);
    usart_receive_config(RS232_RS485_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(RS232_RS485_USART, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(RS232_RS485_USART, USART_RECEIVE_DMA_ENABLE);
    usart_enable(RS232_RS485_USART);
}
