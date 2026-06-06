#ifndef MCU_CMIC_GD32F470VET6_H
#define MCU_CMIC_GD32F470VET6_H

#include "gd32f4xx.h"
#include "gd32f4xx_dma.h"
#include "systick.h"
#include "usart_app.h"

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************************************/
/* Wakeup key */
#define WK_UP_PORT                GPIOA
#define WK_UP_PIN                 GPIO_PIN_0
#define WK_UP_CLK_PORT            RCU_GPIOA

/***************************************************************************************************************/
/* V2 USART0 pin map */
#define USART0_RX_PORT            GPIOA
#define USART0_RX_PIN             GPIO_PIN_10
#define USART0_TX_PORT            GPIOA
#define USART0_TX_PIN             GPIO_PIN_9

#define USART1_RX_PORT            GPIOD
#define USART1_RX_PIN             GPIO_PIN_6
#define USART1_TX_PORT            GPIOD
#define USART1_TX_PIN             GPIO_PIN_5

#define AF_USART0                 GPIO_AF_7
#define AF_USART1                 GPIO_AF_7

/***************************************************************************************************************/
/* USART0 debug console */
#define DEBUG_USART               (USART0)
#define USART0_RDATA_ADDRESS      ((uint32_t)&USART_DATA(USART0))
#define USART1_RDATA_ADDRESS      ((uint32_t)&USART_DATA(USART1))

#define USART0_RX_DMA_PERIPH      DMA1
#define USART0_RX_DMA_CHANNEL     DMA_CH5
#define USART0_RX_DMA_SUBPERI     DMA_SUBPERI4

#define USART1_RX_DMA_PERIPH      DMA0
#define USART1_RX_DMA_CHANNEL     DMA_CH5
#define USART1_RX_DMA_SUBPERI     DMA_SUBPERI4

#define USART_RS_PORT             USART1_TX_PORT
#define USART_RS_PORT_RCU         RCU_GPIOD
#define USART_RS_RX_PORT          USART1_RX_PORT
#define USART_RS_RX_PIN           USART1_RX_PIN
#define USART_RS_TX_PORT          USART1_TX_PORT
#define USART_RS_TX_PIN           USART1_TX_PIN
#define USART_RS_RX               USART_RS_RX_PIN
#define USART_RS_TX               USART_RS_TX_PIN

#define RS485_CS_PORT             GPIOE
#define RS485_CS_PORT_RCU         RCU_GPIOE
#define RS485_CS_PIN              GPIO_PIN_8
#define RS485_CS_SET(x)           do { if(x) GPIO_BOP(RS485_CS_PORT) = RS485_CS_PIN; else GPIO_BC(RS485_CS_PORT) = RS485_CS_PIN; } while(0)

#define DMA_RS_USART              USART1_RX_DMA_PERIPH
#define DMA_RS_USART_CHANNEL_RX   USART1_RX_DMA_CHANNEL
#define DMA_RS_USART_SUBPERI_RX   USART1_RX_DMA_SUBPERI
#define RS232_RS485_USART         (USART1)
#define USART_RS_RDATA_ADDRESS    USART1_RDATA_ADDRESS
#define USART_RS_RCU              RCU_USART1

#define USART0_PORT               USART0_TX_PORT
#define USART0_CLK_PORT           RCU_GPIOA
#define USART0_TX                 USART0_TX_PIN
#define USART0_RX                 USART0_RX_PIN

/* Compatibility aliases used by older bootloader code. */
#define USART_PORT                USART0_PORT
#define USARTI_CLK_PORT           USART0_CLK_PORT
#define USART_TX                  USART0_TX
#define USART_RX                  USART0_RX

void bsp_usart0_init(void);
void bsp_usart1_init(void);
void bsp_usart_init(void);
void bsp_wkup_key_exti_init(void);
void bsp_enter_deepsleep(void);

/***************************************************************************************************************/

#ifdef __cplusplus
  }
#endif

#endif /* MCU_CMIC_GD32F470VET6_H */
