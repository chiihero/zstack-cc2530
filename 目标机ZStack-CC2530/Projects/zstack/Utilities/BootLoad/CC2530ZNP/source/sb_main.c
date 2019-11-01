/**************************************************************************************************
  Filename:       sb_main.c
  Revised:        $Date: 2010-08-27 09:19:41 -0700 (Fri, 27 Aug 2010) $
  Revision:       $Revision: 23555 $

  Description:    This module contains the main functionality of a Boot Loader for CC2530ZNP.
                  It is a minimal subset of functionality from ZMain.c, OnBoard.c and various
                  _hal_X.c modules for the CC2530ZNP build.


  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "comdef.h"
#include "hal_board_cfg.h"
#include "hal_adc.h"
#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "sb_exec.h"
#include "sb_main.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// The IAR C-Stack initializer value: 0xCD.
#if !defined SB_STACK_VALUE
#define SB_STACK_VALUE  0xCD
#endif
// Not zero and not SB_STACK_VALUE.
#if !defined SB_MAGIC_VALUE
#define SB_MAGIC_VALUE  0xF5
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

halDMADesc_t dmaCh0;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ISR's implemented in the boot loader must be able to quickly determine whether to jump to the
 * boot code handlers or the run code handlers. So mark the bottom of the C call stack space with
 * a special value. Since the boot code linker file starts the C call stack space one byte higher
 * than the run code, the IAR generated initialization code does not initialize this byte;
 * but the boot code does - marking it with the magic value.
 */
#pragma location="SB_MAGIC_SPACE"
volatile __no_init uint8 magicByte;
static uint8 znpCfg1;
static uint8 spiPoll;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void vddWait(uint8 vdd);
static void sblInit(void);
static void sblExec(void);
static uint8 sblWait(void);

// Saving code space by not using the _hal_uart.c API and directly including the low level drivers.
#include "_hal_uart_isr.c"
#include "_hal_uart_spi.c"

/**************************************************************************************************
 * @fn          main
 *
 * @brief       ISR for the reset vector.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void main(void)
{
  sblInit();

  if (sbImgValid())
  {
    if (sblWait())
    {
      if (znpCfg1 == ZNP_CFG1_SPI)
      {
        HalUARTUnInitSPI();
      }
      else
      {
        HalUARTUnInitISR();
      }

      // Simulate a reset for the Application code by an absolute jump to location 0x2000.
      asm("LJMP 0x2000\n");
      HAL_SYSTEM_RESET();
    }
  }

  sblExec();
  HAL_SYSTEM_RESET();
}

/**************************************************************************************************
 * @fn          vddWait
 *
 * @brief       Loop waiting for 16 reads of the Vdd over the requested limit.
 *
 * input parameters
 *
 * @param       vdd - Vdd level to wait for.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void vddWait(uint8 vdd)
{
  uint8 cnt = 16;

  do {
    do {
      ADCCON3 = 0x0F;
      while (!(ADCCON1 & 0x80));
    } while (ADCH < vdd);
  } while (--cnt);
}

/**************************************************************************************************
 * @fn          sblInit
 *
 * @brief       Initialization for SBL.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void sblInit(void)
{
  HAL_BOARD_INIT();
  vddWait(VDD_MIN_RUN);
  magicByte = SB_MAGIC_VALUE;

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

#if defined CC2530_MK
  znpCfg1 = ZNP_CFG1_SPI;
#else
  znpCfg1 = P2_0;
#endif
  if (znpCfg1 == ZNP_CFG1_SPI)
  {
    SRDY_CLR();

    // Select general purpose on I/O pins.
    P0SEL &= ~(NP_RDYIn_BIT);      // P0.3 MRDY - GPIO
    P0SEL &= ~(NP_RDYOut_BIT);     // P0.4 SRDY - GPIO

    // Select GPIO direction.
    P0DIR &= ~NP_RDYIn_BIT;        // P0.3 MRDY - IN
    P0DIR |= NP_RDYOut_BIT;        // P0.4 SRDY - OUT

    P0INP &= ~NP_RDYIn_BIT;        // Pullup/down enable of MRDY input.
    P2INP &= ~BV(5);               // Pullup all P0 inputs.

    HalUARTInitSPI();
  }
  else
  {
    halUARTCfg_t uartConfig;

    HalUARTInitISR();
    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = HAL_UART_BR_115200;
    uartConfig.flowControl          = FALSE;
    uartConfig.flowControlThreshold = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.rx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.tx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.idleTimeout          = 0;  // CC2530 by #define - see hal_board_cfg.h
    uartConfig.intEnable            = TRUE;
    uartConfig.callBackFunc         = NULL;
    HalUARTOpenISR(&uartConfig);
  }
}

/**************************************************************************************************
 * @fn          sblExec
 *
 * @brief       Infinite SBL execute loop that returns upon receiving a code enable.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void sblExec(void)
{
  vddWait(VDD_MIN_NV);

  if (znpCfg1 == ZNP_CFG1_UART)
  {
    HAL_ENABLE_INTERRUPTS();
  }

  while (1)
  {
    if (sbExec())
    {
      break;
    }
  }

  if (znpCfg1 == ZNP_CFG1_UART)
  {
    uint32 dlyCnt;

    // Delay to allow the SB_ENABLE_CMD response to be flushed.
    for (dlyCnt = 0; dlyCnt < 0x40000; dlyCnt++)
    {
      asm("NOP");
    }
  }
}

/**************************************************************************************************
 * @fn          sblWait
 *
 * @brief       A timed-out wait loop that exits early upon receiving a force code/sbl byte.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE to run the code image, FALSE to run the SBL.
 **************************************************************************************************
 */
static uint8 sblWait(void)
{
  uint32 dlyCnt;
  uint8 rtrn = FALSE;

  if (znpCfg1 == ZNP_CFG1_SPI)
  {
    // Slave signals ready for read by setting its ready flag first.
    SRDY_SET();
    // Flag to sbRx() to poll for 1 Rx byte instead of blocking read until MRDY_CLR.
    spiPoll = TRUE;
    dlyCnt = 0x38;  // About 50 msecs.
  }
  else
  {
    HAL_ENABLE_INTERRUPTS();
    dlyCnt = 0x260000;  // About 1 minute.
  }

  while (1)
  {
    uint8 ch;

    if (sbRx(&ch, 1))
    {
      if (ch == SB_FORCE_BOOT)
      {
        break;
      }
      else if (ch == SB_FORCE_RUN)
      {
        dlyCnt = 0;
      }
    }

    if (dlyCnt-- == 0)
    {
      rtrn = TRUE;
      break;
    }
  }

  if (znpCfg1 == ZNP_CFG1_SPI)
  {
    // Master blocks waiting for slave to clear its ready flag before continuing.
    SRDY_CLR();
    // Flag to sbRx() to now block while reading Rx bytes until MRDY_CLR.
    spiPoll = FALSE;
  }
  else
  {
    HAL_DISABLE_INTERRUPTS();
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          sbRx
 *
 * @brief       Serial Boot loader read API that makes the low-level read according to RPC mode.
 *
 * input parameters
 *
 * @param       buf - Pointer to a buffer to fill with up to 'len' bytes.
 * @param       len - Maximum count of bytes to fill into the 'buf'.
 *
 *
 * output parameters
 *
 * None.
 *
 * @return      The count of the number of bytes filled into the 'buf'.
 **************************************************************************************************
 */
uint16 sbRx(uint8 *buf, uint16 len)
{
  if (znpCfg1 == ZNP_CFG1_UART)
  {
    return HalUARTReadISR(buf, len);
  }
  else
  {
    if (spiPoll)
    {
      if (URXxIF)
      {
        *buf = UxDBUF;
        return 1;
      }
      else
      {
        return 0;
      }
    }
    else
    {
      return HalUARTReadSPI(buf, len);
    }
  }
}

/**************************************************************************************************
 * @fn          sbTx
 *
 * @brief       Serial Boot loader write API that makes the low-level write according to RPC mode.
 *
 * input parameters
 *
 * @param       buf - Pointer to a buffer of 'len' bytes to write to the serial transport.
 * @param       len - Length in bytes of the 'buf'.
 *
 *
 * output parameters
 *
 * None.
 *
 * @return      The count of the number of bytes written from the 'buf'.
 **************************************************************************************************
 */
uint16 sbTx(uint8 *buf, uint16 len)
{
  if (znpCfg1 == ZNP_CFG1_UART)
  {
    return HalUARTWriteISR(buf, len);
  }
  else
  {
    return HalUARTWriteSPI(buf, len);
  }
}

/**************************************************************************************************
 * @fn          URXx/UTXx_VECTOR
 *
 * @brief       Intercept the USART0 ISR's here and consume locally if in boot loader, or forward
 *              to a valid run-code image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
HAL_ISR_FUNCTION( halUart0RxIsr, URX0_VECTOR )
{
  if (magicByte == SB_STACK_VALUE)
  {
    void (*rxIsr)(void);
    rxIsr = (void (*)(void))0x2013;
    rxIsr();
  }
  else if (magicByte == SB_MAGIC_VALUE)
  {
    halUartRxIsr();
  }
  else
  {
    asm("NOP");  // Not expected.
  }
}

HAL_ISR_FUNCTION( halUart0TxIsr, UTX0_VECTOR )
{
  if (magicByte == SB_STACK_VALUE)
  {
    void (*txIsr)(void);
    txIsr = (void (*)(void))0x203B;
    txIsr();
  }
  else if (magicByte == SB_MAGIC_VALUE)
  {
    halUartTxIsr();
  }
  else
  {
    asm("NOP");  // Not expected.
  }
}

/**************************************************************************************************
*/
