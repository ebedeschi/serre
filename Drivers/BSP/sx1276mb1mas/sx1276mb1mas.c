/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /*******************************************************************************
  * @file    sx1276mb1mas.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    15-September-2016
  * @brief   driver sx1276mb1mas board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276mb1mas.h"


#define IRQ_HIGH_PRIORITY  0
/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
  SX1276IoInit,
  SX1276IoDeInit,
  SX1276Init,
  SX1276GetStatus,
  SX1276SetModem,
  SX1276SetChannel,
  SX1276IsChannelFree,
  SX1276Random,
  SX1276SetRxConfig,
  SX1276SetTxConfig,
  SX1276CheckRfFrequency,
  SX1276GetTimeOnAir,
  SX1276Send,
  SX1276SetSleep,
  SX1276SetStby, 
  SX1276SetRx,
  SX1276StartCad,
  SX1276ReadRssi,
  SX1276Write,
  SX1276Read,
  SX1276WriteBuffer,
  SX1276ReadBuffer,
  SX1276SetSyncWord,
  SX1276SetMaxPayloadLength
};


void SX1276IoInit( void )
{
  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_PULLUP;
  initStruct.Speed = GPIO_SPEED_HIGH;

  HW_GPIO_Init( RADIO_DIO0_GPIO_Port, RADIO_DIO0_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, &initStruct );

}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
  HW_GPIO_SetIrq( RADIO_DIO0_GPIO_Port, RADIO_DIO0_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
  HW_GPIO_SetIrq( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, IRQ_HIGH_PRIORITY, irqHandlers[1] );
  HW_GPIO_SetIrq( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, IRQ_HIGH_PRIORITY, irqHandlers[2] );
  HW_GPIO_SetIrq( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, IRQ_HIGH_PRIORITY, irqHandlers[3] );
}


void SX1276IoDeInit( void )
{
  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode = GPIO_MODE_IT_RISING ;//GPIO_MODE_ANALOG;
  initStruct.Pull = GPIO_NOPULL;
  
  HW_GPIO_Init( RADIO_DIO0_GPIO_Port, RADIO_DIO0_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, &initStruct );
  HW_GPIO_Init( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, &initStruct );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
	return RF_PACONFIG_PASELECT_PABOOST;
//    return RF_PACONFIG_PASELECT_RFO;
}


static void SX1276AntSwInit( void )
{
//  GPIO_InitTypeDef initStruct={0};
//
//  initStruct.Mode =GPIO_MODE_OUTPUT_PP;
//  initStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP;
//  initStruct.Speed = GPIO_SPEED_HIGH;
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, RADIO_ANT_SWITCH_SET_RX);
}

static void SX1276AntSwDeInit( void )
{
//  GPIO_InitTypeDef initStruct={0};
//
//  initStruct.Mode = GPIO_MODE_OUTPUT_PP ;
//
//  initStruct.Pull = GPIO_NOPULL;
//  initStruct.Speed = GPIO_SPEED_HIGH;
//
//  HW_GPIO_Init(  RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, &initStruct );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, 0);
}


void SX1276SetAntSwLowPower( bool status )
{
//  if( RadioIsActive != status )
//  {
//    RadioIsActive = status;
//
//    if( status == false )
//    {
//      SX1276AntSwInit( );
//    }
//    else
//    {
//      SX1276AntSwDeInit( );
//    }
//  }
}

void SX1276SetAntSw( uint8_t rxTx )
{
//  if( SX1276.RxTx == rxTx )
//  {
//    return;
//  }
//
//  SX1276.RxTx = rxTx;
//
//  if( rxTx != 0 ) // 1: TX, 0: RX
//  {
//    HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, RADIO_ANT_SWITCH_SET_TX );
//  }
//  else
//  {
//    HW_GPIO_Write( RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, RADIO_ANT_SWITCH_SET_RX );
//  }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
