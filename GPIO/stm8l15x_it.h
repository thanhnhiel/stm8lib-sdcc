/**
  ******************************************************************************
  * @file    GPIO/GPIO_Toggle/stm8l15x_it.h
  * @author  MCD Application Team
  * @version V1.5.2
  * @date    30-September-2014
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8L15x_IT_H
#define __STM8L15x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler);

INTERRUPT_HANDLER(FLASH_IRQHandler, 1);

INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler, 2);

INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler, 3);

INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler, 4);

INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler, 5);

INTERRUPT_HANDLER(EXTIB_G_IRQHandler, 6);

INTERRUPT_HANDLER(EXTID_H_IRQHandler, 7);

INTERRUPT_HANDLER(EXTI0_IRQHandler, 8);

INTERRUPT_HANDLER(EXTI1_IRQHandler, 9);

INTERRUPT_HANDLER(EXTI2_IRQHandler, 10);

INTERRUPT_HANDLER(EXTI3_IRQHandler, 11);

INTERRUPT_HANDLER(EXTI4_IRQHandler, 12);

INTERRUPT_HANDLER(EXTI5_IRQHandler, 13);

INTERRUPT_HANDLER(EXTI6_IRQHandler, 14);

INTERRUPT_HANDLER(EXTI7_IRQHandler, 15);

INTERRUPT_HANDLER(LCD_AES_IRQHandler, 16);

INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler, 17);

INTERRUPT_HANDLER(ADC1_COMP_IRQHandler, 18);

INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19);

INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler, 20);

INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler, 21);

INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler, 22);

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler, 23);

INTERRUPT_HANDLER(TIM1_CC_IRQHandler, 24);

INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler, 25);

INTERRUPT_HANDLER(SPI1_IRQHandler, 26);

INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler, 27);

INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler, 28);

INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler, 29);


#endif /* __STM8L15x_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/