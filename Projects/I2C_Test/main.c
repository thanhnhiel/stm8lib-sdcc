/**
  ******************************************************************************
  * @file    GPIO/GPIO_Toggle/main.c
  * @author  MCD Application Team
  * @version V1.5.2
  * @date    30-September-2014
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "mpr121.h"

/** @addtogroup STM8L15x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* define the GPIO port and pins connected to Leds mounted on STM8L152X-EVAL board */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
INTERRUPT_HANDLER(EXTI1_IRQHandler, 9);
__IO uint8_t pressed = 0;

/* Private functions ---------------------------------------------------------*/

void I2C_LowLevel_Init(void)
{
    //CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
    CLK->PCKENR1 |= (uint8_t)(1 << 0x03);
    
  /* Configure PC.4 as Input pull-up, used as TemperatureSensor_INT */
 // GPIO_Init(GPIOC, LM75_I2C_SMBUSALERT_PIN, GPIO_Mode_In_FL_No_IT);

}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    I2C_LowLevel_Init();
    I2C_init();
    mpr121_setup();
    
    /* Initialize LEDs mounted on STM8L152X-EVAL board */
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast);
    
    GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_PU_IT);
    
    //EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Falling);
    EXTI->CR1 &=  (uint8_t)(~EXTI_CR1_P1IS);
    EXTI->CR1 |= (uint8_t)((uint8_t)(EXTI_Trigger_Falling) << EXTI_Pin_1);
    enableInterrupts();
    
    while (1)
    {
        if (pressed == 1 || (GPIOB->IDR & GPIO_Pin_1)==0)
        {
            GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);
            pressed = 0;
        }
        
        
    }
}

INTERRUPT_HANDLER(EXTI1_IRQHandler, 9)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */

  /* Joystick UP button is pressed */
  pressed = 1;
  /* Cleat Interrupt pending bit */
  //EXTI_ClearITPendingBit(EXTI_IT_Pin1);
  EXTI->SR1 = (uint8_t) (EXTI_IT_Pin1);
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
