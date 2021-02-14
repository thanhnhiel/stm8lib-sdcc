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
//#include "mpr121.h"
#include "uart.h"


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
void I2C_LowLevel_Init(void);
void UART_LowLevel_Init(void);
void LM75_Init(void);

uint8_t LM75_ReadTemp(uint16_t *val);

void Delay (uint16_t nCount);
INTERRUPT_HANDLER(EXTI1_IRQHandler, 9);
__IO uint8_t pressed = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    uint8_t counter = 0;

    //CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
    CLK->CKDIVR = (uint8_t)(CLK_SYSCLKDiv_8);

    LM75_Init();
   
    UART_LowLevel_Init();
    uart_init();

    //I2C_init();
    //mpr121_setup();
    
    /* Initialize LEDs mounted on STM8L152X-EVAL board */
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast);
    
    GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_PU_IT);
    
    //EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Falling);
    EXTI->CR1 &=  (uint8_t)(~EXTI_CR1_P1IS);
    EXTI->CR1 |= (uint8_t)((uint8_t)(EXTI_Trigger_Falling) << EXTI_Pin_1);
    //enableInterrupts();
    
    printf("LM75_ReadTemp...");
    uint16_t t;
    if (LM75_ReadTemp(&t) == 0) 
    {
      printf("Done\r\n");
    }
    else
    {
      printf("ERROR\r\n");
    }
    
    while (1)
    {
        // if (pressed == 1 || (GPIOB->IDR & GPIO_Pin_1)==0)
        // {
            // GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
            // Delay(0xFFFF);
            // Delay(0xFFFF);
            // Delay(0xFFFF);
            // Delay(0xFFFF);
            // pressed = 0;
        // }
        

        {
            printf("Test, %d\r\n", counter++);
            //delay_ms(500);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);
        }
    }
}

void I2C_LowLevel_Init(void)
{
  /* Enable the peripheral Clock */
  //CLK->PCKENR1 |= (uint8_t)((uint8_t)1 << CLK_Peripheral_I2C1);
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);

}

#define LM75_I2C_SPEED      100000 /*!< I2C Speed */
void LM75_Init(void)
{

  I2C_LowLevel_Init();

  /* I2C DeInit */
  I2C_DeInit(I2C1);

  /* I2C configuration */
  I2C_Init(I2C1, LM75_I2C_SPEED, 0x00, I2C_Mode_I2C,
           I2C_DutyCycle_2, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

  /*!< Enable SMBus Alert interrupt */
  //I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

  /*!< LM75_I2C Init */
  I2C_Cmd(I2C1, ENABLE);
}

void UART_LowLevel_Init(void)
{
    /*!< USART1 Tx- Rx (PC3- PC2) remapping to PA2- PA3 */
    SYSCFG->RMPCR1 &= (uint8_t)((uint8_t)((uint8_t)0x011C << 4) | (uint8_t)0x0F);
    SYSCFG->RMPCR1 |= (uint8_t)((uint16_t)0x011C & (uint16_t)0x00F0);

    /* Enable USART clock */
    //CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
      /* Enable the peripheral Clock */
    CLK->PCKENR1 |= (uint8_t)((uint8_t)1 << CLK_Peripheral_USART1);

    /* Configure USART Tx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_3, ENABLE);
    //GPIOA->CR1 |= GPIO_Pin_3;

    /* Configure USART Rx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_2, ENABLE);
    //GPIOA->CR1 |= GPIO_Pin_2;
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

//=================

#define I2C_TIMEOUT         (uint32_t)0x3FFFF /*!< I2C Time out */
#define   LM75_ADDR           0x5A   /*!< LM75 address */
#define	ELE_CFG	0x5E
#define LM75_REG_TEMP        ELE_CFG  /* Temperature Register of LM75 */

uint8_t LM75_ReadTemp(uint16_t *val)
{
  uint32_t I2C_TimeOut = I2C_TIMEOUT;
  __IO uint16_t RegValue = 0;

  /* Enable LM75_I2C acknowledgement if it is already disabled by other function */
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /*------------------------------------- Transmission Phase ------------------*/
  /* Send LM75_I2C START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on LM75_I2C EV5 and clear it */
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && I2C_TimeOut)  /* EV5 */
  {
    I2C_TimeOut--;
  }
  
  if (I2C_TimeOut == 0)
  {
    return 1;
  }
  
  I2C_TimeOut = I2C_TIMEOUT;

  /* Send STLM75 slave address for write */
  I2C_Send7bitAddress(I2C1, LM75_ADDR, I2C_Direction_Transmitter);

  /* Test on LM75_I2C EV6 and clear it */
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && I2C_TimeOut)/* EV6 */
  {
    I2C_TimeOut--;
  }
  
  if (I2C_TimeOut == 0)
  {
    return 1;
  }

  /* Send the temperature register data pointer */
  I2C_SendData(I2C1, LM75_REG_TEMP);

  I2C_TimeOut = I2C_TIMEOUT;
  /* Test on LM75_I2C EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)  && I2C_TimeOut) /* EV8 */
  {
        I2C_TimeOut--;
  }
  
  if (I2C_TimeOut == 0)
  {
    return 1;
  }

  /* Send the temperature register data pointer */
  I2C_SendData(I2C1, 0x00);

  I2C_TimeOut = I2C_TIMEOUT;
  /* Test on LM75_I2C EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)  && I2C_TimeOut) /* EV8 */
  {
        I2C_TimeOut--;
  }
  
  if (I2C_TimeOut == 0)
  {
    return 1;
  }
  #if 0
  /*-------------------------------- Reception Phase --------------------------*/
  /* Send Re-STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

I2C_TimeOut = I2C_TIMEOUT;
  /* Test on EV5 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)  && I2C_TimeOut)  /* EV5 */
  {
    I2C_TimeOut--;
  }
   if (I2C_TimeOut == 0)
  {
    return 1;
  }

  /* Send STLM75 slave address for read */
  I2C_Send7bitAddress(I2C1, LM75_ADDR, I2C_Direction_Receiver);

I2C_TimeOut = I2C_TIMEOUT;
  /* Test on EV6 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)  && I2C_TimeOut)  /* EV6 */
  {
       I2C_TimeOut--;
  }
    if (I2C_TimeOut == 0)
  {
    return 1;
  }

  I2C_TimeOut = I2C_TIMEOUT;
  /* Test on EV7 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)  && I2C_TimeOut)  /* EV7 */
  {
        I2C_TimeOut--;
  }
    if (I2C_TimeOut == 0)
  {
    return 1;
  }

  /* Store LM75_I2C received data */
  RegValue = I2C_ReceiveData(I2C1) << 8;

  /* Disable LM75_I2C acknowledgement */
  I2C_AcknowledgeConfig(I2C1, DISABLE);

  /* Send LM75_I2C STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);

  I2C_TimeOut = I2C_TIMEOUT;
  /* Test on RXNE flag */
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET  && I2C_TimeOut)
  {
        I2C_TimeOut--;
  }
    if (I2C_TimeOut == 0)
  {
    return 1;
  }

  /* Store LM75_I2C received data */
  RegValue |= I2C_ReceiveData(I2C1);

  /* Return Temperature value */
  *val = (RegValue >> 7);
  #endif
  
  return 0;
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
