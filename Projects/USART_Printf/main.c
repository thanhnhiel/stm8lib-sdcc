/**
  ******************************************************************************
  * @file    USART/USART_Printf/main.c
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
#include "stdio.h"



/** @addtogroup STM8L15x_StdPeriph_Examples
  * @{
  */

/**
  * @addtogroup USART_Printf
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define FREG 2000000
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void STM_COMInit();
void uart1_init(uint32_t BaudRate);

void Delay(__IO uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /*High speed internal clock prescaler: 1*/
  //CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);   
  CLK->CKDIVR = (uint8_t)(CLK_SYSCLKDiv_8);


  /* EVAL COM (USART1) configuration -----------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Receive and transmit enabled
        - USART Clock disabled
  */
  STM_COMInit();

  /* Output a message on Hyperterminal using printf function */
  printf("\n\rUSART Example: retarget the C library printf()\getchar() functions to the USART\n\r");
  printf("\n\rEnter Text\n\r");

uint8_t counter = 0;
uint32_t clockFreg = FREG; //CLK_GetClockFreq();

  while (1)
  {
            printf("Test, %d freg: %ld\r\n", counter++, clockFreg);
            //delay_ms(500);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);
            Delay(0xFFFF);

  }
}

void STM_COMInit()
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

  /* USART configuration */
  uart1_init(115200);
  // USART_Init(USART1, 115200,
  //            USART_WordLength_8b,
  //            USART_StopBits_1,
  //            USART_Parity_No,
  //            (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
}

void uart1_init(uint32_t BaudRate)
{
  uint32_t BaudRate_Mantissa = 0;

  /* Clear the word length and Parity Control bits */
  USART1->CR1 &= (uint8_t)(~(USART_CR1_PCEN | USART_CR1_PS | USART_CR1_M));
  /* Set the word length bit according to USART_WordLength value */
  /* Set the Parity Control bit to USART_Parity value */
  USART1->CR1 |= (uint8_t)((uint8_t)USART_WordLength_8b | (uint8_t)USART_Parity_No);

  /* Clear the STOP bits */
  USART1->CR3 &= (uint8_t)(~USART_CR3_STOP);
  /* Set the STOP bits number according to USART_StopBits value */
  USART1->CR3 |= (uint8_t)USART_StopBits_1;

  /* Clear the LSB mantissa of USARTDIV */
  USART1->BRR1 &= (uint8_t)(~USART_BRR1_DIVM);
  /* Clear the MSB mantissa of USARTDIV */
  USART1->BRR2 &= (uint8_t)(~USART_BRR2_DIVM);
  /* Clear the Fraction bits of USARTDIV */
  USART1->BRR2 &= (uint8_t)(~USART_BRR2_DIVF);

  //BaudRate_Mantissa  = (uint32_t)(CLK_GetClockFreq() / BaudRate );
  BaudRate_Mantissa  = (uint32_t)(FREG / BaudRate );

  /* Set the fraction of USARTDIV */
  USART1->BRR2 = (uint8_t)((BaudRate_Mantissa >> (uint8_t)8) & (uint8_t)0xF0);
  /* Set the MSB mantissa of USARTDIV */
  USART1->BRR2 |= (uint8_t)(BaudRate_Mantissa & (uint8_t)0x0F);
  /* Set the LSB mantissa of USARTDIV */
  USART1->BRR1 = (uint8_t)(BaudRate_Mantissa >> (uint8_t)4);

  /* Disable the Transmitter and Receiver */
  USART1->CR2 &= (uint8_t)~(USART_CR2_TEN | USART_CR2_REN);
  /* Set TEN and REN bits according to USART_Mode value */
  USART1->CR2 |= (uint8_t)(USART_Mode_Tx | USART_Mode_Rx);
}
/**
  * @brief Retargets the C library printf function to the USART.
  * @param[in] c Character to send
  * @retval char Character sent
  * @par Required preconditions:
  * - None
  */
int putchar (int c)
{
  /* Write a character to the USART */
  USART1->DR = c;
  //USART_SendData8(USART1, c);
  /* Loop until the end of transmission */
  while ((USART1->SR & (uint8_t)USART_FLAG_TC) == 0);

  return (c);
}
/**
  * @brief Retargets the C library scanf function to the USART.
  * @param[in] None
  * @retval char Character to Read
  * @par Required preconditions:
  * - None
  */
int getchar (void)
{
    int c = 0;
    /* Loop until the Read data register flag is SET */
    while ((USART1->SR & (uint8_t)USART_FLAG_RXNE) == 0);
    c = USART1->DR;
    return (c);
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
