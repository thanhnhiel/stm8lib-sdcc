#include "stm8l15x.h"
#include "uart.h"

void uart_init() 
{
    /* round to nearest integer */
    uint16_t div = (uint16_t)(F_CPU  / BAUDRATE);

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

    /* Set the fraction of USARTDIV */
    USART1->BRR2 = (uint8_t)((div >> (uint8_t)8) & (uint8_t)0xF0);
    /* Set the MSB mantissa of USARTDIV */
    USART1->BRR2 |= (uint8_t)(div & (uint8_t)0x0F);
    /* Set the LSB mantissa of USARTDIV */
    USART1->BRR1 = (uint8_t)(div >> (uint8_t)4);

    /* enable transmitter and receiver */
    /* Disable the Transmitter and Receiver */
    USART1->CR2 &= (uint8_t)~(USART_CR2_TEN | USART_CR2_REN);
    /* Set TEN and REN bits according to USART_Mode value */
    USART1->CR2 |= (uint8_t)(USART_Mode_Tx | USART_Mode_Rx);
}

void uart_write(uint8_t data) 
{
    /* Write a character to the USART */
    USART1->DR = data;
    /* Loop until the end of transmission */
    while (!(USART1->SR & (uint8_t)USART_FLAG_TC));
}

uint8_t uart_read() 
{
    /* Loop until the Read data register flag is SET */
    while (!(USART1->SR & (uint8_t)USART_FLAG_RXNE));
    return USART1->DR;
}

int putchar (int c)
{
    uart_write(c);  
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
    return uart_read();
}
