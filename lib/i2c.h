/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define _SFR_(mem_addr)       (*(volatile uint8_t *)(0x5000 | (mem_addr)))

/* I2C */
#define I2C1_BASE_ADDRESS        (I2C1_BASE)
#define I2C1_CR1                 _SFR_(I2C1_BASE_ADDRESS + 0x00)
#define I2C1_CR1_PE              0
#define I2C1_CR2                 _SFR_(I2C1_BASE_ADDRESS + 0x01)
#define I2C1_CR2_ACK             2
#define I2C1_CR2_STOP            1
#define I2C1_CR2_START           0
#define I2C1_FREQR               _SFR_(I2C1_BASE_ADDRESS + 0x02)
#define I2C1_FREQR_FREQ2         2
#define I2C1_FREQR_FREQ1         1
#define I2C1_FREQR_FREQ0         0
#define I2C1_OARL                _SFR_(I2C1_BASE_ADDRESS + 0x03)
#define I2C1_OARH                _SFR_(I2C1_BASE_ADDRESS + 0x04)
#define I2C1_OARH_ADDMODE        7
#define I2C1_OARH_ADDCONF        6
#define I2C1_DR                  _SFR_(I2C1_BASE_ADDRESS + 0x06)
#define I2C1_SR1                 _SFR_(I2C1_BASE_ADDRESS + 0x07)
#define I2C1_SR1_TXE             7
#define I2C1_SR1_RXNE            6
#define I2C1_SR1_BTF             2
#define I2C1_SR1_ADDR            1
#define I2C1_SR1_SB              0
#define I2C1_SR2                 _SFR_(I2C1_BASE_ADDRESS + 0x08)
#define I2C1_SR3                 _SFR_(I2C1_BASE_ADDRESS + 0x09)
#define I2C1_SR3_BUSY            1
#define I2C1_SR3_MSL             0
#define I2C1_ITR                 _SFR_(I2C1_BASE_ADDRESS + 0x0A)
#define I2C1_CCRL                _SFR_(I2C1_BASE_ADDRESS + 0x0B)
#define I2C1_CCRH                _SFR_(I2C1_BASE_ADDRESS + 0x0C)
#define I2C1_TRISER              _SFR_(I2C1_BASE_ADDRESS + 0x0D)
#define I2C1_PECR                _SFR_(I2C1_BASE_ADDRESS + 0x0E)


/* Exported macro ------------------------------------------------------------*/
#define I2C_READ            0x01
#define I2C_WRITE           0x00

/* Exported functions ------------------------------------------------------- */

void I2C_init();

void I2C_start();

void I2C_stop();

void I2C_write(uint8_t data);

void I2C_write_addr(uint8_t addr);

uint8_t I2C_read();

void I2C_read_arr(uint8_t *buf, int len);


void i2c_write_reg(uint8_t address, uint8_t reg, uint8_t val) ;
uint8_t i2c_read_reg(uint8_t address, uint8_t reg) ;
void i2c_read_regs(uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count);

#endif /* __STM8L15x_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/