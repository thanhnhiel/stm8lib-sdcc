/* Includes ------------------------------------------------------------------*/
#include "i2c.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/


void I2C_init() 
{
    I2C1_FREQR = (uint8_t)(1 << I2C1_FREQR_FREQ1);
    I2C1_CCRL = (uint8_t)0x0A; // 100kHz
    I2C1_OARH = (uint8_t)(1 << I2C1_OARH_ADDMODE); // 7-bit addressing
    I2C1_CR1 = (uint8_t)(1 << I2C1_CR1_PE);
}

void I2C_start() 
{
    I2C1_CR2 |= (uint8_t)(1 << I2C1_CR2_START);
    while (!(I2C1_SR1 & (uint8_t)(1 << I2C1_SR1_SB)));
}

void I2C_stop() 
{
    I2C1_CR2 |= (uint8_t)(1 << I2C1_CR2_STOP);
    while (I2C1_SR3 & (uint8_t)(1 << I2C1_SR3_MSL));
}

void I2C_write(uint8_t data) 
{
    I2C1_DR = data;
    while (!(I2C1_SR1 & (uint8_t)(1 << I2C1_SR1_TXE)));
}

void I2C_write_addr(uint8_t addr) 
{
    I2C1_DR = addr;
    while (!(I2C1_SR1 & (uint8_t)(1 << I2C1_SR1_ADDR)));
    (void) I2C1_SR3; // check BUS_BUSY
    I2C1_CR2 |= (uint8_t)(1 << I2C1_CR2_ACK);
}

uint8_t I2C_read() 
{
    I2C1_CR2 &= ~(1 << I2C1_CR2_ACK);
    I2C_stop();
    while (!(I2C1_SR1 & (uint8_t)(1 << I2C1_SR1_RXNE)));
    return I2C1_DR;
}

void I2C_read_arr(uint8_t *buf, int len) 
{
    while (len-- > 1) {
        I2C1_CR2 |= (uint8_t)(1 << I2C1_CR2_ACK);
        while (!(I2C1_SR1 & (uint8_t)(1 << I2C1_SR1_RXNE)));
        *(buf++) = I2C1_DR;
    }
    *buf = I2C_read();
}

//===========================================================
// (MMA8452_ADDR << 1) + I2C_WRITE
void I2C_write_reg(uint8_t address, uint8_t reg, uint8_t val) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);
    I2C_write(val);
    I2C_stop();
}

uint8_t I2C_read_reg(uint8_t address, uint8_t reg) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);

    /* Generate repeated start and read one byte */
    I2C_start();
    I2C_write_addr(address | I2C_READ);
    return I2C_read();
}

void I2C_read_regs(uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);

    /* Generate repeated start */
    I2C_start();
    I2C_write_addr(address | I2C_READ);

    /* Read multiple registers */
    I2C_read_arr(dest, count);
}


/**
  * @}
  */

/**
  * @}
  */
