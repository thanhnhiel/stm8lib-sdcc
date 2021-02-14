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
    /* Enable I2C peripheral */
    I2C1->CR1 |= I2C_CR1_PE;
    /* sEE_I2C configuration after enabling it */
    //I2C_Init(sEE_I2C, I2C_SPEED, I2C_SLAVE_ADDRESS7, I2C_Mode_I2C, I2C_DutyCycle_2,
    //       I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
 uint32_t result = 0x0004;
  uint16_t tmpval = 0;
  uint8_t tmpccrh = 0;


  /*------------------------- I2C FREQ Configuration ------------------------*/
  /* Clear frequency bits */
  I2C1->FREQR &= (uint8_t)(~I2C_FREQR_FREQ);
  /* Write new value */
  I2C1->FREQR |= (uint8_t) (F_CPU / 1000000);

  /*--------------------------- I2C CCR Configuration ------------------------*/
  /* Disable I2C to configure TRISER */
  I2C1->CR1 &= (uint8_t)(~I2C_CR1_PE);

  /* Clear CCRH & CCRL */
  I2C1->CCRH &= (uint8_t)(~(I2C_CCRH_FS | I2C_CCRH_DUTY | I2C_CCRH_CCR));
  I2C1->CCRL &= (uint8_t)(~I2C_CCRL_CCR);
 

    /* Calculate standard mode speed */
    result = (uint16_t)(F_CPU / (I2C_SPEED << (uint8_t)1));
    
    /* Verify and correct CCR value if below minimum value */
    if (result < (uint16_t)0x0004)
    {
    /* Set the minimum allowed value */
    result = (uint16_t)0x0004;
    }
    
    /* Set Maximum Rise Time: 1000ns max in Standard Mode
    = [1000ns/(1/input_clock.10e6)]+1
    = input_clock+1 */
    I2C1->TRISER = (uint8_t)((uint8_t) (F_CPU / 1000000) + (uint8_t)1);
     

  /* Write CCR with new calculated value */
  I2C1->CCRL = (uint8_t)result;
  I2C1->CCRH = (uint8_t)((uint8_t)((uint8_t)((uint8_t)result >> 8) & I2C_CCRH_CCR) | tmpccrh);

  /* Enable I2C and  Configure its mode*/
  I2C1->CR1 |= (uint8_t)(I2C_CR1_PE | I2C_Mode_I2C);

  /* Configure I2C acknowledgement */
  I2C1->CR2 |= (uint8_t)I2C_Ack_Enable;

  /*--------------------------- I2C OAR Configuration ------------------------*/
  I2C1->OARL = (uint8_t)(0xA0); // I2C_SLAVE_ADDRESS7
  I2C1->OARH = (uint8_t)((uint8_t)(I2C_AcknowledgedAddress_7bit | I2C_OARH_ADDCONF ) | \
                         (uint8_t)((uint16_t)( (uint16_t)0xA0 &  (uint16_t)0x0300) >> 7));
    // I2C1_FREQR = (uint8_t)(1 << I2C1_FREQR_FREQ1);
    // I2C1_CCRL = (uint8_t)0x0A; // 100kHz
    // I2C1_OARH = (uint8_t)(1 << I2C1_OARH_ADDMODE); // 7-bit addressing

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

void i2c_write_reg(uint8_t address, uint8_t reg, uint8_t val) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);
    I2C_write(val);
    I2C_stop();
}

uint8_t i2c_read_reg(uint8_t address, uint8_t reg) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);

    /* Generate repeated start and read one byte */
    I2C_start();
    I2C_write_addr(address | I2C_READ);
    return I2C_read();
}

void i2c_read_regs(uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count) 
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
