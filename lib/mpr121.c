/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "mpr121.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

void i2c_write_reg_(uint8_t address, uint8_t reg, uint8_t val) 
{
    I2C_start();
    I2C_write_addr(address);
    I2C_write(reg);
    I2C_write(val);
    I2C_stop();
}

void mpr121_setup(void)
{
  i2c_write_reg_(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  i2c_write_reg_(0x5A, MHD_R, 0x01);
  i2c_write_reg_(0x5A, NHD_R, 0x01);
  i2c_write_reg_(0x5A, NCL_R, 0x00);
  i2c_write_reg_(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  i2c_write_reg_(0x5A, MHD_F, 0x01);
  i2c_write_reg_(0x5A, NHD_F, 0x01);
  i2c_write_reg_(0x5A, NCL_F, 0xFF);
  i2c_write_reg_(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  i2c_write_reg_(0x5A, ELE0_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE0_R, REL_THRESH);
 
  i2c_write_reg_(0x5A, ELE1_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE1_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE2_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE2_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE3_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE3_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE4_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE4_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE5_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE5_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE6_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE6_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE7_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE7_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE8_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE8_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE9_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE9_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE10_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE10_R, REL_THRESH);
  
  i2c_write_reg_(0x5A, ELE11_T, TOU_THRESH);
  i2c_write_reg_(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  i2c_write_reg_(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  i2c_write_reg_(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  #if 0
  /*i2c_write_reg_(0x5A, ATO_CFG0, 0x0B);
  i2c_write_reg_(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   i2c_write_reg_(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  i2c_write_reg_(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  #endif
  
  i2c_write_reg_(0x5A, ELE_CFG, 0x0C);
}

uint16_t mpr121_read()
{
    uint16_t touched = 0;
    uint8_t data[2];
    
    // I2C_start();
    // I2C_write_addr(0x5A);
    // I2C_write(reg);

    /* Generate repeated start */
    I2C_start();
    I2C_write_addr(0x5A | I2C_READ);

    /* Read multiple registers */
    I2C_read_arr(data, 2);
    //16bits that make up the touch states
    touched = ((data[1] << 8) | data[0]); 
    return touched;
}

/**
  * @}
  */

/**
  * @}
  */
