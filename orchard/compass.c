
#include "ch.h"
#include "hal.h"
#include "i2c.h"

#include "compass.h"
#include "gpiox.h"
#include "orchard.h"
#include "orchard-events.h"

#include "orchard-test.h"
#include "test-audit.h"


/*
 * 0x03C 0x00 0x70 8 avg, 15 Hz normal measurement
 * 0x03C 0x01 0xA0 gain=5 
 * 0x03C 0x02 0x00 continuous measurement mode
 * wait 6ms
 * 
 * loop
 * 0x03D 0x06 read 6 bytes (3 16bit 2s compliment hex -> x,z,y)
 * 0x03C 0x03 point to first data register 03
 * wait 70ms
 *
 * */

#define COMPASS_DEVICE_ADDRESS (0x1E << 1)
#define COMPASS_READ (1 << 0)
#define COMPASS_WRITE (0 << 0)

/*
 * read = COMPASS_DEVICE_ADDRESS | COMPASS_READ
 * write = COMPASS_DEVICE_ADDRESS | COMPASS_WRITE
 */

#define REG_CONFIG_A 0x00
#define REG_CONFIG_A_DEFAULT 0x70  //0111 0000

#define REG_CONFIG_B 0x01
#define REG_CONFIG_B_DEFAULT 0x20 //0010 0000
#define REG_CONFIG_B_HIGHGAIN 0xA0 //1010 0000

#define REG_MODE 0x02
#define REG_MODE_DEFAULT 0x00 //0000 0000   400kHz I2C 
#define REG_MODE_HSI2C 0x80 //1000 0000   3400kHz I2C

#define REG_MODE_SINGLE 0x01 //0000 0001   400kHz I2C 
#define REG_MODE_SINGLEHSI2C 0x81 //1000 0001   3400kHz I2C

#define REG_MODE_STANDBY 0x03 //0000 0011 idle mode


#define REG_X_MSB 0x03
#define REG_X_LSB 0x04
#define REG_Z_MSB 0x05
#define REG_Z_LSB 0x06
#define REG_Y_MSB 0x07
#define REG_Y_LSB 0x08

#define REG_STATUS 0x09
#define REG_STATUS_RDY 0x01

#define REG_ID_A 0x0A
#define REG_ID_B 0x0B
#define REG_ID_C 0x0C

static I2CDriver *driver;

static void compass_set(uint8_t reg, uint8_t val) {
  //write to control registers with value
  
  uint8_t tx[2] = {reg, val};

  i2cMasterTransmitTimeout(driver, compassAddr,
                           tx, sizeof(tx),
                           NULL, 0,
                           TIME_INFINITE);
}


static void compass_get_address(uint8_t reg) {
  //set address that will be read from on read call

  i2cMasterTransmitTimeout(driver, compassAddr,
                           &reg, 1,
                           NULL, 0,
                           TIME_INFINITE);
}

int compass_ready(void){
  
    uint8_t reg = 0x01;
    uint8_t val;

    compass_get_address(REG_STATUS);
  
    i2cMasterTransmitTimeout(driver, compassAddr,
                           &reg, 1,
                           &val, 1,
                           TIME_INFINITE);

    return (val & REG_STATUS_RDY);
}


static void compass_get(uint8_t val[]) {
  //reg represents number of registers to read
  
  uint8_t reg = 0x06;

//  while (!(compass_ready()));

  compass_get_address(REG_X_MSB);

  i2cMasterTransmitTimeout(driver, compassAddr,
                           &reg, 1,
                           val, 6,
                           TIME_INFINITE);
}


void compassStop(void) {
  i2cAcquireBus(driver);
  
  //set to standby
  compass_set(REG_MODE, REG_MODE_STANDBY);
  i2cReleaseBus(driver);
}


void compassStart(I2CDriver *i2cp) {

  driver = i2cp;

  i2cAcquireBus(driver);

  /* set-up the chip */
  compass_set(REG_CONFIG_A, REG_CONFIG_A_DEFAULT);
  compass_set(REG_CONFIG_B, REG_CONFIG_B_DEFAULT);
  compass_set(REG_MODE, REG_MODE_DEFAULT);

  i2cReleaseBus(driver);

}



msg_t compassPoll(struct compass_data *data) {
 
  uint8_t rx[6];

  i2cAcquireBus(driver);
    
  compass_get(rx);

  i2cReleaseBus(driver);

  data->x  = ((rx[5] & 0xffff)) << 8;
  data->x |= ((rx[4]) & 0x00ff);
  
  data->y  = ((rx[1] & 0xffff)) << 8;
  data->y |= ((rx[0]) & 0x00ff);
  
  data->z  = ((rx[3] & 0xffff)) << 8;
  data->z |= ((rx[2]) & 0x00ff);

  return MSG_OK;
}

/*
OrchardTestResult test_compass(const char *my_name, OrchardTestType test_type) {
  (void) my_name;
  uint8_t ret;
  
  switch(test_type) {
  case orchardTestPoweron:
  case orchardTestTrivial:
    i2cAcquireBus(driver);
    ret =  compass_get(REG_WHO_AM_I);
    i2cReleaseBus(driver);
    if( ret != 0x2A ) {
      return orchardResultFail;
    } else {
      return orchardResultPass;
    }
    break;
  default:
    return orchardResultNoTest;
  }
  
  return orchardResultNoTest;
}
orchard_test("compass", test_compass);
*/
