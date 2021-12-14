#include "Arduino.h"
#include "Wire.h"

/**
 * The implementation of all of these methods return 0.
 *
 * 980F modified the multi() returns to reverse byte order on the fly if count is negative.
 **/

// initialize I2C
int VL53L0X_i2c_init(TwoWire *i2c);
/** if @param count is negative then pdata will be byte reversed */
int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count, TwoWire *i2c);
/** if @param count is negative then pdata will be byte reversed */
int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, int count, TwoWire *i2c);

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data, TwoWire *i2c);
int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data, TwoWire *i2c);
int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data, TwoWire *i2c);

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data, TwoWire *i2c);
int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data, TwoWire *i2c);
int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data, TwoWire *i2c);
