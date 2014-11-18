/*
 * i2c.h
 * Basic functions for setting up and using hardware I2C
 *
 *  Created on: 16 Nov 2014
 *      Author: ari
 */

#ifndef I2C_H_
#define I2C_H_

//#include <stdbool.h>
//#include <stdint.h>
//#include <stdarg.h>

// parses the command and returns a message describing what was done
char* parse_i2c_command(char* command);

//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(bool high_speed);

void writeI2C0(uint8_t device_address, uint8_t device_data[], uint16_t data_size);

#endif /* I2C_H_ */
