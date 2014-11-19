/*
 * i2c.c
 *
 *  Created on: 16 Nov 2014
 *      Author: Ari Croock
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "i2c.h"

struct i2c_command {
	char message[4096];
	uint8_t address;		// stores the 7 bit address and the read/write bit
	uint8_t data[256];
	uint8_t data_index;
	//TODO: add more fields for receiving
};
static const struct i2c_command empty_command;
static struct i2c_command command;

/*
 * Console State Machine declarations
 * array and enum below must be in sync!
 * Source: http://stackoverflow.com/questions/1371460/state-machines-tutorials/1371654#1371654
 */
#define ENTRY start_bit
int find_start_bit(struct i2c_command* command, char** index);
int find_address(struct i2c_command* command, char** index);
int find_data(struct i2c_command* command, char** index);
int find_stop_bit(struct i2c_command* command, char** index);
int i2c_execute(struct i2c_command* command, char** index);
int end_command(struct i2c_command* command, char** index);
int i2c_error(struct i2c_command* command, char** index);

typedef int (*smFunc)(struct i2c_command* command, char** index);
static const smFunc state[] = {find_start_bit, find_address, find_data, find_stop_bit, i2c_execute, end_command, i2c_error};
enum state_codes {
	start_bit, address, data, stop_bit, execute, end, error
};
enum ret_codes {
	ok, fail, repeat
};

struct transition {
	enum state_codes src_state;
	enum ret_codes ret_code;
	enum state_codes dst_state;
};

// keep this up-to-date!
#define I2C_TRANSITIONS 7
/* transitions from end state aren't needed */
static const struct transition state_transitions[] = {
		{ ENTRY, ok, address },
		{ ENTRY, repeat, ENTRY },
		{ address, ok,	data },
		{ data, ok,	stop_bit },
		{ data, repeat, data },
		{ stop_bit, ok, execute },
		{ execute, ok, end },
		{ error, fail, error }, };

static enum state_codes lookup_transition(enum state_codes curr_state,
		enum ret_codes ret_code) {
	uint8_t i;
	for (i = 0; i < I2C_TRANSITIONS; i++) {
		if (ret_code == fail && state_transitions[i].ret_code == fail) {
			return state_transitions[i].dst_state;
		}

		if (state_transitions[i].src_state == curr_state
				&& state_transitions[i].ret_code == ret_code) {
			return state_transitions[i].dst_state;
		}
	}

	return error;
}

// global state machine current state
// volatile so that interrupts can update the SM
static volatile enum state_codes curr_state = ENTRY;
static volatile enum ret_codes ret_code = ok;
static smFunc state_func = find_start_bit;

void InitI2C0(bool high_speed) {
	//enable I2C module 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//reset module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	//enable GPIO peripheral that contains I2C 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), high_speed);

	//clear I2C FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void writeI2C0(uint8_t device_address, uint8_t device_data[],
		uint16_t data_size) {
	if (data_size < 1) {
		return;
	}

	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);

	I2CMasterDataPut(I2C0_BASE, device_data[0]);

	if (data_size < 2) {
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	} else {
		//send control byte and register address byte to slave device
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

		//wait for MCU to finish transaction
		while (I2CMasterBusy(I2C0_BASE)){}

		uint16_t i;
		for (i = 1; i < data_size - 1; i++) {
			I2CMasterDataPut(I2C0_BASE, device_data[i]);
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			while (I2CMasterBusy(I2C0_BASE)){}
		}

		I2CMasterDataPut(I2C0_BASE, device_data[data_size - 1]);
		//wait while checking for MCU to complete the transaction
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	}

	//wait for MCU & device to complete transaction
	while (I2CMasterBusy(I2C0_BASE)){}
}

uint8_t readI2C0(uint8_t device_address) {
	//specify that we want to communicate to device address with an intended read from bus
	I2CMasterSlaveAddrSet(I2C0_BASE, device_address, true);

	//send control byte and read from the register we
	//specified
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

	//wait for MCU to finish transaction
	while (I2CMasterBusy(I2C0_BASE)){}

	//return data pulled from the specified register
	return I2CMasterDataGet(I2C0_BASE);
}

int find_start_bit(struct i2c_command* command, char** index) {
	if (**index == '[') {
		++(*index);
		return ok;
	}
	strcat(command->message, "No start bit");
	return fail;
}

int find_address(struct i2c_command* command, char** index) {
	char* p = strtok(*index, " ");
	if (p != NULL) {
		// the tokenizer includes the stop char ']' in the last token
		// if it's there, set the next char to ']' and insert a null char in between
		*index = strchr(p, ']');
		if (*index != NULL) {
			**index = '\0';
			(*index)++;
			**index = ']';
		}

		strcat(command->message, "ADDR: ");
		strcat(command->message, p);

		command->address = strtoul(p, NULL, 0);

		return ok;
	}

	strcat(command->message, " No Address byte");
	return fail;
}

int find_data(struct i2c_command* command, char** index) {
	if (command->address & 0x01) {
		// read so skip data bytes
		return ok;
	}

	char* tok = strtok(*index, " ");
	char* next_tok;

	if (tok == NULL) {
		strcat(command->message, " No Data byte");
		return fail;
	}

	while ((next_tok = strtok(NULL, " ")) != NULL) {
		strcat(command->message, ", DATA: ");
		strcat(command->message, tok);
		uint8_t data = strtoul(tok, NULL, 0);
		command->data[command->data_index++] = data;

		tok = next_tok;
	}

	if (tok != NULL) {
		// the tokenizer includes the stop char ']' in the last token
		// if it's there, set the next char to ']' and insert a null char in between
		*index = strchr(tok, ']');
		if (*index != NULL) {
			**index = '\0';
			(*index)++;
			**index = ']';
		}

		strcat(command->message, ", DATA: ");
		strcat(command->message, tok);

		uint8_t data = strtoul(tok, NULL, 0);
		command->data[command->data_index++] = data;
	}

	return ok;
}

int find_stop_bit(struct i2c_command* command, char** index) {
	if (**index == ']') {
		return ok;
	}
	strcat(command->message, " No stop bit");
	return fail;
}

int i2c_execute(struct i2c_command* command, char** index) {
	// if lsb of address is 0, write to slave, else read
	if (command->address & 0x01) {
		// read
		uint8_t data = readI2C0(command->address >> 1);
		char temp[25];
		sprintf(temp, "\nReceived Data: 0x%x", data);
		strcat(command->message, temp);
		return ok;
	} else {
		// write
		// right shift to get rid of read/write bit (7 bit address)
		writeI2C0(command->address >> 1, command->data, command->data_index);
		strcat(command->message, ", Transmitting...");
		//TODO: return error code
		return ok;
	}
}

int i2c_error(struct i2c_command* command, char** index) {
	strcat(command->message, "Error");
	return fail;
}

int end_command(struct i2c_command* command, char** index) {
	return ok;
}

// takes a command entered on the command line as a string and attempts to execute it.
// syntax is [{ADDRESS with read/write bit} {DATA} {DATA} ... {DATA}]
// ADDRESS, DATA can be decimal or hex (0xff) for now
// NOTE: command should be a copy as this function will modify it
char* parse_i2c_command(char* str_command) {
	//start the parser state machine
	// index is a pointer to the current position in the command string being parsed
	char* index = str_command;
	command = empty_command;
	//command = (struct i2c_command){0};	// clear struct
	curr_state = ENTRY;
	while (curr_state != end && curr_state != error) {
		state_func = state[curr_state];
		ret_code = state_func(&command, &index);
		curr_state = lookup_transition(curr_state, ret_code);
	}

	return command.message;
}
