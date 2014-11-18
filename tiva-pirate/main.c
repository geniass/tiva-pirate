/*
 * main.c
 * Really basic bus pirate clone for TI Tiva C demo boards
 *
 *  Created on: 16 Nov 2014
 *      Author: Ari Croock
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

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
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "i2c.h"

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define CLOCK_FREQ 80000000
#define LCD_ADDR 0x38

/*
 * Console State Machine declarations
 * array and enum below must be in sync!
 * Source: http://stackoverflow.com/questions/1371460/state-machines-tutorials/1371654#1371654
 */
#define ENTRY mode_select
int console_mode_select();
int console_i2c_speed_select();
int console_i2c_command();
int console_error();

typedef int (*smFunc)(void);
smFunc state[] = {console_mode_select, console_i2c_speed_select, console_i2c_command, console_error};
enum state_codes {
	mode_select, i2c_speed_select, i2c_command, error
};
enum ret_codes {
	ok, fail, repeat, i2c
};

struct transition {
	enum state_codes src_state;
	enum ret_codes ret_code;
	enum state_codes dst_state;
};

// keep this up-to-date!
#define NUM_TRANSITIONS 7
/* transitions from end state aren't needed */
static const struct transition state_transitions[] = {
		{ ENTRY, i2c, i2c_speed_select },
		{ ENTRY, repeat, ENTRY },
		{ i2c_speed_select, repeat,	i2c_speed_select },
		{ i2c_speed_select, ok,	i2c_command },
		{ i2c_command, repeat, i2c_command },
		{ i2c_command, ok, ENTRY },
		{ error, fail, ENTRY }, };

enum state_codes lookup_transition(enum state_codes curr_state,
		enum ret_codes ret_code) {
	uint8_t i;
	for (i = 0; i < NUM_TRANSITIONS; i++) {
		if (state_transitions[i].src_state == curr_state
				&& state_transitions[i].ret_code == ret_code) {
			return state_transitions[i].dst_state;
		}
	}

	return error;
}

// global state machine current state
// volatile so that interrupts can update the SM
volatile enum state_codes curr_state = ENTRY;
volatile enum ret_codes ret_code;
volatile smFunc state_fun;

// buffer for UART console input
char console_read_buffer[4096];
uint16_t console_buffer_index = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void UARTInit() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlDelay(3);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, 115200, 16000000);

	// Enable the UART interrupt.
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	// Enable processor interrupts.
	ROM_IntMasterEnable();
}

void delay_ms(uint32_t ms) {
	SysCtlDelay((ms / 1000.0) * SysCtlClockGet() / 3.0); // assuming clock is Mhz
}

int console_mode_select() {
	if (strlen(console_read_buffer) > 0) {
		if (strcmp(console_read_buffer, "i2c") == 0) {
			// clear input buffer so it doesn't affect next state
			console_read_buffer[0] = '\0';
			return i2c;
		}

		UARTprintf("\nInvalid mode. Try again\n");
	}

	// invalid input; prompt again
	console_buffer_index = 0;
	console_read_buffer[0] = '\0';
	UARTprintf("\nMODE> ");
	// wait for interrupt to continue
	return repeat;
}

int console_i2c_speed_select() {
	if (strlen(console_read_buffer) > 0) {
		if (strcmp(console_read_buffer, "hi") == 0) {
			//high speed
			InitI2C0(true);
			// clear input buffer so it doesn't affect next state
			console_read_buffer[0] = '\0';
			return ok;
		} else if (strcmp(console_read_buffer, "lo") == 0) {
			//low speed
			InitI2C0(false);
			// clear input buffer so it doesn't affect next state
			console_read_buffer[0] = '\0';
			return ok;
		}
		UARTprintf("\nInvalid speed. Try again\n");
	}

	// invalid input; prompt again
	console_buffer_index = 0;
	console_read_buffer[0] = '\0';
	UARTprintf("\nI2C Speed (Enter either 'hi' or 'lo'> ");
	// wait for interrupt to continue
	return repeat;
}

int console_i2c_command() {
	if (console_read_buffer[0] != '\0') {
		if (strcmp(console_read_buffer, "close") == 0) {
			// clear input buffer so it doesn't affect next state
			console_read_buffer[0] = '\0';
			return ok;
		} else {
			char command[4096];
			strcpy(command, console_read_buffer);
			char* message = parse_i2c_command(command);
			UARTprintf(message);
			// clear input buffer so it doesn't affect next state
			console_read_buffer[0] = '\0';
			return repeat;
		}
	}

	// invalid input; prompt again
	console_buffer_index = 0;
	console_read_buffer[0] = '\0';
	UARTprintf("\nI2C Command> ");
	// wait for interrupt to continue
	return repeat;
}

int console_error() {
	UARTprintf("\nerror\n ");
	return fail;
}

int main() {
	SysCtlClockSet(
	SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED | LED_BLUE | LED_GREEN);

	UARTInit();

	// initial state machine setup
	state_fun = state[curr_state];
	ret_code = state_fun();
	curr_state = lookup_transition(curr_state, ret_code);

	for (;;) {
		if (UARTPeek(13)) {
			// Blink the LED to show a character transfer is occuring.
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
			SysCtlDelay(SysCtlClockGet() / (1000 * 3));
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
			UARTgets(console_read_buffer, 4096);

			state_fun = state[curr_state];
			ret_code = state_fun();
			curr_state = lookup_transition(curr_state, ret_code);

			if (ret_code != fail || ret_code != repeat) {
				state_fun = state[curr_state];
				ret_code = state_fun();
				curr_state = lookup_transition(curr_state, ret_code);
			}

			UARTFlushRx();
		}
	}

}
