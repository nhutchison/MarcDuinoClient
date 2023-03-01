/*
 * 	main.c
 *  MarcDuino Slave (HoloProjector & Lights Control)
 *  Created on: July 4th, 2012
 *  Author: Marc Verdiell
 *  Version 1.2r: original release 03/26/2013
 *	Version 1.3r: adapting for new JEDI, McWhlr Magic Panel
 *	Version 1.4: consolidate private and release code, HP lights from MarcDuino, PassThrough command
 *	Version 1.5: new buffered serial
 *	Version 1.7: MarcDuino v2 support
 *  Version 1.8: I2C command parsing support, revised serial.c
 *
 *  Version 2.1
 *  	Author: Neil Hutchison
 *  	Updated: March 14th, 2020
 *  	Setup Commands added, EEPROM Support, 13 Servo support etc
 *
 */

/***********************
 * Version 3.6 - Feb 22 2023
 *
 * Support for triggering EXT1 Pin on Client added
 * Can be used to trigger a smoke machine as an example.
 *
 */

/***********************
 *  Version 2.0
 *  Support for 2 Servos on the Slave
 *  EEPROM Support added for storing MarcDuino Settings
 *  Reverso Servo is now dynamic, using the Setup Commands
 *  	Servo direction is settable per servo
 *  Panel Sequencer added to Slave to allow 13 panel sequences
 *  Panel Speed can be set per row in the Panel Sequencer
 *  Start and Stop Servo entries per row in the Panel Sequencer
 *  Maxstang's Additional Panel sequences added by default
 *  Panel Sequencer supports opening Panels half way
 *  Panel Sequencer updated to support 13 servos, and all sequences updated
 *  Support for Neil's Magic Panel FW
 *  	Support for Marc's old MP code removed.
 *  	Servo 11 on the Slave is now used for a servo, not MP
 *  Fix for RC Input code (could not be disabled due to missing #ifdef)
 *  Fix for Servo Hum in certain command cases
 *
 ***********************/

/***********************
 *  Version 1.8:
 *  I2C command parsing support
 *  Updated serial.c and serial.h libraries
 *  	Changed serial_puts so it waits for available output buffer by default
 *  		If not characters were just dropped when sending data too fast.
 *  	Renamed previous version serial_puts_nowait
 *  Updated the comments in i2c.h
 *
 ***********************/

/**********
 * v1.7
 * MarcDuinoV2 support
 * 	Bumped to version 1.7 to match the Master release
 * 	New suart.c library with stuart2 on PC1 output instead of PC4
 * 	Exchange suart usage to macth MarcDuino v2 board (and Master)
 * 		- Light Output to suart2/PC1 (instead of suart)
 * 		- Slave output on suart/PC0 (instead of suart2)
 * 	I2C library support
 * 	REON holoprojector support
 */

/**********
 * v1.52
 * Added HP test movement command
 * Did not create new project
 */

/**********
 * v1.51
 * Corrected non centered HP servo, add extended movement option
 * (all done in the header files)
 * Did not create new project
 * Corrected debug message version
 */

/** v1.5
 *
 * Up'ed the version number to match the corresponding Dome Panel Control version
 * Switch to Interrupt Driven / Buffered serial implementation
 * Reduce delays after JEDI commands now that suart is more reliable
 */

/** v1.4
 * Add control of HP through our own board
 * 	ON and OF commands will turn on HPs located on srv7-9 headers
 * 	New HP lights commands
 * 	*H1xx, *H2xx, *H3xx, and *H0xx
 * 		Will turn on HP1, 2, 3, and all for xx seconds
 * 	*F1xx, *F2xx, *F3xx, and *F0xx
 * 		Will flicker HP1, 2, 3, and all for xx seconds
 * New library files:
 * 	RS232 with pgm memory for strings
 * 	Refactor sor serial calls
 * 	New realtime library with registered timers
 * 	Remove sequencer files
 *  Added print.c and print.h (convenience, adds 2% of code) (? do I need this)
 * Much smaller DRAM footprint for further expansino
 *  Change most strings constant to program memory strings
 * Support for Alt1 (!) and Alt2 (%) expansion commands
 * 	! commands output on suart (with strip)
 * 	% commands output on suart2 (with strip)
 * Disabled interrupts while transmitting in suart.h -> improved reliability
 */

/** v1.3
 *
 * Created compile switch for 9600 baud rate for new JEDI compatibility
 * Created compile switch for inverted on Magic Panel Output
 * 	(to works with my new Holo Lights and Michael Wheeler's magic panel)
 *
 */


/** Commands implemented in v1.2
 *
 * 	Hardware Setup:
 * 	- 8 servo outputs
 * 		Holo 1 V on pins 1 (front, holo 1)
 * 		Holo 1 H on pins 2
 * 		Holo 2 V on pins 3 (rear, holo 2)
 * 		Holo 2 H on pins 4
 * 		Holo 3 V on pins 5 (top, holo 3)
 * 		Holo 3 H on pins 6
 * 		Panel 12 on pin 10 (Smallest front Panel)
 * 		Panel 13 on AUX2 (Top centre panel)
 * 	- 4 digital outputs
 * 		HP1 (front, holo 1) on PORT B Pin 2 (pins labeled SRV7)
 * 		HP2 (rear, holo 2)  on PORT B Pin 3 (pins labeled SRV8)
 * 		HP3 (top, holo 3)   on PORT B Pin 4 (pins labeled SRV9)
 * 	- two suarts (software uart) output
 * 		- suart connected to the JEDI Display light control system at 2400 bauds
 * 		- suart2 initialized at 9600 bauds, for future use (daisy-chain expansion?)
 * 	- one RC input
 * 	- one main hardware UART input
 *		- connected to the master panel controller suart output at 9600 bauds, to receive commands
 *		- output sends commands acknowledgments, (not physically connected to anything at present)
 *
 *  Valid start characters recognized in main()
 *  ':' panel command, ignored (see parse_panel_command). This should not be received by this slaved module anyhow
 *  '$' sound command, ignored (see parse_sound_command). This should not be received by this slaved module anyhow
 *  '@' display command, forwarded to JEDI controller on suart1 after stripping the '@' control character
 *  '*' hp command, acted upon here, see below
 *  '!' Alt1 alternate display command, passed to suart after stripping
 *  '%' Alt2 expansion command, passed to suart2 after stripping
 *		The master HP board will forward these to us
 * "#" MarcDuino Setup commands used to configure various settings on the MarcDuino
 *
 *  Commands recognized (see parse_hp_command)
 *  *RDxx Random Holo movement (xx=01 to 03). xx=00 and >3 all random.
 *  *ONxx Turns Holo Light on (xx=01 to 03). xx=00 or >3 all lights on
 *  *OFxx Turns Holo Lights off (xx=01 to 03). xx=00 turns all lights off
 *  *RCxx Holo vertical movement under RC control, horizontal centered (xx=01-03).
 *  	00 or >3 all RC
 *  *TExx Holo movement test (xx=01-03). Goes through a loop of holo movements
 *  	to assist in adjusting holo servos mechanical setup
 *  	00 or >3 all HPs to test
 *  *STxx stop/reset Holos random movement, turns lights off, and RC off. 00=all off
 *  *HDxx hold: stop holo, do not change light level. 00=all stopped
 *  *MOxx magic panel on. xx=01 to 98, on from 1 to 98 seconds. 00=off, 99=on permanent
 *  *MFxx magic panel flicker xx=1 to 99 flicker for 1 to 99 seconds. 00= off.
 *  *H1xx, *H2xx, *H3xx, and *H0xx
 * 		Will turn on-board HP1, 2, 3, and all (HP0xx) for xx seconds
 * 		99 is on permanently.
 * 		0 is off.
 * 	*F1xx, *F2xx, *F3xx, and *F0xx
 * 		Will flicker on-board HP1, 2, 3, and all (F0xx) for xx seconds
 * 		0 is off.
 * 		(99 is not permanently, but 99 seconds)
 *
 *	Setup Commands
 *	//// SERVO CONTROLS
 *	#SD00 Set Servo direction forward
 *	#SD01 Set servo direction reversed
 *	#SRxxy Set individual servo to either forward or reversed xx=servo number y=direction
 *		Must be a 2 digit Servo number i.e. Servo 4 is 04
 *		Must be either 0 or 1 to set the direction (0 normal, 1 reversed)
 *		Use SDxx to globally set the Servo direction, then SRxxy to change individual servos.*
 */

#include "main.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>  // needed for the UART interrupt
#include <avr/pgmspace.h>	// for the sequencer data arrays defined with PROGMEM
#include <avr/eeprom.h>		// for state save and setup mode

#include <stdlib.h>			// for itoa(), integer to string conversion
#include <errno.h>			// for errors on atoi (check errno)
#include <stdio.h>			// for sprintf(), don't use if you are short on memory
#include <string.h>			// for strlen()


#include "toolbox.h"		// map, digitalWrite, digitalMode
#include "servo.h"			// servo drivers
#include "realtime.h"		// real time interrupt services
#include "serial.h"			// hardware serial
#include "suart.h"			// software serial (write only)
#include "wmath.h"			// random functions
#include "sequencer.h"		//  13th Servo Sequences
#include "panel_sequences.h"	//  13 servo panel sequences

#ifdef _MARCDUINOV2_
#include "i2c.h"			// include I2C Master libraries for MarcDuino v2
#endif

#define HP_NUM 3			// controlling 3 HPs for RC

// command globals
char command_buffer[CMD_MAX_LENGTH];	// command string buffer
uint8_t panel_rc_control[SERVO_NUM-2];		// flag array for which panels are under RC control (Exclude the panel servos)
uint8_t panel_to_silence[SERVO_NUM];		// flag array for servos we need to turn off after a panel is closed

//setup globals
/* E2END : last valid address in Internal EEPROM */
unsigned int servo_eeprom_addr = 0; // This is a word.
// unsigned int servo_eeprom_addr2 = 1;
//unsigned int start_sound_eeprom_addr=2; // Not used in Slave
unsigned int last_servo_addr=3; // only used for CRC
unsigned int stored_crc_addr = 6; // Uses a word.

#ifdef OLD_MP_CONTROL
uint8_t mp_command_args=0;
char* mpcmd="%MP";
char* mp_cmd_val="";
#endif

//setup for ext1 control
uint8_t ext1_command_args=0;
char* ext1cmd="%EX";
char* ext1_cmd_val="";


 // Controls verbosity level - affecting i2c for now
 uint8_t errormessageon=_ERROR_MSG_;
 uint8_t feedbackmessageon=_FEEDBACK_MSG_;

// timeout counter
rt_timer killbuzz_timer;

// flags for the HP control mode (0=nothing, 1=random, 2=RC)
static uint8_t hp1_control=0;
static uint8_t hp2_control=0;
static uint8_t hp3_control=0;
// Magic panel and holo effect depends on the value of the following global flags:
// if it's 0, it's off
// if it's 1, it's on
// if it's 2, it's on as long as rt_timeout_magic_panel is non-zero
// if it's 3, random flicker as long as rt_timeout_magic_panel is non-zero
#ifdef OLD_MP_CONTROL
static uint8_t magic_control=0;
#endif
static uint8_t hp1_light_control=0;
static uint8_t hp2_light_control=0;
static uint8_t hp3_light_control=0;

// ext1 Pin Control.
// Ext1 will be held high for the given duration
static uint8_t ext1_control=0;

// MarcDuino V2, holo states for I2C
// set them to 1 initially or they won't turn off during init...
uint8_t holo1state=1;
uint8_t holo2state=1;
uint8_t holo3state=1;

// timers

rt_timer move_timer_HP1;		// HP1 random movement timer
rt_timer move_timer_HP2;		// HP2 random movement timer
rt_timer move_timer_HP3;		// HP3 random movement timer

#ifdef OLD_MP_CONTROL
rt_timer on_timer_magic;		// magic on-time timer
rt_timer flicker_timer_magic;	// magic panel flicker timer
#endif

// timer for the ext1 pin
rt_timer on_timer_ext1;

rt_timer on_timer_HP1;			// HP1 on-time timer
rt_timer on_timer_HP2;			// HP2 on-time timer
rt_timer on_timer_HP3;			// HP3 on-time timer
rt_timer flicker_timer_HP1; 	// HP1 flicker timer
rt_timer flicker_timer_HP2; 	// HP2 flicker timer
rt_timer flicker_timer_HP3; 	// HP3 flicker timer

rt_timer test_timer;			// For HP movement test

// string constants are in program memory to save DRAM
const char strOK[] PROGMEM="OK\n\r";
const char strWelcome[] PROGMEM="\n\rMarcDuino HP Control v3.6 \n\r";
const char strEnterPrompt[] PROGMEM="Enter HP or Display command starting with \'*\' or \'@\'\n\r";
const char strSuart1OK[] PROGMEM="\n\rsuart1 Communication OK \n\r";
const char strSuart2OK[] PROGMEM="\n\rsuart2 Communication OK \n\r";
const char strStartCharErr[] PROGMEM="**Unrecognized Command Start Character\r\n";

// utility to echo characters back cleanly
void echo(char ch)
{
	// echo return and line feeds nicely on a terminal
	if(ch=='\r' || ch=='\n' || ch == 0x0D )
	{
		serial_putc('\n');
		serial_putc('\r');
	}
	else serial_putc(ch);
}

int main(void) {

	// start hardware and software UARTs, send check string
	serial_init_9600b8N1();	// 9600 bauds, 8 bits, 1 stop, no parity, use for a regular terminal console
	serial_puts_p(strWelcome);
	serial_puts_p(strEnterPrompt);

// initialize suart used to communicate with the JEDI Display at 2400 or 9600 bauds
	uint16_t baudrate;
#ifdef _9600BAUDSJEDI_
	baudrate=9600;
#else
	baudrate=2400;
#endif
	lightsuart_init(baudrate);


	//suart_puts_p(strSuart1OK);
	//_delay_ms(200);

// suart used for slave out initialized at 9600 bauds, used for Alt2 commands
	slavesuart_init(9600);
#ifdef _MARCDUINOV2_
	slavesuart_puts_p(strSuart1OK); // on suart1 for MarcDuino v2
#else
	slavesuart_puts_p(strSuart2OK);	// on suart2 for MarcDuino v1
#endif


	// abort test routine, and optionally do extra Jedi programmatic setup
	_delay_ms(3000);		// wait that the display system is up
	init_jedi();

	// Received command characters from uart0 are processed asynchronously via interrupt handler
	// further below. They will be used to fill the 'command_buffer' string.
	// The 'command_buffer_full' flag will be raised when the full command is available.
	// The buffer is then read and processed in the main loop.
	serial_enable_rx_interrupt();

	// Read the EEPROM to set the Servo direction
	// Note we'll only read it if the EEPROM is ready.
	while (!eeprom_is_ready())
	{
		// Just wait ... should be ready soon
		// Should be ready instantly, so this will likely not be called.
		_delay_ms(500);
	}

	// Calculate the CRC for the EEPROM to determine if it has been written or not.
	uint16_t calculatedCRC = calc_crc();

	// Get the current CRC from the EEPROM
	uint16_t storedCRC = eeprom_read_word((uint16_t*)stored_crc_addr);

	char string[128];
	sprintf(string, "Calc CRC: %X StoredCRC: %X \r\n", calculatedCRC, storedCRC);
	serial_puts(string);

	// If we've never written data to the EEPROM, the values will be 0xFF
	// Use this and the CRC to determine if we should save a set of defaults.
	if (storedCRC != calculatedCRC)
	{
		sprintf(string, "EEPROM Corrupt or never written.  Writing defaults. \r\n");
		serial_puts(string);

		// We have either corrupted the EEPROM, or
		// we've never written to it before.
		// Regardless, let's write good values!

		// Set the Servo direction to "normal"
		eeprom_write_word((uint16_t*)servo_eeprom_addr, (uint16_t)0x0000);

		//Set the last servo number (this is just to give something else for the CRC
		// it is not currently used.
		eeprom_write_byte((uint8_t*)last_servo_addr, 7);

		sprintf(string, "Generating new CRC. \r\n");
		serial_puts(string);

		// Now that we have written the new values we need to write the CRC
		calculatedCRC = calc_crc();
		sprintf(string, "Calc CRC: %X StoredCRC: %X \r\n", calculatedCRC, storedCRC);
		serial_puts(string);

		eeprom_write_word((uint16_t*)stored_crc_addr, calculatedCRC);

		sprintf(string, "New Defaults and CRC Written to EEPROM \r\n");
		serial_puts(string);
	}


	// If Servo Direction is 0, then it's "forward servos"
	// If Servo Direction is 1, then it's "reverse servos"
	// The Setup Mode will set this byte in the EEPROM

	// Set the servos forward/reverse direction
	uint16_t servo_dir = eeprom_read_word((uint16_t*)servo_eeprom_addr);
	//char string[30];
	for (int i=0; i<SERVO_NUM; i++)
	{
		uint8_t temp;
		if (servo_dir & (1<<i))
		{
			temp  = 1;
		}
		else
		{
			temp = 0;
		}
		//sprintf(string, "Bitmask: %4X Servo %2d, direction %2d \r\n", servo_dir, i+1, temp);
		//serial_puts(string);
		servo_direction[i] = temp;
	}


	//last_servo = eeprom_read_byte((uint8_t*)last_servo_addr);

	// initialize servo and realtime units
	servo_init();
	realtime_init();
	seq_init();

#ifdef _MARCDUINOV2_
	// initialize I2C hardware on MarcDuino v2's with 10k pull-up resistors on.
	i2c_init(TRUE);
#endif

	// register our buzz kill timer
	rt_add_timer(&killbuzz_timer);

	// register all our timers and timeouts

	rt_add_timer(&move_timer_HP1);
	rt_add_timer(&move_timer_HP2);
	rt_add_timer(&move_timer_HP3);
#ifdef OLD_MP_CONTROL
	rt_add_timer(&flicker_timer_magic);
	rt_add_timer(&on_timer_magic);
#endif
	rt_add_timer(&on_timer_HP1);
	rt_add_timer(&on_timer_HP2);
	rt_add_timer(&on_timer_HP3);
	rt_add_timer(&flicker_timer_HP1);
	rt_add_timer(&flicker_timer_HP2);
	rt_add_timer(&flicker_timer_HP3);
	rt_add_timer(&test_timer);
	rt_add_timer(&on_timer_ext1);

	// initialize magic panel and holo lights as output
#ifdef OLD_MP_CONTROL
	digitalMode(MAGIC_PORT, MAGIC_PIN, OUTPUT);
#endif
	digitalMode(HP1_LIGHT_PORT, HP1_LIGHT_PIN, OUTPUT);
	digitalMode(HP2_LIGHT_PORT, HP2_LIGHT_PIN, OUTPUT);
	digitalMode(HP3_LIGHT_PORT, HP3_LIGHT_PIN, OUTPUT);
	// initialise the ext1 as output
	digitalMode(EXT1_PORT, EXT1_PIN, OUTPUT);

	// turn off all lights
	holo1OFF();
	holo2OFF();
	holo3OFF();
#ifdef OLD_MP_CONTROL
	MAGIC_OFF;
#endif

	// Default state for the EXT1 Pin is "off" i.e. not triggered
	EXT1_OFF;

	// run a close sequence on the panels to make sure they are all shut
	seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
	seq_startsequence();

  while (1)
  {
	/////////////////////////////////////////
	// Serial Command Input
	////////////////////////////////////////
	char command_str[CMD_MAX_LENGTH];
	bool command_available;

	// check for command line input
	if(serial_available())
	{
		char ch;
		ch=serial_getc();										// get input
		echo(ch);												// echo back
		command_available=build_command(ch, command_str);		// build command line
		if (command_available) dispatch_command(command_str);	// send command line to dispatcher
	}

	///////////////////////////////////////////
	// RC and random pattern of HP servos
	//////////////////////////////////////////

	 // How the servo is control depends on hpx_control flags
	 // if it's 0, nothing happens
	 // if it's 1, random pattern
	 // if it's 2, RC control of vertical servo

	 // *********  HP RC movement  ************************

	 if(hp1_control==2) {servo_set(FRONT_HP_SERVO_H, 1500); servo_set(FRONT_HP_SERVO_V, servo_RCread());}
	 if(hp2_control==2) {servo_set(REAR_HP_SERVO_H, 1500); servo_set(REAR_HP_SERVO_V, servo_RCread());}
	 if(hp3_control==2) {servo_set(TOP_HP_SERVO_H, 1500); servo_set(TOP_HP_SERVO_V, servo_RCread());}

	 // *********  HP random movement  ************************
	 if(hp1_control==1)
	 {
		 if(move_timer_HP1==0)
		 {
			 //move_timer_HP1 = rand_howsmall_howbig(50, 500);
			 move_timer_HP1 = rand_howsmall_howbig(20, 200);
			 servo_set(FRONT_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(FRONT_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 seq_setspeed(FRONT_HP_SERVO_H, random_howsmall_howbig(1,25));
			 seq_setspeed(FRONT_HP_SERVO_V, random_howsmall_howbig(1,25));
		 }
	 }
	 if(hp2_control==1)
	 {
		 if(move_timer_HP2==0)
		 {
			 //move_timer_HP2 = rand_howsmall_howbig(50, 500);
			 move_timer_HP2 = rand_howsmall_howbig(20, 200);
			 servo_set(REAR_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(REAR_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 seq_setspeed(REAR_HP_SERVO_H, random_howsmall_howbig(1,25));
			 seq_setspeed(REAR_HP_SERVO_V, random_howsmall_howbig(1,25));
		 }
	 }
	 if(hp3_control==1)
	 {
		 if(move_timer_HP3==0)
		 {
			 //move_timer_HP3 = rand_howsmall_howbig(50, 500);
			 move_timer_HP3 = rand_howsmall_howbig(20, 200);
			 servo_set(TOP_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(TOP_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 seq_setspeed(TOP_HP_SERVO_H, random_howsmall_howbig(1,25));
			 seq_setspeed(TOP_HP_SERVO_V, random_howsmall_howbig(1,25));
		 }
	 }

	 // *********  HP test movement  ************************
	 // move servos to all extreme positions to adjust HP
	 if(hp1_control==3 || hp2_control==3 || hp3_control==3)
	 {
		 if(test_timer==0)
		 {
			 static uint8_t step;
			 static uint16_t a,b;
			 if (step>=17)step=0;
			 step ++;
			 test_timer = 200;
			 switch (step)
			 {
			 	 case 1: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 2: // up
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 3: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 4: // down
				 	a= (SERVO_HP_MIN+SERVO_HP_MAX)/2;
				 	b= SERVO_HP_MIN;
			 		break;
			 	 case 5: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 6: // right
			 		a= SERVO_HP_MAX;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 7: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 8: // left
			 		a= SERVO_HP_MIN;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 9: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 10: // up/right
			 		a= SERVO_HP_MAX;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 11: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 12: // down/right
			 		a= SERVO_HP_MAX;
			 		b= SERVO_HP_MIN;
			 		break;
			 	 case 13: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 14: // down/left
			 		a= SERVO_HP_MIN;
			 		b= SERVO_HP_MIN;
			 		break;
			 	 case 15: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 16: // up/left
			 		a= SERVO_HP_MIN;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 17: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 default:
			 		 break;
			 }
			 if(hp1_control==3)
			 	 {servo_set(FRONT_HP_SERVO_H, a); servo_set(FRONT_HP_SERVO_V, b);}
			 if(hp2_control==3)
			 	 {servo_set(REAR_HP_SERVO_H, a); servo_set(REAR_HP_SERVO_V, b);}
			 if(hp3_control==3)
			 	 {servo_set(TOP_HP_SERVO_H, a); servo_set(TOP_HP_SERVO_V, b);}
		 }
	 }

#ifdef OLD_MP_CONTROL
	 ///////////////////////////////////////////////
	 // Magic panel control
	 //////////////////////////////////////////////

	 // Magic panel effect depends on the 'magic_control' flag:
	 // Control 0-3 uses pin 3 on the MP and toggles to create effects
	 // Effects are generated by the MarcDuino.
	 // if it's 0, it's off
	 // if it's 1, it's on
	 // if it's 2, it's on as long as rt_timeout_magic_panel is non-zero
	 // if it's 3, random flicker as long as rt_timeout_magic_panel is non-zero
	 // if it's 4, Send the command via serial to the MP, and the MP manages it

	 switch(magic_control)
	 {
	 	 case 0:
	 		 MAGIC_OFF;
	 		 break;
	 	 case 1:
	 		 MAGIC_ON;
	 		 break;
	 	 case 2:
	 		 if(on_timer_magic) MAGIC_ON;
	 		 else MAGIC_OFF;
	 		 break;
	 	 case 3:
	 		 if(!on_timer_magic) MAGIC_OFF;
	 		 else
	 		 {
	 			 if(!flicker_timer_magic)	// end of random flicker period
	 			 {
	 				// Michael Wheeler magic panel has reversed polarity
					#ifdef _REVERSEMAGICPANEL_
	 				if(!digitalRead(MAGIC_PORT,MAGIC_PIN)) // panel was on
					#else
	 				if(digitalRead(MAGIC_PORT,MAGIC_PIN)) // panel was on
					#endif
	 				{
	 					flicker_timer_magic=rand_howsmall_howbig(3,10);
	 					MAGIC_OFF;
	 				}
	 				else
	 				{
	 					flicker_timer_magic=rand_howsmall_howbig(5,20);
	 					MAGIC_ON;
	 				}
	 			 }
	 		 }
	 		 break;
	 	 default:
	 		 break;
	 }
#endif

	 ///////////////////////////////////////////////
	 // EXT1 Pin control
	 //////////////////////////////////////////////

	 // EXT1 PIN effect depends on the 'EXT1_control' flag:
	 // This toggles the pin high/low to enable an output on the pin to trigger
	 // Perfect example would be a smoke machine trigger.
	 // Effects are generated by the MarcDuino.
	 // if it's 0, it's off
	 // if it's 1, it's on
	 // if it's 2, it's on as long as rt_timeout_ext1 is non-zero

	 switch(ext1_control)
	 {
	 	 case 0:
	 		 EXT1_OFF;
	 		 break;
	 	 case 1:
	 		 EXT1_ON;
	 		 break;
	 	 case 2:
	 		 if(on_timer_ext1) EXT1_ON;
	 		 else EXT1_OFF;
	 		 break;
	 	 default:
	 		 break;
	 }

	 ///////////////////////////////////////////////
	 // HP Light control
	 //////////////////////////////////////////////

	 // HP light mode depends on the 'hpx_light_control' flag:
	 // if it's 0, it's off
	 // if it's 1, it's on
	 // if it's 2, it's on as long as on_timer_HPx is non-zero
	 // if it's 3, random flicker as long as on_timer_HPx is non-zero


	 switch(hp1_light_control)
	 {
		 case 0:
			 holo1OFF();
			 break;
		 case 1:
			 holo1ON();
			 break;
		 case 2:
			 if(on_timer_HP1) holo1ON();
			 else holo1OFF();
			 break;
		 case 3:
			 if(!on_timer_HP1) holo1OFF();
			 else
			 {
				 if(!flicker_timer_HP1)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP1_LIGHT_PORT,HP1_LIGHT_PIN))
					#else
					if(digitalRead(HP1_LIGHT_PORT,HP1_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP1=rand_howsmall_howbig(3,10);
						holo1OFF();
					}
					else
					{
						flicker_timer_HP1=rand_howsmall_howbig(5,20);
						holo1WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	 switch(hp2_light_control)
	 {
		 case 0:
			 holo2OFF();
			 break;
		 case 1:
			 holo2ON();
			 break;
		 case 2:
			 if(on_timer_HP2) holo2ON();
			 else holo2OFF();
			 break;
		 case 3:
			 if(!on_timer_HP2) holo2OFF();
			 else
			 {
				 if(!flicker_timer_HP2)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP2_LIGHT_PORT,HP2_LIGHT_PIN))
					#else
					if(digitalRead(HP2_LIGHT_PORT,HP2_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP2=rand_howsmall_howbig(3,10);
						holo2OFF();
					}
					else
					{
						flicker_timer_HP2=rand_howsmall_howbig(5,20);
						holo2WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	 switch(hp3_light_control)
	 {
		 case 0:
			 holo3OFF();
			 break;
		 case 1:
			 holo3ON();
			 break;
		 case 2:
			 if(on_timer_HP3) holo3ON();
			 else holo3OFF();
			 break;
		 case 3:
			 if(!on_timer_HP3) holo3OFF();
			 else
			 {
				 if(!flicker_timer_HP3)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP3_LIGHT_PORT,HP3_LIGHT_PIN))
					#else
					if(digitalRead(HP3_LIGHT_PORT,HP3_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP3=rand_howsmall_howbig(3,10);
						holo3OFF();
					}
					else
					{
						flicker_timer_HP3=rand_howsmall_howbig(5,20);
						holo3WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	/***** simple debug RC input test: loopback input to servo */
	//servo_set(2, servo_RCread());

	/***** simple input RC diagnostic test: prints out the RC input value
	char string[17]; 				// put outside while loop?
	sprintf(string, "RC-in: %04d \r\n", servo_RCread());
	serial_puts(string);
	*/

	// kill servo buzz if panel have been marked as just closed and the timeout period has expired
	if(killbuzz_timer==0)
	{
		for(int i=1; i<=SERVO_NUM; i++)
		{
			if(panel_to_silence[i-1])
			{
				servo_set(i,_NP);
				panel_to_silence[i-1]=0;
			}
		}
	}

  }
return 0;
}

/*
 * Calculate the CRC based on the values we use in the EEPROM
 */
uint16_t calc_crc()
{
	uint16_t calculatedCRC = 0;
	// Set the Servo direction to "normal"
	calculatedCRC += eeprom_read_word((uint16_t*)servo_eeprom_addr);
	calculatedCRC += eeprom_read_byte((uint8_t*)last_servo_addr);

	return calculatedCRC;
}

// builds the command string from the character input
uint8_t build_command(char ch, char* output_str)
{
	static uint8_t pos=0;

	switch(ch)
	{
		case CMD_END_CHAR:								// end character recognized
			command_buffer[pos]='\0';					// append the end of string character
			pos=0;										// reset buffer pointer
			strcpy(output_str, (char*)command_buffer);	// copy result
			return TRUE;								// return and signal command ready
			break;

		default:										// regular character
			command_buffer[pos]=ch;						// append the  character to the command string
			if(pos<=CMD_MAX_LENGTH-1)pos++;				// too many characters, discard them.
			break;
	}
	return FALSE;
}

// dispatches further command processing depending on start character
void dispatch_command(char* command_str)
{
	char start_char=command_str[0];
	uint8_t length=strlen(command_str);

	// prompt on empty command to show life at console
	if(length==0)
	{
		serial_puts_p(strOK);
		return;
	}

	// dispatch the command to the appropriate parser depending on start character
	switch(start_char)
	{
	 case PANEL_START_CHAR:
		 parse_panel_command(command_str, length);
		 break;
	 case HP_START_CHAR:
		 parse_hp_command(command_str, length);
		 break;
	 case DISPLAY_START_CHAR:
		 parse_display_command(command_str,length);
		 break;
	 case SOUND_START_CHAR:
		parse_sound_command(command_str,length);
		break;
	 case ALT1_START_CHAR:
		parse_alt1_command(command_str,length);
		break;
	 case ALT2_START_CHAR:
		parse_alt2_command(command_str,length);
		break;
	 case I2C_START_CHAR:
		 parse_i2c_command(command_str,length);
		 break;
	 case SETUP_START_CHAR:
		 parse_setup_command(command_str, length);
		 break;
	 default:
		 if(errormessageon) serial_puts_p(strStartCharErr);
		 break;
	}
}

// New Setup and Configuration functions.
const char strsetupCommand[] PROGMEM="Setup command\r\n";
void parse_setup_command(char* command, uint8_t length)
{
	if(feedbackmessageon) serial_puts_p(strsetupCommand);

	char cmd[3];
	char arg[4];

	// character '*' begins command, should have been already checked if this is called
	if(command[0]!=SETUP_START_CHAR)
	{
		if(errormessageon) serial_puts_p("Err Setup Cmd\n\r");
		return;
	}

	cmd[0]=command[1];
	cmd[1]=command[2];
	cmd[2]='\0';
	arg[0]=command[3];
	arg[1]=command[4];
	arg[2]=command[5];
	arg[3]='\0';

	// Get the value of the command
	uint8_t value;
	value=atoi(arg);

	if(strcmp(cmd,SETUP_SERVO_DIR)==0)
		{

			// a properly constructed command should have 5 chars
			//if (length!=5)
			//{
			//	if(errormessageon) serial_puts_p("Err Setup Cmd\n\r");
			//	return;
			//}

			//Value must be either 0 or 1
			if (value == 0)
			{
				//Write the value to EEPROM
				eeprom_write_word((uint16_t*)servo_eeprom_addr, (uint16_t)0x0000);
			}
			if (value == 1)
			{
				//Write the value to EEPROM
				eeprom_write_word((uint16_t*)servo_eeprom_addr, (uint16_t)0x07FF);
			}
			// Now that we have written the new values we need to write the CRC
			uint16_t calculatedCRC = calc_crc();
			eeprom_write_word((uint16_t*)stored_crc_addr, calculatedCRC);
			serial_puts_p(strOK);
			return;
		}
		if(strcmp(cmd,SETUP_SERVO_REVERSE)==0)
		{

			// a properly constructed command should have 5 chars
			//if (length!=6)
			//{
			//	if(errormessageon) serial_puts_p("Err Setup Cmd\n\r");
			//	return;
			//}

			// Get the servo settings from the command
			uint8_t servo_number = value/10;
			uint8_t servo_value_received = value%10;

			/*
			char string[30];
			sprintf(string, "Servo %2d, direction %2d \r\n", servo_number, servo_value_received);
			serial_puts(string);

			// DEBUG ... read the current servo values.
			for (int i=0; i<SERVO_NUM; i++)
			{
				uint8_t temp = servo_direction[i];
				sprintf(string, "Servo %2d, direction %2d \r\n", i+1, temp);
				serial_puts(string);
			}
			*/

			// Set the Servo Number to the requested reverse setting
			servo_direction[servo_number-1] = servo_value_received;
			if (servo_value_received == 1)
			{
				servo_direction[servo_number-1] = 1;
			}
			else if (servo_value_received == 0)
			{
				servo_direction[servo_number-1] = 0;
			}

			/*
			serial_puts("Changed to: \r\n");
			for (int i=0; i<SERVO_NUM; i++)
			{
				uint8_t temp = servo_direction[i];
				sprintf(string, "Servo %2d, direction %2d \r\n", i+1, temp);
				serial_puts(string);
			}
			*/


			// Loop through the array, and set the bits to store in EEPROM
			uint16_t servo_set = 0x0000;
			for (int i=0; i< SERVO_NUM; i++)
			{
				// Set the bit
				if (servo_direction[i])
				{
					servo_set |= 1 << i;
				}
				else
				{
					servo_set &= ~(1 << i);
				}
			}

			//Write the value to EEPROM
			eeprom_write_word((uint16_t*)servo_eeprom_addr, (uint16_t)servo_set);
			// Now that we have written the new values we need to write the CRC
			uint16_t calculatedCRC = calc_crc();
			eeprom_write_word((uint16_t*)stored_crc_addr, calculatedCRC);
			serial_puts_p(strOK);
			return;
		}
	if(strcmp(cmd,SETUP_LAST_SERVO)==0)
	{
		//Max support is for 12 servos.
		if (value < 13)
		{
			//Write the value to EEPROM
			eeprom_write_byte((uint8_t*)last_servo_addr, value);
			// Now that we have written the new values we need to write the CRC
			uint16_t calculatedCRC = calc_crc();
			eeprom_write_word((uint16_t*)stored_crc_addr, calculatedCRC);
		}
		serial_puts_p(strOK);
		return;
	}

	if(errormessageon) serial_puts_p("Err Setup Cmd\n\r");
}


const char strPanelCommandIgnored[] PROGMEM="Panel command - ignored\r\n";
const char strPanelCmdErr[] PROGMEM="**Invalid Panel Command\r\n";
void parse_panel_command(char* command_string, uint8_t length)
{
	//HAHA ... now we control the 13th servo!
	/************************************
	 * Command vocabulary
	 * : as start command sign
	 * :SExx as play sequence
	 * 	argument: 00 01 ... 10 sequence number
	 * :OPxx as open panel:
	 *  argument 01...10 as panel number
	 *  00 = all panels
	 *  11 = top panels
	 *  12 = bottom panels
	 * :CLxx as close panel, remove RC, kill servo
	 *  argument 01...10 as panel number
	 *  00 = close all panels using a slow close sequence
	 * :RCxx as listen to RC
	 * 	argument 00 -> move all panels
	 *  argument 01 ... 10 -> move corresponding panels
	 * :STxx as stop (soft hold/buzz kill)
	 * 	Removes RC, and kill servo power
	 * 	00=all panels servo off
	 * :HDxx as hold (hard hold)
	 * 	Removes RC, hold the servo to last position
	 * 	00=all panels that had RC will hold
	 *
	 */

	char cmd[3];
	char arg[3];

	// character ':' begins command, just double check, should not happen
	if(command_string[0]!=PANEL_START_CHAR)
	{
		if(errormessageon) serial_puts_p(strPanelCmdErr);
		return;
	}

	cmd[0]=command_string[1];
	cmd[1]=command_string[2];
	cmd[2]='\0';
	arg[0]=command_string[3];
	arg[1]=command_string[4];
	arg[2]='\0';

	process_command(cmd, arg);

}

const char strDisplayCommandForwarded[] PROGMEM="Display command - forwarded to display\r\n";
void parse_display_command(char* command, uint8_t length)
{
	// forward the command minus the @ character to the JEDI display
	if(feedbackmessageon) serial_puts_p(strDisplayCommandForwarded);
	if(length>=3)	// command must have at least a start, a character and an end
	{
		lightsuart_puts(command+1); // discard the start character, JEDI display doesn't use any
		lightsuart_putc('\r');		// add the termination character
	}
}

const char strSoundCommandIgnored[] PROGMEM="Sound command - ignored\r\n";
void parse_sound_command(char* command,uint8_t length)
{
	// we should not receive any of this, ignore
	if(feedbackmessageon) serial_puts_p(strSoundCommandIgnored);
}

// for custom/future expansion,
// Forward command to Lights with start character dropped
// However in normal use the Master MarcDuino catches these, so we don't see them
const char strAlt1Command[] PROGMEM="Alt1 command, output to lights\r\n";
void parse_alt1_command(char* command, uint8_t length)
{
	if(feedbackmessageon) serial_puts_p(strAlt1Command);

	lightsuart_puts(command+1); // discard the start character
	lightsuart_putc('\r');		// add the termination character
}

// for custom/future expansion
// Forward command to next slave after stripping
// These were already forwarded to us in the same manner by the master MarcDuino
const char strAlt2Command[] PROGMEM="Alt2 command, output to slave out\r\n";
void parse_alt2_command(char* command, uint8_t length)
{
	if(feedbackmessageon) serial_puts_p(strAlt1Command);

	slavesuart_puts(command+1);  // discard the start character
	slavesuart_putc('\r');		// add the termination character
}


const char strHPCmdErr[] PROGMEM="**Invalid Panel Command\r\n";
void parse_hp_command(char* command_string, uint8_t length)
{
	/************************************
	 * Command vocabulary
	 * * as start command sign
	 * ST as STop holo
	 * 	argument: 01 02 03 holo number, 00= all holos
	 * RD as RanDom sequence:
	 *  01 02 03 holo number, 00= all holos
	 * ON as turn HP ON
	 *  01 02 03 holo number, 00= all holos
	 * OF as turn HP OFF
	 * RC as control HP (vertical) via RC
	 * 	01 02 03 holo number,
	 * 	00= no RC control
	 *  >03 control all HPs
	 * TE as test HP
	 * 	01 02 03 holo number,
	 *  00= no test
	 */

	char cmd[3];
	char arg[3];

	// a properly constructed command should have 5 chars
	if (length!=5)
	{
		if(errormessageon) serial_puts_p(strHPCmdErr);
		return;
	}

	// character '*' begins command, should have been already checked if this is called
	if(command_string[0]!=HP_START_CHAR)
	{
		if(errormessageon) serial_puts_p(strHPCmdErr);
		return;
	}

	cmd[0]=command_string[1];
	cmd[1]=command_string[2];
	cmd[2]='\0';
	arg[0]=command_string[3];
	arg[1]=command_string[4];
	arg[2]='\0';

	process_command(cmd, arg);
}

const char strI2CCmdErr[] PROGMEM="**Invalid I2C Command\r\n";
void parse_i2c_command(char* cmd, uint8_t length)
{
	/*****************************************
	 * I2C Command structure:
	 *
	 * &device_address(in decimal),<arg>,<arg>,<arg>\r
	 *
	 * Where <arg> is to be sent on i2c as a byte according to
	 * one of four converions:
	 * 	decimal number: 210 (no prefix, max 255, min -128)
	 * 	hex number: xA7 (x prefix, max FF)
	 * 	single char: 'c' (' prefix, optional ' at the end)
	 * 	ASCII string:"hello world (" prefix, no other " at the end)
	 *
	 * Note that numbers are one byte max. To send larger numbers
	 * you have to break them in hex bytes and send them one byte
	 * at a time
	 *
	 * To debug it is useful to set #define _FEEDBACK_MSG_ 1
	 * in main.h, it shows how it interpreted the command payload
	 * on the serial console output
	 *
	 */

	uint8_t i2caddress=0;
	uint8_t payload[256];
	uint8_t payloadIndex=0;
	uint8_t success=0;
	const char delim[]=",";
	char* token;
	//### for debug output
	char str[65];

	// a properly constructed command should have at least 2 chars
	if (length<2)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 1\n\r");
#endif
		return;
	}

	// check first character '&' begins command
	if(cmd[0]!=I2C_START_CHAR)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 2\n\r");
#endif
		return;
	}

	// good enough to send on to the next slave
	// so all slaves execute the same I2C command
	suart2_puts(cmd);
	suart2_putc('\r');	// add the termination character

	// get the address field. Need to tokenize on the next "," or "\0"
	token = strtok(cmd+1, delim);
	if(token == NULL )
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 3\n\r");
#endif
		return;
	}

	// convert and check the address
	unsigned int temp;
	success=sscanf(token, "%u", &temp);
	//i2caddress=atoi(token);
	// make sure I can do the conversion to uint8_t
	if(temp<255) i2caddress=(uint8_t)temp;
	else success=0;

	//### confirm first address token is read correctly
	if (success) sprintf(str, "Token: %s, recognized address: %u \r\n", token, i2caddress);
	else sprintf(str, "Token: %s, unrecognized address\r\n", token);
	serial_puts(str);

	if(i2caddress > 127 || !success)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 4\n\r");
#endif
		return;
	}

	// get all arguments separated by commas
	while(token!=NULL)
	{
		token=strtok(NULL, delim); 	// get next token
		if(token==NULL) break;		// exit immediately if token no good

#if _FEEDBACK_MSG_ == 1	// verify token
		serial_puts("Token: "); serial_puts(token); serial_puts("\r\n");
#endif

		// try to convert and append to payload
		success=append_token(payload, &payloadIndex, token);

#if _FEEDBACK_MSG_ == 1 // verify payload
		if(success) serial_puts("Data Good - ");
		else serial_puts("Data Error - ");
		sprintf(str, "Index = %u \r\n", payloadIndex);
		serial_puts(str);
#endif

		//break immediately on payload error
		if(!success) break;
	}

	// send the I2C command if good payload
	if(success && payloadIndex!=0)	sendI2C(i2caddress, payload, payloadIndex);
	else
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 5\n\r");
#endif
	}
}

uint8_t append_token(uint8_t* payload, uint8_t* index, char* token)
{
	uint8_t result=0;
	unsigned int unum;
	int num;
	char ch;
	uint8_t i;
	switch(token[0])
	{
		case 'x':	// hex character
			result=sscanf(token+1, "%x", &unum); // skip the x and read the hex number
			if(result)
			{
				if(unum>255) return 0; // limited to 8 bit hex values
				payload[*index]=(uint8_t)unum;
				(*index)++;
				if (*index==0) return 0; // force error on max payload overrun
			}
			break;
		case '"':	// string
			i=1; 	// start after the "
			while(i<255)
			{
				ch=token[i];				// read all the characters in the token
				if(ch=='\0') break;			// end of string
				payload[*index]=ch;			// put in the payload
				(*index)++;					// advance payload index
				if (*index==0) return 0; 	// index wrapped around, exit with error
				i++;						// advance string index
			}
			result=1;
			break;
		case '\'':	// single character
			result=sscanf(token+1, "%c", &ch);
			if(result)
			{
				payload[*index]=ch;
				(*index)++;
				if (*index==0) return 0;
			}
			break;
		default:
			// I have problem here if I get a 16 bit int and it doesn't fit in an int8_t or uint8_t.
			// So I am reducing the allowed range to -128 / +255
			result=sscanf(token, "%d", &num);
			if(result)
			{
				if(num>255 || num<-128) return 0; 				// limited to 8 bit signed or unsigned arguments
				if(num<=127) payload[*index]=(int8_t)num;	// allow signed from -128 to 127
				else payload[*index]=(uint8_t)num;			// but allow unsigned numbers up to 255
				(*index)++;
				if (*index==0) return 0; // force error on max payload overrun
			}
			break;
	}
	return result;
 }

void sendI2C(uint8_t address, uint8_t* payload, uint8_t payload_length)
{

#if _FEEDBACK_MSG_ == 1 // ifdef it to save memory when not using
	serial_puts("### RESULT IS ####\r\n");
	char str[65];
	sprintf(str, "I2C address = %u \r\n", address);
	serial_puts(str);
	sprintf(str, "Payload length= %u \r\n", payload_length);
	serial_puts(str);
	serial_puts("Payload = \r\n");
	for(uint8_t i=0; i<payload_length; i++)
	{
		sprintf(str, "Byte %u = %u \r\n", i, payload[i]);
		serial_puts(str);
	}
	serial_puts("\r\n");
#endif
	// send the data via i2c
	i2c_send_data(address,payload, payload_length, TRUE);
}

void process_command(char* thecommand, char* theargument)
{
	// hex conversion example
	// result=(int)strtol(str,(char **)NULL,16);
	// for now use base10 value conversion
	uint8_t value;
	// char string[35];
	value=atoi(theargument);

	if(strcmp(thecommand,CMD_SEQUENCE )==0)
	{
		serial_puts_p(strOK);
		sequence_command(value);
		return;
	};

	if(strcmp(thecommand,CMD_OPEN )==0)
	{
		serial_puts_p(strOK);
		open_command(value);
		return;
	};
	if(strcmp(thecommand,CMD_CLOSE )==0)
	{
		serial_puts_p(strOK);
		close_command(value);
		return;
	};

	if(strcmp(thecommand,CMD_HOLD)==0)
	{
		serial_puts_p(strOK);
		hold_command(value);
		return;
	}

	if(strcmp(thecommand,CMD_STOP)==0)
	{
		serial_puts_p(strOK);
		stop_command(value);
		return;
	}

	if(strcmp(thecommand,CMD_HOLD )==0)
	{
		serial_puts_p(strOK);
		hold_command(value);
		return;
	};

	if(strcmp(thecommand,CMD_RANDOM )==0)
	{
		serial_puts_p(strOK);
		random_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ON)==0)
	{
		serial_puts_p(strOK);
		on_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_OFF)==0)
	{
		serial_puts_p(strOK);
		off_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_RC )==0)
	{
		serial_puts_p(strOK);
		rc_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_TEST )==0)
	{
		serial_puts_p(strOK);
		test_command(value);
		return;
	}
#ifdef OLD_MP_CONTROL
	if(strcmp(thecommand,CMD_MAGIC_ON )==0)
	{
		serial_puts_p(strOK);
		magic_on_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_MAGIC_FLICKER )==0)
	{
		serial_puts_p(strOK);
		magic_flicker_command(value);
		return;
	}
#endif
	if(strcmp(thecommand,CMD_EXT1_ON )==0)
	{
		serial_puts_p(strOK);
		ext1_on_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO1_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo1_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO2_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo2_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO3_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo3_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ALL_HOLO_FLICKER )==0)
	{
		serial_puts_p(strOK);
		all_holo_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO1_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo1_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO2_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo2_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO3_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo3_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ALL_HOLO_FLASH )==0)
	{
		serial_puts_p(strOK);
		all_holo_flash_command(value);
		return;
	}
	if(errormessageon) serial_puts_p(strHPCmdErr);
}

const char strHP1Stop[] PROGMEM="(HP1 stop) \r\n";
const char strHP2Stop[] PROGMEM="(HP2 stop) \r\n";
const char strHP3Stop[] PROGMEM="(HP3 stop) \r\n";
const char strHPAllStop[] PROGMEM="(All HPs  stop) \r\n";

void stop_command(uint8_t value)
{
	uint8_t i;

	switch(value)
	{
		case 1:
			serial_puts_p(strHP1Stop);
			hp1_control=0;
			off_command(1);
			servo_set(FRONT_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(FRONT_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 2:
			serial_puts_p(strHP2Stop);
			hp2_control=0;
			off_command(2);
			servo_set(REAR_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(REAR_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 3:
			serial_puts_p(strHP3Stop);
			hp3_control=0;
			off_command(3);
			servo_set(TOP_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(TOP_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 0:
		default:
			serial_puts_p(strHPAllStop);
			hp1_control=0;
			hp2_control=0;
			hp3_control=0;
			off_command(0);
			// stop the 3 servos
			for(i=1; i<=HP_NUM*2; i++)
			{
				servo_set(i, SERVO_NO_PULSE);
				panel_to_silence[i]=1;	// flag the panel to silence, will be caught in main loop
			}

			for(i=7; i<=SERVO_NUM; i++)
			{
				 servo_set(i,SERVO_NO_PULSE);
				 panel_to_silence[i-1]=1;	// flag the panel to silence, will be caught in main loop
			}
			killbuzz_timer=COUNT_PER_SECOND/3; // set a 1/3s timer
			break;
	}
}

const char strHP1Hold[] PROGMEM="(HP1 hold) \r\n";
const char strHP2Hold[] PROGMEM="(HP2 hold) \r\n";
const char strHP3Hold[] PROGMEM="(HP3 hold) \r\n";
const char strHPAllDHold[] PROGMEM="(All HPs  hold) \r\n";
void hold_command(uint8_t value)
{
	// like stop/reset, except just stops movement, does not turn off the lights

	uint8_t i;

	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Hold);
			hp1_control=0;
			servo_set(FRONT_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(FRONT_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Hold);
			hp2_control=0;
			servo_set(REAR_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(REAR_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Hold);
			hp3_control=0;
			servo_set(TOP_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(TOP_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDHold);
			hp1_control=0;
			hp2_control=0;
			hp3_control=0;
			// stop the 3 servos
			for(i=1; i<=HP_NUM*2; i++)
			{
				servo_set(i, SERVO_NO_PULSE);
			}
			for(i=7; i<=SERVO_NUM; i++)
			{
				 servo_set(i,SERVO_NO_PULSE);
			}
			break;
	}
}

const char strHP1Rand[] PROGMEM="(HP1 random) \r\n";
const char strHP2Rand[] PROGMEM="(HP2 random) \r\n";
const char strHP3Rand[] PROGMEM="(HP3 random) \r\n";
const char strHPAllDRand[] PROGMEM="(All HPs  random) \r\n";
void random_command(uint8_t value)
{
	switch(value)
	{

		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Rand);
			hp1_control=1;
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Rand);
			hp2_control=1;
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Rand);
			hp3_control=1;
			break;
		case 0:	// all holos into random movement mode
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDRand);
			hp1_control=hp2_control=hp3_control=1;
			break;
	}
}

const char strHP1On[] PROGMEM="(HP1 on) \r\n";
const char strHP2On[] PROGMEM="(HP2 on) \r\n";
const char strHP3On[] PROGMEM="(HP3 on) \r\n";
const char strHPAllDOn[] PROGMEM="(All HPs  on) \r\n";
void on_command(uint8_t value)
{
	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1On);
			hp1_light_control=1;
			lightsuart_puts("6T1\r"); // turn the light on (front)
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2On);
			hp2_light_control=1;
			lightsuart_puts("7T1\r"); // turn the light on (rear)
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3On);
			hp3_light_control=1;
			lightsuart_puts("8T1\r"); // turn the light on (top)
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDOn);
			hp1_light_control=1;
			hp2_light_control=1;
			hp3_light_control=1;
			lightsuart_puts("6T1\r"); // turn the light on (front)
			_delay_ms(100);
			lightsuart_puts("7T1\r"); // turn the light on (rear)
			_delay_ms(100);
			lightsuart_puts("8T1\r"); // turn the light on (top)
			break;
	}
}

const char strHP1Off[] PROGMEM="(HP1 off) \r\n";
const char strHP2Off[] PROGMEM="(HP2 off) \r\n";
const char strHP3Off[] PROGMEM="(HP3 off) \r\n";
const char strHPAllDOff[] PROGMEM="(All HPs  off) \r\n";
void off_command(uint8_t value)
{
	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Off);
			hp1_light_control=0;
			lightsuart_puts("6D\r"); // turn the light off (front)
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Off);
			hp2_light_control=0;
			lightsuart_puts("7D\r"); // turn the light off (rear)
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Off);
			hp3_light_control=0;
			lightsuart_puts("8D\r"); // turn the light off (top)
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDOff);
			hp1_light_control=0;
			hp2_light_control=0;
			hp3_light_control=0;
			lightsuart_puts("6D\r"); // turn the light on (front)
			_delay_ms(100);
			lightsuart_puts("7D\r"); // turn the light on (rear)
			_delay_ms(100);
			lightsuart_puts("8D\r"); // turn the light on (top)
			break;
	}
}

const char strHP1RC[] PROGMEM="(HP1 RC) \r\n";
const char strHP2RC[] PROGMEM="(HP2 RC) \r\n";
const char strHP3RC[] PROGMEM="(HP3 RC) \r\n";
const char strHPAllRC[] PROGMEM="(All HPs RC) \r\n";
void rc_command(uint8_t value)
{

	switch(value)
	{
		// release control of all the HP that were under RC (control value=2)
		// turn servos and light off.

		// add the hp in question to RC control, turn the light on.
		case 1:
			hp1_control=2;		// servo flag to RC control
			servo_set(FRONT_HP_SERVO_H,1500);	// horizontal servo to middle
			if(feedbackmessageon) serial_puts_p(strHP1RC);
			break;
		case 2:
			hp2_control=2;
			servo_set(REAR_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHP2RC);
			break;
		case 3:
			hp3_control=2;
			servo_set(TOP_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHP3RC);
			break;
		case 0: // all holos under RC
		default:
			hp1_control=hp2_control=hp3_control=2;
			servo_set(FRONT_HP_SERVO_H,1500);
			servo_set(REAR_HP_SERVO_H,1500);
			servo_set(TOP_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHPAllRC);
			break;
	}
}

const char strHP1TE[] PROGMEM="(HP1 Test) \r\n";
const char strHP2TE[] PROGMEM="(HP2 Test) \r\n";
const char strHP3TE[] PROGMEM="(HP3 Test) \r\n";
const char strHPAllTE[] PROGMEM="(All HPs Test) \r\n";
// test mode
void test_command(uint8_t value)
{
	switch(value)
	{

		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1TE);
			hp1_control=3;
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2TE);
			hp2_control=3;
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3TE);
			hp3_control=3;
			break;
		case 0:	// all holos into random movement mode
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllTE);
			hp1_control=hp2_control=hp3_control=3;
			break;
	}
}

#ifdef OLD_MP_CONTROL
// Magic panel on command
// 0= off, 99= on, 1 to 98 : on for 1 to 98 seconds
const char strMagicOn[] PROGMEM="(Magic On) \r\n";
void magic_on_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strMagicOn);
	switch(value)
	{
		case 0:
			magic_control=0;
			on_timer_magic=0;
			break;
		case 99:
			magic_control=1;
			on_timer_magic=0;
			break;
		default:
			magic_control=2;
			on_timer_magic=value*100;
			break;
	}
}

// Magic panel flicker command
// 0= flicker off, 1 to 99 : on for 1 to 99 seconds
const char strMagicFlicker[] PROGMEM="(Magic Flicker) \r\n";
void magic_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strMagicFlicker);
	switch(value)
	{
		case 0:
			magic_control=0;
			on_timer_magic=0;
			break;
		default:
			magic_control=3;
			on_timer_magic=value*100;
			break;
	}
}
#endif

// EXT1 on command
// 0= off, 99= on, 1 to 98 : on for 1 to 98 seconds
const char strExt1On[] PROGMEM="(EXT1 On) \r\n";
void ext1_on_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strExt1On);
	switch(value)
	{
		case 0:
			ext1_control=0;
			on_timer_ext1=0;
			break;
		case 99:
			ext1_control=1;
			on_timer_ext1=0;
			break;
		default:
			ext1_control=2;
			on_timer_ext1=value*100;
			break;
	}
}

const char strHP1Flicker[] PROGMEM="(HP1 Flicker) \r\n";
void holo1_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP1Flicker);
	switch(value)
	{
		case 0:
			hp1_light_control=0;
			on_timer_HP1=0;
			break;
		default:
			hp1_light_control=3;
			on_timer_HP1=value*100;
			break;
	}
}

const char strHP2Flicker[] PROGMEM="(HP2 Flicker) \r\n";
void holo2_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP2Flicker);
	switch(value)
	{
		case 0:
			hp2_light_control=0;
			on_timer_HP2=0;
			break;
		default:
			hp2_light_control=3;
			on_timer_HP2=value*100;
			break;
	}
}

const char strHP3Flicker[] PROGMEM="(HP3 Flicker) \r\n";
void holo3_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP3Flicker);
	switch(value)
	{
		case 0:
			hp3_light_control=0;
			on_timer_HP3=0;
			break;
		default:
			hp3_light_control=3;
			on_timer_HP3=value*100;
			break;
	}
}

const char strAllHPFlicker[] PROGMEM="(All HP Flicker) \r\n";
void all_holo_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strAllHPFlicker);
	switch(value)
	{
		case 0:
			hp1_light_control=hp2_light_control=hp3_light_control=0;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		default:
			hp1_light_control=hp2_light_control=hp3_light_control=3;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=value*100;
			break;
	}
}

const char strHP1Flash[] PROGMEM="(HP1 Flash) \r\n";
void holo1_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP1Flash);
	switch(value)
	{
		case 0:
			hp1_light_control=0;
			on_timer_HP1=0;
			break;
		case 99:
			hp1_light_control=1;
			on_timer_HP1=0;
			break;
		default:
			hp1_light_control=2;
			on_timer_HP1=value*100;
			break;
	}
}

const char strHP2Flash[] PROGMEM="(HP2 Flash) \r\n";
void holo2_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP2Flash);
	switch(value)
	{
		case 0:
			hp2_light_control=0;
			on_timer_HP2=0;
			break;
		case 99:
			hp2_light_control=1;
			on_timer_HP2=0;
			break;
		default:
			hp2_light_control=2;
			on_timer_HP2=value*100;
			break;
	}
}

const char strHP3Flash[] PROGMEM="(HP3 Flash) \r\n";
void holo3_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP3Flash);
	switch(value)
	{
		case 0:
			hp3_light_control=0;
			on_timer_HP3=0;
			break;
		case 99:
			hp3_light_control=1;
			on_timer_HP3=0;
			break;
		default:
			hp3_light_control=2;
			on_timer_HP3=value*100;
			break;
	}
}

const char strAllHPFlash[] PROGMEM="(All HP Flash) \r\n";
void all_holo_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strAllHPFlash);
	switch(value)
	{
		case 0:
			hp1_light_control=hp2_light_control=hp3_light_control=0;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		case 99:
			hp1_light_control=hp2_light_control=hp3_light_control=1;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		default:
			hp1_light_control=hp2_light_control=hp3_light_control=2;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=value*100;
			break;
	}
}


void init_jedi()
{
	lightsuart_puts("0T1\r");	// abort test routine, reset all to normal
	_delay_ms(20);
#ifdef _DIGITALJEDI_
	/**** initialize JEDI display for digital output on HPs and PSI ******/
	// I connected Mike Velchecks rear PSI to the JEDI, which requires output to be turned to digital
	// My holo lights are the older version and also require HPs to be set to digital
	lightsuart_puts("6P91\r");	// change front holo (6) parameter 9 (P9) to digital (1)
	_delay_ms(20);
	lightsuart_puts("5P91\r");   // change rear PSI (5) parameter 9 (P9) to digital (1)
	_delay_ms(20);
#endif
}

/*****************************************
 *  MarcDuino v2 REON holo support
 */

void holo1ON()
{
	if(holo1state!=1)
	{
		holo1state=1;
		HP1_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2ON()
{
	if(holo2state!=1)
	{
		holo2state=1;
		HP2_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3ON()
{
	if(holo3state!=1)
	{
		holo3state=1;
		HP3_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}


void holo1WHITE()
{
	if(holo1state!=1)
	{
		holo1state=1;
		HP1_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2WHITE()
{
	if(holo2state!=1)
	{
		holo2state=1;
		HP2_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3WHITE()
{
	if(holo3state!=1)
	{
		holo3state=1;
		HP3_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}

void holo1OFF()
{
	if(holo1state!=0)
	{
		holo1state=0;
		HP1_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2OFF()
{
	if(holo2state!=0)
	{
		holo2state=0;
		HP2_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3OFF()
{
	if(holo3state!=0)
	{
		holo3state=0;
		HP3_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}


const char strSeqCloseAll[] PROGMEM="(Close all panels) \r\n";
const char strSeqScream[] PROGMEM="(Scream) \r\n";
const char strSeqWave[] PROGMEM="(Wave) \r\n";
const char strSeqFastWave[] PROGMEM="(Fast Wave) \r\n";
const char strSeqOpenCloseWave[] PROGMEM="(Open Close Wave) \r\n";
const char strSeqCantinaMarchingAnts[] PROGMEM="(Cantina Marching Ants) \r\n";
const char strSeqShortCircuit[] PROGMEM="(Short Circuit) \r\n";
const char strSeqCantinaDance[] PROGMEM="(Cantina Dance) \r\n";
const char strSeqLeia[] PROGMEM="(Leia Message) \r\n";
const char strSeqDisco[] PROGMEM="(Disco Dance) \r\n";
const char strSeqQuiet[] PROGMEM="(Set to Quiet) \r\n";
const char strSeqWideAwake[] PROGMEM="(Set to Wide Awake) \r\n";
const char strSeqTopRC[] PROGMEM="(All pie panels to RC) \r\n";
const char strSeqAwake[] PROGMEM="(Set to Awake) \r\n";
const char strSeqExcited[] PROGMEM="(Set to Excited) \r\n";
const char strSeqScreamNoPanels[] PROGMEM="(Scream No Panels) \r\n";
const char strSeqRythmicPanels[] PROGMEM="(Rythmic Panels) \r\n";
const char strSeqMarchingAnts[] PROGMEM="(Marching Ants Panels) \r\n";
const char strSeqPanelWiggle[] PROGMEM="(Panel Wiggle with Scream) \r\n";
const char strSeqByeByeWave[] PROGMEM="(Wave Bye Bye) \r\n";

void sequence_command(uint8_t value)
{
	char string[35];
	switch(value)
	{
		case 0: // CLOSE ALL PANELS
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadspeed(panel_slow_speed);	// slow speed for soft close
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
			seq_startsequence();				// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqCloseAll);		// debug console feedback
			break;

		case 1: // SCREAM
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_all_open, SEQ_SIZE(panel_all_open));
			seq_loadspeed(panel_slow_speed);	// slow open
			seq_startsequence();				// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqScream);		// debug console feedback
			break;

		case 2: // WAVE
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_wave, SEQ_SIZE(panel_wave));
			seq_resetspeed();
			seq_startsequence();				// start sequence
			if(feedbackmessageon) serial_puts_p(strSeqWave);			// debug console feedback
			break;

		case 3: // MOODY FAST WAVE
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_fast_wave, SEQ_SIZE(panel_fast_wave));
			seq_resetspeed();
			seq_startsequence();				// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqFastWave);		// debug console feedback
			break;

		case 4: // OPEN WAVE
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_open_close_wave, SEQ_SIZE(panel_open_close_wave));
			seq_resetspeed();
			seq_startsequence();				// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqOpenCloseWave);	// debug console feedback
			break;

		case 5: // Beep Cantina (R2 beeping the cantina, panels doing marching ants)
			seq_stopsequence(); 				// abort any previous sequence immediately
			//seq_add_completion_callback(resetJEDIcallback); 	// callback to reset displays at end of sequence
			seq_loadsequence(panel_marching_ants, SEQ_SIZE(panel_marching_ants));
			seq_loadspeed(panel_slow_speed);					// slow speed marching ants
			seq_startsequence();								// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqCantinaMarchingAnts);			// debug console feedback
			break;

		case 6: // SHORT CIRCUIT / FAINT
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_all_open_long, SEQ_SIZE(panel_all_open_long));
			seq_loadspeed(panel_super_slow_speed);	// very slow speed open
			seq_startsequence();					// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqShortCircuit);		// debug console feedback
			break;

		case 7: // Cantina (Orchestral Cantina, Rhythmic Panels)
			seq_stopsequence(); 				// abort any previous sequence immediately
			//seq_add_completion_callback(resetJEDIcallback); 	// callback to reset displays at end of sequence
			seq_loadsequence(panel_dance, SEQ_SIZE(panel_dance));
			seq_resetspeed();
			seq_startsequence();				// start panel sequence
			if(feedbackmessageon) serial_puts_p(strSeqCantinaDance);	// debug console feedback
			break;

		case 8: // LEIA
			seq_stopsequence(); 				// Abort previous sequence
			seq_loadspeed(panel_slow_speed);	// Go slow
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));	// Close panels
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqLeia);			// debug console feedback
			break;

		case 9:	// DISCO
			seq_stopsequence(); 				// abort any previous sequence immediately
			//seq_add_completion_callback(resetJEDIcallback); // callback to reset displays at end of sequence
			seq_resetspeed();
			seq_loadsequence(panel_long_disco, SEQ_SIZE(panel_long_disco)); // 6:26 seconds sequence
			seq_startsequence();				// start panel sequence
			//if(feedbackmessageon) serial_puts_p(strSeqDisco);			// debug console feedback
			break;

		case 10: // QUIET   sounds off, holo stop, panel closed
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadspeed(panel_slow_speed);	// go slow
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
			seq_startsequence();				// close panels
			seq_resetspeed();					// sequence speed to fast
			stop_command(0);					// all panels off RC
			if(feedbackmessageon) serial_puts_p(strSeqQuiet);			// debug console feedback
			break;

		case 11: // WIDE AWAKE	random sounds, holos on random, panels closed

			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadspeed(panel_slow_speed);	// go slow
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
			seq_startsequence();				// close panels
			seq_resetspeed();					// sequence speed to fast
			stop_command(0);					// all panels off RC and closed
			if(feedbackmessageon) serial_puts_p(strSeqWideAwake);		// debug console feedback
			break;

		case 12: // TOP PIE PANELS RC
			//rc_command(7);
			//rc_command(8);
			//rc_command(9);
			//rc_command(10);
			if(feedbackmessageon) serial_puts_p(strSeqTopRC);
			break;

		case 13: // AWAKE	random sounds, holos off, panels closed

			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadspeed(panel_slow_speed);	// go slow
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
			seq_startsequence();				// close panels
			seq_resetspeed();					// sequence speed to fast
			stop_command(0);					// all panels off RC and closed
			if(feedbackmessageon) serial_puts_p(strSeqAwake);			// debug console feedback
			break;
		case 14: // EXCITED	random sounds, holos movement, holo lights on, panels closed
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadspeed(panel_slow_speed);	// go slow
			seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
			seq_startsequence();				// close panels
			seq_resetspeed();					// sequence speed to fast
			stop_command(0);					// all panels off RC and closed
			if(feedbackmessageon) serial_puts_p(strSeqExcited);		// debug console feedback
			break;

		case 15: // SCREAM no panels: sound + lights but no panels
			seq_stopsequence(); 				// abort any previous sequence immediately
			if(feedbackmessageon) serial_puts_p(strSeqScreamNoPanels); // debug console feedback
			break;
		case 16: // Panel Wiggle
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_wiggle, SEQ_SIZE(panel_wiggle));
			seq_loadspeed(panel_medium_speed);
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqPanelWiggle);
			break;


		///////////////////////////////////////////
		//	sequences of panels only, no sounds or light effects
		//
		//	:SE51 Scream, with all panels open
		//	:SE52 Wave, one panel at a time
		//	:SE53 Fast (Smirk) back and forth wave
		//	:SE54 Wave 2 (open progressively all panels, then close one by one)
		//	:SE55 Marching ants
		//	:SE56 Faint/Short Circuit
		//	:SE57 Rythmic panel dance
		//  :SE58 Bye Bye Wave
		//////////////////////////////////////////

		case 51: // SCREAM
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_all_open, SEQ_SIZE(panel_all_open));
			seq_loadspeed(panel_slow_speed);	// softer close
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqScream);
			break;
		case 52: // WAVE1
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_wave, SEQ_SIZE(panel_wave));
			seq_resetspeed();
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqWave);
			break;
		case 53: // MOODY FAST WAVE
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_fast_wave, SEQ_SIZE(panel_fast_wave));
			seq_resetspeed();
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqFastWave);
			break;
		case 54: // WAVE2
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_open_close_wave, SEQ_SIZE(panel_open_close_wave));
			seq_resetspeed();
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqOpenCloseWave);
			break;
		case 55: // Marching ant
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_marching_ants, SEQ_SIZE(panel_marching_ants));
			seq_loadspeed(panel_slow_speed);	// softer close
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqMarchingAnts);
			break;
		case 56: // SHORT CIRCUIT / FAINT
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_all_open_long, SEQ_SIZE(panel_all_open_long));
			seq_loadspeed(panel_super_slow_speed);	// very slow close
			seq_startsequence();
			if(feedbackmessageon) serial_puts_p(strSeqShortCircuit);
			break;
		case 57: // Rhythmic Panels
			seq_stopsequence(); 				// abort any previous sequence immediately
			seq_loadsequence(panel_dance, SEQ_SIZE(panel_dance));
			seq_resetspeed();
			if(feedbackmessageon) serial_puts_p(strSeqRythmicPanels);
			seq_startsequence();
			break;
		case 58: // Panel Wave Bye Bye
			seq_stopsequence();
			seq_loadsequence(panel_bye_bye_wave, SEQ_SIZE(panel_bye_bye_wave));
			seq_loadspeed(panel_slow_speed);
			if(feedbackmessageon) serial_puts_p(strSeqByeByeWave);
			seq_startsequence();
			break;
		case 59: // Panel all open - So we can open the slave panels from the master.
			seq_stopsequence();
			seq_loadsequence(panel_all_open_no_close, SEQ_SIZE(panel_all_open_no_close));
			seq_loadspeed(panel_slow_speed);
			seq_startsequence();
			break;
		default:
			sprintf(string, "(Sequence %02d not implemented) \r\n", value);
			seq_resetspeed();
			if(errormessageon) serial_puts(string);
			break;
	}
}

void open_command(uint8_t value)
{
	uint8_t i;

	// Without it, the open command will wait until the previous sequence completes.
	seq_stopsequence(); // abort any previous sequence immediately

	if(value==0) // open all
	{
		// On the slave, open all starts after the HP servos
		for (i=7; i<=SERVO_NUM; i++)
		{
			servo_set(i, _OPN);
		}
		return;
	}
	if(value<=SERVO_NUM) // open specific panel
	{
		servo_set(value, _OPN);
		return;
	}
}

void close_command(uint8_t value)
{
	if(value==0)
	{
		// all panels off RC
		for(int i=7; i<=SERVO_NUM; i++)
		{
			// Silence all panels.
			panel_to_silence[i-1]=1;
		}

		killbuzz_timer=COUNT_PER_SECOND/3; // set a 1/3s timer

		//sequence to close all panels, turn them off slowly
		seq_stopsequence(); // abort any previous sequence immediately
		seq_loadspeed(panel_slow_speed);
		seq_loadsequence(panel_init, SEQ_SIZE(panel_init));
		seq_startsequence();
		return;
	}
	if(value<=SERVO_NUM)
	{
		panel_rc_control[value-1]=0;	// turn off RC control which would re-open the panel
		servo_set(value, _CLS);			// close the servo

		// Give time for the panel to close, then shut it off for buzz control
		killbuzz_timer=COUNT_PER_SECOND/3; // set a 1/3s timer
		panel_to_silence[value-1]=1;	// flag the panel to silence, will be caught in main loop
		return;							// return immediately without blocking
	}
}


/*****************************************
 * MarcDuino v1 versus v2 suart and suart2 change
 * In the slave, suarts were swapped compared to Master (in error)
 * This is corrected in v2:
 *
 * Lights output is
 * 	suart which is PC0 on v1
 * 	suart2 which is PC1 on v2
 * Next slave outpur is
 * 	suart2 which is PC4 on v1
 * 	suart which is PC0 on v2
 * The suart.c library handles the PC4/PC1 pin change
 * These functions handle the suart/suart2 inversion
 *******************************************/

void slavesuart_init(uint16_t baudrate)
{
#ifdef _MARCDUINOV2_
	suart_init(baudrate); 		// suart (PC0) for MarcDuino v2
#else
	suart2_init(baudrate);		// suart2 (PC4) for MarcDuino v1
#endif
}
void slavesuart_putc(unsigned char c)
{
#ifdef _MARCDUINOV2_
	suart_putc(c); 		// suart (PC0) for MarcDuino v2
#else
	suart2_putc(c);		// suart2 (PC4) for MarcDuino v1
#endif
}

void slavesuart_puts(char* s)
{
#ifdef _MARCDUINOV2_
	suart_puts(s); 		// suart (PC0) for MarcDuino v2
#else
	suart2_puts(s);		// suart2 (PC4) for MarcDuino v1
#endif
}

void slavesuart_puts_p(const char *progmem_s )
{
#ifdef _MARCDUINOV2_
	suart_puts_p(progmem_s); 		// suart (PC0) for MarcDuino v2
#else
	suart2_puts_p(progmem_s);		// suart2 (PC4) for MarcDuino v1
#endif
}

void lightsuart_init(uint16_t baudrate)
{
#ifdef _MARCDUINOV2_
	suart2_init(baudrate); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_init(baudrate);		// suart (PC0) for MarcDuino v1
#endif
}

void lightsuart_putc(unsigned char c)
{
#ifdef _MARCDUINOV2_
	suart2_putc(c); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_putc(c);		// suart (PC0) for MarcDuino v1
#endif
}

void lightsuart_puts(char* s)
{
#ifdef _MARCDUINOV2_
	suart2_puts(s); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_puts(s);		// suart (PC0) for MarcDuino v1
#endif
}
void lightsuart_puts_p(const char *progmem_s )
{
#ifdef _MARCDUINOV2_
	suart2_puts_p(progmem_s); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_puts_p(progmem_s);		// suart (PC0) for MarcDuino v1
#endif
}
