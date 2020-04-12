/*
 * main.h
 * 	MarcDuino Slave HP control
 * 	Author: Marc Verdiell
 * 	v1.8
 *
 *  v2.0
 *  Author: Neil Hutchison
 *
 * 	See description in implementation file
 *
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h> 		// uint8_t and companions

/********* BUILD SETTINGS FOR ALL VARIANTS, COMMENT IN OR OUT TO CHANGE OPTIONS ********/
#define _RELEASEVERSION_		// un-comment for public release
#define _MARCDUINOV2_			// for the V2 boards
/***************************************************************************************/

// turn to 1 for debug only
#define _ERROR_MSG_ 1
#define _FEEDBACK_MSG_ 1

// default to private version settings
#ifndef _RELEASEVERSION_
#define _PRIVATEVERSION_		// settings for my own droid
#endif


// settings for the public release version are here
#ifdef _RELEASEVERSION_
	#define _REVERSEMAGICPANEL_				// for compatibility with Michael Wheeler's Magic Panel and my own Holo Lights
	#define _REVERSEHOLOS_					// my own HoloLights 2.0 are easier to connect using the reverse logic also
	//#define _9600BAUDSJEDI_				// for compatibility with future version of JEDI display
	//#define _RESTRICT_HP_SERVO_RANGE_		// you could turn that one on if your HP servos move to much and go to end of travel
	//#define _EXTEND_HP_SERVO_RANGE_		// For Jason and the users of the BobC holo kit
#endif

// settings for my own droid version are here
#ifdef _PRIVATEVERSION_
	#define _DIGITALJEDI_					// my JEDI setting is non-standard
//	#define	_REVERSEMAGICPANEL_				// my own magic panel works opposite from Michael Weir's and my own Holo Lights
//	#define _REVERSEHOLOS_					// ### while debugging with v2.0 holo lights
	#define _RESTRICT_HP_SERVO_RANGE_		// smaller range of servo movement to fit my HP setup
//	#define _9600BAUDSJEDI_					// for future compatibility with new version of JEDI display

#endif

#define CMD_MAX_LENGTH   64		//Max length of the command string

// all commands must start with one of these characters
#define PANEL_START_CHAR 	':'
#define HP_START_CHAR		'*'
#define DISPLAY_START_CHAR 	'@'
#define SOUND_START_CHAR 	'$'
#define ALT1_START_CHAR		'!'	// for custom extensions, messages get forwarded on SUART (PC0) pin of slave HPPanel Marcduino
#define ALT2_START_CHAR		'%'	// for custom extensions, messages get forwarded on SUART2(PC4) pin of slave HPPanel Marcduino
#define I2C_START_CHAR		'&' // for outputting an i2c commands
#define SETUP_START_CHAR    '#' // For MarcDuino Setup commands.

// all command must end with one of these characters
#define CMD_END_CHAR 	'\r'

// I2C control characters (not used, hard coded for now)
#define CMD_SEP_CHAR	','		// separator for I2C arguments
#define CMD_HEX_CHAR	'x' 	// for hex numbers
#define CMD_CHAR_CHAR	'\'' 	// char delimiter
#define CMD_STR_CHAR	'"' 	// string delimiter

// Setup command vocabulary
#define SETUP_SERVO_DIR "SD"
#define SETUP_SERVO_REVERSE "SR"	// Reverse Individual Servo.  #SRxxy where xx is the servo y is 0 for forward, 1 for reverse.
#define SETUP_LAST_SERVO "SL"

// define HP servo max random movement, and which servo positions are they connected to
#ifdef _RESTRICT_HP_SERVO_RANGE_
	#define SERVO_HP_MIN 		1200		// constrains the holo servo max swing
	#define SERVO_HP_MAX 		1800	// in both axis (usual RC min is 1000 and max is 2000)
#elif  defined(_EXTEND_HP_SERVO_RANGE_)
	#define SERVO_HP_MIN 		600		// constrains the holo servo extreme positions
	#define SERVO_HP_MAX 		2400	// in both axis (usual RC min is 500 and max is 2000)
#else
	#define SERVO_HP_MIN 		1000	// constrains the holo servo extreme positions
	#define SERVO_HP_MAX 		2000	// in both axis (version 1.5 had 500 as min, in error)
#endif

#define FRONT_HP_SERVO_H 	1
#define FRONT_HP_SERVO_V 	2
#define REAR_HP_SERVO_H 	3
#define REAR_HP_SERVO_V 	4
#define TOP_HP_SERVO_H 		5
#define TOP_HP_SERVO_V 		6

#ifdef OLD_MP_CONTROL
// define magic panel connection (was servo 10)
#define MAGIC_PORT	PORTB
#define MAGIC_PIN	5
#endif

// define holo Lights connection (was servo 10)
#define HP1_LIGHT_PORT	PORTB	// front
#define HP1_LIGHT_PIN	2
#define HP2_LIGHT_PORT	PORTB	// rear
#define HP2_LIGHT_PIN	3
#define HP3_LIGHT_PORT	PORTB	// top
#define HP3_LIGHT_PIN	4

#ifdef _REVERSEHOLOS_	//inverted logic, low is on
#define HP1_ON digitalWrite(HP1_LIGHT_PORT, HP1_LIGHT_PIN, LOW)
#define HP1_OFF digitalWrite(HP1_LIGHT_PORT, HP1_LIGHT_PIN, HIGH)
#define HP2_ON digitalWrite(HP2_LIGHT_PORT, HP2_LIGHT_PIN, LOW)
#define HP2_OFF digitalWrite(HP2_LIGHT_PORT, HP2_LIGHT_PIN, HIGH)
#define HP3_ON digitalWrite(HP3_LIGHT_PORT, HP3_LIGHT_PIN, LOW)
#define HP3_OFF digitalWrite(HP3_LIGHT_PORT, HP3_LIGHT_PIN, HIGH)
#else					// positive logic, high is on
#define HP1_ON digitalWrite(HP1_LIGHT_PORT, HP1_LIGHT_PIN, HIGH)
#define HP1_OFF digitalWrite(HP1_LIGHT_PORT, HP1_LIGHT_PIN, LOW)
#define HP2_ON digitalWrite(HP2_LIGHT_PORT, HP2_LIGHT_PIN, HIGH)
#define HP2_OFF digitalWrite(HP2_LIGHT_PORT, HP2_LIGHT_PIN, LOW)
#define HP3_ON digitalWrite(HP3_LIGHT_PORT, HP3_LIGHT_PIN, HIGH)
#define HP3_OFF digitalWrite(HP3_LIGHT_PORT, HP3_LIGHT_PIN, LOW)
#endif

#ifdef OLD_MP_CONTROL
	#ifdef _REVERSEMAGICPANEL_	// v1.3 Michael Wheeler Logic is inverted, LOW is ON
		#define MAGIC_ON digitalWrite(MAGIC_PORT, MAGIC_PIN, LOW)
		#define MAGIC_OFF digitalWrite(MAGIC_PORT, MAGIC_PIN, HIGH)
	#else						// positive logic, high is on
		#define MAGIC_ON digitalWrite(MAGIC_PORT, MAGIC_PIN, HIGH)
		#define MAGIC_OFF digitalWrite(MAGIC_PORT, MAGIC_PIN, LOW)
	#endif
#endif


// Panel command vocabulary
#define CMD_SEQUENCE 	    "SE"		// launches a sequence
#define CMD_OPEN 		    "OP"		// opens a panel (00=all panels)
#define CMD_CLOSE 		    "CL"		// closes a panel, removes from RC, kill servos (00=all panels)
#define CMD_HOLD 			"HD"
#define CMD_STOP 			"ST"
#define CMD_RANDOM 			"RD"
#define CMD_ON 				"ON"
#define CMD_OFF				"OF"
#define CMD_RC				"RC"
#define CMD_TEST			"TE"
#ifdef OLD_MP_CONTROL
	#define CMD_MAGIC_ON		"MO" // 0-off, 99-on, 1 to 98= on during the time in seconds
	#define CMD_MAGIC_FLICKER	"MF" // flicker for the time in seconds
#endif
#define CMD_HOLO1_FLICKER 	"F1" // flicker for the time in seconds
#define CMD_HOLO2_FLICKER 	"F2" // flicker for the time in seconds
#define CMD_HOLO3_FLICKER 	"F3" // flicker for the time in seconds
#define CMD_ALL_HOLO_FLICKER "F0" // flicker for the time in seconds
#define CMD_HOLO1_FLASH 	"H1" // flash for the time in seconds
#define CMD_HOLO2_FLASH 	"H2" // flash for the time in seconds
#define CMD_HOLO3_FLASH 	"H3" // flash for the time in seconds
#define CMD_ALL_HOLO_FLASH  "H0" // flash for the time in seconds

uint8_t build_command(char ch, char* output_str);
void dispatch_command(char* command_str);
void parse_panel_command(char* command_string, uint8_t length);
void parse_hp_command(char* command,uint8_t length);
void parse_display_command(char* command,uint8_t length);
void parse_sound_command(char* command,uint8_t length);
void parse_alt1_command(char* command, uint8_t length);
void parse_alt2_command(char* command, uint8_t length);
void parse_setup_command(char* command, uint8_t length);
void process_command(char* thecommand, char* theargument);
void sequence_command(uint8_t value);
void open_command(uint8_t value);
void close_command(uint8_t value);
void stop_command(uint8_t value);
void hold_command(uint8_t value);
void random_command(uint8_t value);
void on_command(uint8_t value);
void off_command(uint8_t value);
void rc_command(uint8_t value);
void test_command(uint8_t value);
#ifdef OLD_MP_CONTROL
	void magic_on_command(uint8_t value);
	void magic_flicker_command(uint8_t value);
#endif
void holo1_flicker_command(uint8_t value);
void holo2_flicker_command(uint8_t value);
void holo3_flicker_command(uint8_t value);
void all_holo_flicker_command(uint8_t value);
void holo1_flash_command(uint8_t value);
void holo2_flash_command(uint8_t value);
void holo3_flash_command(uint8_t value);
void all_holo_flash_command(uint8_t value);
void init_jedi();

// i2c parsing (v1.8)
void parse_i2c_command(char* command,uint8_t length);
void sendI2C(uint8_t address, uint8_t* payload, uint8_t payload_length);
uint8_t append_token(uint8_t* payload, uint8_t* index, char* token);

/********MARCDUINO V2*********************/
// to handle the REON projectors as well as the native ones,
// generic holo functions are defined

// I2C addresses of the REONs
#define REON1 25 // front
#define REON2 27 // rear
#define REON3 26 // top
// I2C commands
#define REON_ON 3  // 3 is on with random changing colors
#define REON_OFF 1 // off command
#define REON_WHITE 2 // 2 is solid white

void holo1ON();
void holo2ON();
void holo3ON();
void holo1WHITE();
void holo2WHITE();
void holo3WHITE();
void holo1OFF();
void holo2OFF();
void holo3OFF();

// to handle the suart switch between MarcDuino v1 and MarcDuino v2
// redefine the software serial commands independent of suart or suart2
void slavesuart_init(uint16_t baudrate);
void slavesuart_putc(unsigned char c);
void slavesuart_puts(char* s);
void slavesuart_puts_p(const char *progmem_s );

void lightsuart_init(uint16_t baudrate);
void lightsuart_putc(unsigned char c);
void lightsuart_puts(char* s);
void lightsuart_puts_p(const char *progmem_s );

#endif /* MAIN_H_ */
