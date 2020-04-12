/*
 * panel_sequences.h
 *
 *  Created on: Aug 5, 2012
 *  Author: Marc Verdiell
 *  Defines the sequences matrices for the dome panel moves
 *  Each of them is stored in program memory
 *  See sequencer.h for details on how to declare and use them
 *  See examples on how they are used in main.c
 *
 *  Revised on March 14th, 2020
 *  	Author: Neil Hutchison
 *
 *  Added Per row speed configuration
 *  Added Ability to select a range of servos to be acted on
 *  Added Partial Panel opening
 *  Added 13 panel support
 *
 */
/************ example of how to use the sequencer
* define a sequence matrix like this
* make sure to follow exactly this declaration to put the array in program memory
* (it needs the const and the PROGMEM):
	sequence_t const servo_dance PROGMEM=
	{
			// time	servo1	servo2	servo3	servo4
			{50, 	1000, 	1000, 	1000, 	1000},
			{50, 	2000, 	1000, 	1000, 	1000},
			{50, 	1000, 	2000, 	1000, 	1000},
			{50, 	1000, 	1000, 	2000, 	1000},
			{50, 	1000, 	1000, 	1000, 	2000},
			{50, 	1000, 	1000, 	1000, 	1000},
			{50, 	2000, 	2000, 	1000, 	1000},
			{50, 	1000, 	1000, 	1000, 	1000},
			{50, 	1000, 	1000, 	2000, 	2000},
			{50, 	1500, 	1500, 	1500, 	1500},
			{0, 	_NP, 	_NP, 	_NP, 	_NP},
	};
	// time units are 1/100 seconds. Max is 65535.
	// 10 is 1/10 of a second
	// 100 is 1 second
	// 1000 is 10 second
	// 6000 is a minute
	// 36000 is 6 minutes
	// optionally define a speed matrix for each servo, (0 = max speed, larger numbers = lower speed)
	int16_t servo_dance_speed[]={50,20,0,20};
	in main() or elsewhere call:
	seq_loadsequence(servo_dance, SEQ_SIZE(servo_dance));		// SEQ_SIZE macro defined in sequencer.h to calculate the sequence length
	seq_loadspeed(servo_dance_speed);							// optional, to set the speed of servos
	seq_startsequence();
	To stop it:
	seq_stopsequence();
**************************************/
#ifndef PANEL_SEQUENCES_H_
#define PANEL_SEQUENCES_H_
#include <avr/pgmspace.h>	// for the sequencer data arrays defined with PROGMEM
#include "sequencer.h"		// servo sequencer
// Dome panel sequences
// pulse values for opening and closing panel servos
#define _OPN 1000
#define _CLS 2000
sequence_t const panel_all_open PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // 6 & 7 == 12 & 13!
	{300, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_OPN,	_NP,	6,		7},
	{150, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_all_open_long PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{1000, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_OPN,	_NP,	6,		7},
	{150, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_all_open_no_close PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // 6 & 7 == 12 & 13!
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_OPN,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_wave PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{265, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // Sync with master servo sequence.
	{30, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_CLS,	_NP,	6,		7},
	{115, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{30, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_OPN,	_NP,	6,		7},
	{30, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_fast_wave PROGMEM=
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{135, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // Sync with master servo sequence.
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_CLS,	_NP,	6,		7},
	{60, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_OPN,	_NP,	6,		7}, // All opened forwards.
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_OPN,	_NP,	6,		7}, // Start reverse
	{75, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7}, //45
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_OPN,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_open_close_wave PROGMEM=
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{180, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{100, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{220, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_OPN,	_NP,	6,		7},
	{80, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7},
	{40, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_marching_ants PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //1
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //2
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //3
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //4
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //5
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //6
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //7
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //8
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //9
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //10
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //11
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //12
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //13
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //14
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7}, //15
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{100, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7}, //Close all
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
// TODO ... figure this one out!
sequence_t const panel_dance PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_init PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{100, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
// 6min26sec disco sequence to trigger callback at the right time
sequence_t const panel_long_disco PROGMEM=
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{135, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{60, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_OPN,	_NP,	6,		7},
	{75, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{15, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_CLS,	_NP,	6,		7},
	{75, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},
	{36000, _NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // 6 minutes
	{2100, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_CLS,	_CLS,	_NP,	6,		7},  // 21 seconds
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
// Maxstang's sequences
sequence_t const panel_bye_bye_wave PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
sequence_t const panel_wiggle PROGMEM =
{
	//  -------------------MASTER-------------------------  ----SLAVE------ -----------CONFIG-----------
	// time	HPFV	HPFH	HPRV	HPRH	HPTV	HPTH	servo12	servo13	speed	first 	last
	{20, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_OPN,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_OPN,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_OPN,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{14, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_OPN,	_OPN,	_NP,	6,		7},
	{50, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP,	_CLS,	_CLS,	_NP,	6,		7},
	{0, 	_NP, 	_NP, 	_NP, 	_NP,	_NP, 	_NP, 	_NP,	_NP,	_NP,	6,		7}
};
int16_t panel_fast_speed[]={0,0,0,0,0,0,0,0,0,0,0,0};
int16_t panel_medium_speed[]={25,25,25,25,25,25,25,25,25,25,25,25};
int16_t panel_slow_speed[]={15,15,15,15,15,15,15,15,15,15,15,15};
int16_t panel_super_slow_speed[]={9,9,9,9,9,9,9,9,9,9,9,9};
#endif /* PANEL_SEQUENCES_H_ */
