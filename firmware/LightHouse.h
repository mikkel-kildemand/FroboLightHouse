/****************************************************************************
# FrobolLightHouse FroboMind Controller interface
# Control unit FroboLightHouse.
# Copyright (c) 2014-2016, Mikkel K. Larsen
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: FroboLightHouse.h
# Project: FroboLightHouse_SW
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Mikkel K. Larsen
# Created:  2014-10-23 Mikkel K. Larsen
# Modified: 

****************************************************************************/
/* includes */
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

/***************************************************************************/

/* FroboLightHouse defines  */

#define false					0
#define true					1

#define FLH_Port			PORTA

#define RED_STATE_OFF			0
#define RED_STATE_ON			1
#define RED_DELAY				4 /* times the cycle */

#define YELLOW_STATE_OFF		0
#define YELLOW_STATE_ON			1
#define YELLOW_DELAY			4 /* times the cycle */

#define GREEN_STATE_OFF			0
#define GREEN_STATE_ON			1
#define GREEN_DELAY				4 /* times the cycle */

#define RED_flash				0
#define YELLOW_flash			1
#define GREEN_flash				2
#define RYG_MassageSkiped		3
#define RED_sollid				4
#define YELLOW_sollid			5
#define GREEN_sollid			6
#define OFF_ALL					7

/* FroboLightHouse defines  */
#define BV(bit)					(1<<bit) /* return bit value for a bit */

/* ATmega port defines (output) */
#define PB_OUT( ddr, bit)		ddr |= BV(bit) /* set port bit as output */
#define PB_HIGH( port, bit)		port |= BV(bit) /* set port bit high */
#define PB_LOW( port, bit)		port &= ~BV(bit) /* set port bit low */
#define PB_FLIP( port, bit)		port ^= BV(bit) /* flip port bit */

/* ATmega port defines (input) */
#define PB_IN( ddr, bit)		ddr &= ~BV(bit) /* set port bit as input */
#define PB_PULL_UP( port, bit)		PB_HIGH(port, bit) /* enable pull-up resistor */
#define PB_IS_HIGH( inport, bit)	inport & BV(bit) /* true if port bit is high */
#define PB_IS_LOW( inport, bit)		!(inport & BV(bit)) /* true if port bit is low */

/* FroboLightHouse RED defines */
#define INT_RED_INIT			PB_OUT (DDRA,DDA1) /* set LED bit as output */
#define INT_RED_OFF				PB_LOW (PORTA,PA1) /* turn LED on */
#define INT_RED_ON				PB_HIGH (PORTA,PA1) /* turn LED off */

/* FroboLightHouse Yellow defines */
#define INT_YELLOW_INIT			PB_OUT (DDRA,DDA2) /* set LED bit as output */
#define INT_YELLOW_OFF				PB_LOW (PORTA,PA2) /* turn LED on */
#define INT_YELLOW_ON				PB_HIGH (PORTA,PA2) /* turn LED off */

/* FroboLightHouse Green defines */
#define INT_GREEN_INIT			PB_OUT (DDRA,DDA3) /* set LED bit as output */
#define INT_GREEN_OFF				PB_LOW (PORTA,PA3) /* turn LED on */
#define INT_GREEN_ON				PB_HIGH (PORTA,PA3) /* turn LED off */

/* FroboLightHouse variables */


char red_state;
char red_signal;
char red_command;
char red_count;

char yellow_state;
char yellow_signal;
char yellow_command;
char yellow_count;

char green_state;
char green_signal;
char green_command;
char green_count;

char flash_running;
char solid_running;

char next_light_state;
char light_state;

/***************************************************************************/

/* Funktion declarations */

void LightHouse_Update(int red_command, int yellow_command, int green_command);

void FroboLightHouse_RED_Update(void);
void FroboLightHouse_YELLOW_Update(void);
void FroboLightHouse_GREEN_Update(void);
