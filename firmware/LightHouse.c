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
# File: FroboLightHouse.c
# Project: FroboLightHouse_SW
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Mikkel K. Larsen
# Created:  2014-10-23 Mikkel K. Larsen
# Modified: 
******************************************************************************/
/* includes */
#include "LightHouse.h"

/***************************************************************************/

void LightHouse_Update(int red_command, int yellow_command, int green_command)
{
	if (0 != red_command){
		if (red_command > 9 ){
			next_light_state = RED_sollid ;
		}
		else{	
		next_light_state = RED_flash ;
		}

		if(flash_running == 0){
			red_signal = red_command;
			}
		else if (red_count < red_command && red_command < 10){
			red_signal = red_command;
			}
		}

	else if (0 != yellow_command){
		if (yellow_command > 9 ){
			next_light_state = YELLOW_sollid ;
		}
		else{	
		next_light_state = YELLOW_flash;
		}

		if(flash_running == 0){
			yellow_signal = yellow_command;
			}
		else if (yellow_count < yellow_command && yellow_command < 10){
			yellow_signal = yellow_command;
			}
		}

	else if (0 != green_command){
				if (green_command > 9 ){
			next_light_state = GREEN_sollid ;
		}
		else{	
		next_light_state = GREEN_flash;
		}

		if(flash_running == 0){
			green_signal = green_command;
			}
		else if (green_count < green_command && green_command < 10){
			green_signal = green_command;
			}
		}
	else {
			red_signal = 0;
			yellow_signal = 0;
			green_signal = 0;

	}
	

	if (flash_running == 0){

		if (solid_running == 1){
			if (next_light_state != light_state){
				next_light_state = OFF_ALL;
			}
		}

		light_state = next_light_state ;

	}



	switch (light_state)
	{
		case OFF_ALL:
			INT_RED_OFF;
			INT_YELLOW_OFF;
			INT_GREEN_OFF;
			solid_running = 0;
			break;

		case RED_sollid:
			INT_RED_ON;
			INT_YELLOW_OFF;
			INT_GREEN_OFF;
			yellow_signal = 0;
			green_signal = 0;
			solid_running = 1;
			break;

		case RED_flash:
			yellow_signal = 0;
			green_signal = 0;
			flash_running = 1;
			switch (red_state) {
				case RED_STATE_ON:
					red_state = RED_STATE_OFF;
					INT_RED_OFF;
					break;

				case RED_STATE_OFF:
					red_count++;
					if (red_count <= red_signal) {
						INT_RED_ON;
						red_state = RED_STATE_ON;
					}
					else if (red_count > red_signal + RED_DELAY) {
						red_count = 0;
						flash_running = 0;
					}
					break;
			}
			break;

		case YELLOW_sollid:
			red_signal = 0;
			green_signal = 0;
			INT_RED_OFF;
			INT_YELLOW_ON;
			INT_GREEN_OFF;
			solid_running = 1;
			break;

		case YELLOW_flash:
			red_signal = 0;
			green_signal = 0;
			flash_running = 1;
			switch (yellow_state) {
				case YELLOW_STATE_ON:
					yellow_state = YELLOW_STATE_OFF;
					INT_YELLOW_OFF;
					break;

				case YELLOW_STATE_OFF:
					yellow_count++;
					if (yellow_count <= yellow_signal) {
						INT_YELLOW_ON;
						yellow_state = YELLOW_STATE_ON;
					}
					else if (yellow_count > yellow_signal + YELLOW_DELAY) {
						yellow_count = 0;
						flash_running = 0;
					}
					break;
			}
			break;


		case GREEN_sollid:
			INT_RED_OFF;
			INT_YELLOW_OFF;
			INT_GREEN_ON;
			red_signal = 0;
			yellow_signal = 0;
			solid_running = 1;
			break;

		case GREEN_flash:
			red_signal = 0;
			yellow_signal = 0;
			flash_running = 1;
			switch (green_state) {
				case GREEN_STATE_ON:
					green_state = GREEN_STATE_OFF;
					INT_GREEN_OFF;
					break;

				case GREEN_STATE_OFF:
					green_count++;
					if (green_count <= green_signal) {
						INT_GREEN_ON;
						green_state = GREEN_STATE_ON;
					}
					else if (green_count > green_signal + GREEN_DELAY) {
						green_count = 0;
						flash_running = 0;
					}
					break;
			}
			break;

		default:
			light_state = 1;

	}



}
void FroboLightHouse_RED_Update(void)
{

}
/***************************************************************************/
void FroboLightHouse_YELLOW_Update(void)
{}
/***************************************************************************/
void FroboLightHouse_GREEN_Update(void)
{

}
