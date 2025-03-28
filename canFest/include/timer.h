/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/**
 * @file   timer.h
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief Header file for timer.c
*/

#ifndef __timer_h__
#define __timer_h__

#include <timerscfg.h>
#include <applicfg.h>

#define TIMER_HANDLE INTEGER16

#include "data.h"

/* --------- types and constants definitions --------- */
#define TIMER_FREE 0        // timer is available for use
#define TIMER_ARMED 1       // row is active
#define TIMER_TRIG 2        // timer has been triggered
#define TIMER_TRIG_PERIOD 3

#define TIMER_NONE -1

typedef void (*TimerCallback_t)(CO_Data* d, UNS32 id);

/**
 * @brief timer struct 
 */
struct struct_s_timer_entry {
	UNS8 state;
	CO_Data* d;
	TimerCallback_t callback; /*!< The callback func. */
	UNS32 id; /*!< The callback func. */
	TIMEVAL val;  /*!< this is the current timer value */
	TIMEVAL interval; /*!< Periodicity */
};

typedef struct struct_s_timer_entry s_timer_entry;

/****************Variables****************************/
extern unsigned char osc_value;

/* ---------  prototypes --------- */
TIMER_HANDLE SetAlarm(CO_Data* d, UNS32 id, TimerCallback_t callback, TIMEVAL value, TIMEVAL period);
TIMER_HANDLE DelAlarm(TIMER_HANDLE handle);
void TimeDispatch(void);

//for timer_AVR.c
void setTimer(TIMEVAL value);
TIMEVAL getElapsedTime(void);

#endif /* #define __timer_h__ */
