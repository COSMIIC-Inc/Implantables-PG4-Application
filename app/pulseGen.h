//    pulseGen: .h     HEADER FILE.

#ifndef _H
#define _H


// -------- DEFINITIONS ----------

#define MAX_PULSE_CHAN			4

#define MIN_PULSE_PERIOD		20
#define MAX_PULSE_PERIOD		1000




// --------   DATA   ------------
static struct
{
	struct PulseDef
	{
		UINT16 duration;		// 0-255 usec = 0-2040 clock ticks @ 8MHz
		UINT8 amplitude;		// 0.0 - 20.0 mA
		UINT16 ipInterval;		// 50-250 usec; 0=off
		BW16  dacBits;			// formatted dac bits

	} pulseDef[ MAX_PULSE_CHAN ];

	UINT16 period;				// 20 - 1000 msec

	UINT16 timer;				// 1 msec counter
	UINT8 flag;				//= ff at end of frame

} frame = 
	{
		{{ 0, 0, 0, 0 },
		{  0, 0, 0, 0 },
		{  0, 0, 0, 0 },
		{  0, 0, 0, 0 }},

		0, 0, 0

	};

// -------- PROTOTYPES ----------
void initPulseGenerator( void );

UINT8 configPulseChannel( UINT8 chan, UINT8 ampl, UINT8 width, UINT8 ipi );
//UINT8 configPulsePeriod( UINT16 period );
UINT8 isStimCycleDone( UINT8 mask );
void configVOS( UINT8 stim );
volatile void StimPulse(UINT8);
extern unsigned char  setupVOSComplete; 
#endif
 
