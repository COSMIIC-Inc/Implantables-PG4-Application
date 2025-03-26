//    stimTask: .c
/*	HOW IT WORKS:  
	Stim runs from the profile in OD.  
	SDO can save/recall profiles to/from EEprom.
	Here we maintain profile memories per SDO commands and leave status for SDO.
	Here we interpolate the profile based on OD x value, per OD selection commands.
	here we configure the pulse generator.
*/

#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "pulseGen.h"
#include "ObjDict.h"
#include "states.h"
#include "stimTask.h"
#include "scheduler.h"
#include "eedata.h"


// -------- DEFINITIONS ----------
//Patterns are organized in EEPROM:
//  * Pattern01 at 0x400-0x43F
//  * Pattern02 at 0x440-0x47F
//     ...
//  * Pattern48 at 0xFC0-0xFFF
// Note: 0x000-0x3FF is reserved for Restore List
//Patterns are organized as follows:
//  * channel [1 byte (1-4)]
//  * commandID [1 byte (0-7)]
//  * number of points in pattern [1 byte (0-20)] 
//  * x [20 bytes]
//  * pw [20 bytes]
//  * amp [20 bytes]
//  * reserved [1 byte]

#define PATTERNS_EEPROM_ADDRESS 0x0400
#define BYTES_PER_PATTERN       0x0040
#define MAX_PATTERNS            48

// --------   DATA   ------------

/*************Data Ranges for pulse generation: ************************

		UINT8 duration;			// 0-250 usec
		UINT8 amplitude;		// 0.0 - 20.0 (value / 10) mA
		UINT8 ipInterval;		// 50-250 usec; 0=off
		BW16  dacBits;			// formatted dac bits

	        UINT16 period;			// 20 - 1000 msec

	        UINT16 timer;			// 1 msec counter
	        UINT8 flag;			//= ff at end of frame 
**************************************************************************/

//vvJML I don't think this is being used anymore:
/* flash profile data  
typedef struct 
{
	UINT8 x[ NUM_STIM_PTS ];
	UINT8 pw[ NUM_STIM_PTS ];
	UINT8 ampl[ NUM_STIM_PTS ];

} PROFILE_CHANNEL;


typedef struct 
{
	PROFILE_CHANNEL chan[4];

} PROFILE;

#pragma location = 0x898 // need to move it up out of the way of SaveRestore()
__no_init __eeprom PROFILE eeProfile[ NUM_PROFILES ];
*/
//^^

/**
 * @brief contains the patterns for the active function group for each channel
 *        Note that there is only one pattern per channel, therefore if two function
 *        groups are active simultaneously, they cannot share channels.
*/
struct //JML: this was a struct of pointers instead of arrays, now it could use the PROFILE struct
{
        UNS8  num;
        UNS8  x[ PATTERN_ARRAYSIZE ];
	UNS8  pw[ PATTERN_ARRAYSIZE ];
	UNS8  ampl[ PATTERN_ARRAYSIZE ];
	
} static odPattern[4];

       
extern volatile UINT8 syncCount[4]; //defined in scheduler.c (must be reset upon entering/exiting stim mode)

// -------- PROTOTYPES ----------
static void interpChannelWaveform( UINT8 chan, UINT8 x, UINT8 *pw, UINT8 *ampl );


 
//============================
//    GLOBAL CODE
//============================
void initStimTask( void )
{
  initPulseGenerator();
  ClearAllActivePatterns();  
}

/**
 * @brief Clears active patterns stored in struct "odPattern"
 */
void ClearAllActivePatterns( void )
{
   UNS8 i = 0;
  // initialize the active patterns. 
  for ( i = 0; i < 4; i++)
  {
     ClearActivePattern(i);
  }
}

/**
 * @brief Clears an active pattern (one channel) stored in struct "odPattern"
 * @param ch (0-3)
 */
void ClearActivePattern( UNS8 ch)
{
   odPattern[ch].num = 0;
   memset(odPattern[ch].x, 0, PATTERN_ARRAYSIZE ); 
   memset(odPattern[ch].pw, 0, PATTERN_ARRAYSIZE ); 
   memset(odPattern[ch].ampl, 0, PATTERN_ARRAYSIZE ); 
   
   ActiveFunctionGroups[ch] = 0;
}
  
/**
 * @brief Copies the pattern data for the target function group from EEPROM into struct "odPattern"
 *        for use in interpChannelWaveform(). 
 * @param targetFunctionGroup, specified by param1 in NMT
*  @param active: 1 activates targetFunctionGroup, 0 deactivates targetFunctionGroup
*/
void UpdateActivePatterns ( UNS8 targetFunctionGroup, UNS8 active )
{
  UNS8 i = 0;
  UNS8 channelNumber = 0; // valid range 1 to 4
  UNS8 functionGroup = 0; // valid range 0-255;
  UNS16 addr;
  
  UINT32 size = 0;
  UINT8  ptype;
  UINT32 abortCode = 0;
  
   
   
  // If the function group in the pattern matches the active function group (param1), load the pattern into odPattern
  for (i = 0; (i < Num_ChanPatterns) && (i < MAX_PATTERNS); i++)
  {
    //PORTE |= BIT0; //DEBUG ONLY set PE1 high
    // need to reset the size before the next "read" 
    size = 0;
    ptype = 0;
    abortCode = readLocalDict( &ObjDict_Data, 0x3300, i+1, &functionGroup, &size, &ptype, 0);
   
    if ( abortCode != OD_SUCCESSFUL )
    {
        //PORTE &=~ BIT0; //DEBUG ONLY set PE1 low
       return;
    }
         
    
    if (functionGroup == targetFunctionGroup)
    {
     
        //Read ChannelNumber from EEPROM
        addr = PATTERNS_EEPROM_ADDRESS + BYTES_PER_PATTERN*i;
        EEPROM_read(addr++, &channelNumber, 1);
        
        if(active)
        {
          EEPROM_read(addr++, &X_ChannelMap[channelNumber - 1], 1); 
          EEPROM_read(addr++, &odPattern[channelNumber - 1].num, 1);
          EEPROM_read(addr, odPattern[channelNumber - 1].x, PATTERN_ARRAYSIZE ); 
            addr+=PATTERN_ARRAYSIZE;
          EEPROM_read(addr, odPattern[channelNumber - 1].pw, PATTERN_ARRAYSIZE );
            addr+=PATTERN_ARRAYSIZE;
          EEPROM_read(addr, odPattern[channelNumber - 1].ampl, PATTERN_ARRAYSIZE ); 
          
          ActiveFunctionGroups[channelNumber - 1] = functionGroup;
        }
        else
        {
          X_ChannelMap[channelNumber - 1] = 0;
          ClearActivePattern(channelNumber - 1); 
        }
        
    }
    //PORTE &=~ BIT0; //DEBUG ONLY set PE1 low
  }
  

}


void TransferPatternEEPROM ( UNS8 patternID, UNS8 write )
{
  UNS16 addr;
   
  //Get the starting EEPROM address based on the patternID
  if( (patternID > 0) && (patternID <= MAX_PATTERNS) )
  {
    addr = PATTERNS_EEPROM_ADDRESS + BYTES_PER_PATTERN*(patternID - 1); 

    if(write) //Write from OD to EEPROM
    {
      EEPROM_write(addr++, &channel_PatternTransfer, 1);
      EEPROM_write(addr++, &commandID_PatternTransfer, 1);
      EEPROM_write(addr++, &numPoints_PatternTransfer, 1);
      EEPROM_write(addr, xValues_PatternTransfer, PATTERN_ARRAYSIZE);
        addr+=PATTERN_ARRAYSIZE;
      EEPROM_write(addr, pwValues_PatternTransfer, PATTERN_ARRAYSIZE);
        addr+=PATTERN_ARRAYSIZE;
      EEPROM_write(addr, ampValues_PatternTransfer, PATTERN_ARRAYSIZE);
    }
    else  //Read from EEPROM to OD
    {
      EEPROM_read(addr++, &channel_PatternTransfer, 1);
      EEPROM_read(addr++, &commandID_PatternTransfer, 1);
      EEPROM_read(addr++, &numPoints_PatternTransfer, 1);
      EEPROM_read(addr, xValues_PatternTransfer, PATTERN_ARRAYSIZE);
        addr+=PATTERN_ARRAYSIZE;
      EEPROM_read(addr, pwValues_PatternTransfer, PATTERN_ARRAYSIZE);
        addr+=PATTERN_ARRAYSIZE;
      EEPROM_read(addr, ampValues_PatternTransfer, PATTERN_ARRAYSIZE);
    }
  }
  
}
 

void updateStimTask()
{
  UNS8 i;
  
  
  for (i =0; i<NUM_CHANNELS; i++)
  {
    if(startPulse[i] && !setupComplete[i])
    {
      PORTE |= BIT1; //DEBUG ONLY set PE1 high
      runStimTask(i);
      PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
      setupComplete[i] = 1;  
    }
  }
  
}
/**
 * @brief This task is run on the background thread to update stimulus parameters (PW, amp).
 *        Stimulus period is determined by the SYNC timing which runs the scheduler.  
 *        The amplitude is checked for not exceeding the safety limit.
 *         The method for determining stimulus parameters varies depending on mode:
 *        In Y_Manual, Record_X, and Produce_X, stim parameters are set directly from ChanX_SetValues (OD 3212.1-4)
 *        In X_Manual, Patient_Control, and Patient_Manual, stim parameters are interpolated
 *        using the active function group pattern for each channel stored in odPattern and 
 *        the X_Network value for each channel
 * @param chan 0-based channel number, gets adjusted to 1-based before being used
 */
volatile void runStimTask( UINT8 chan )
{
	
        //Task only runs when the scheduler sees a sync message.
	CO_Data * d = &ObjDict_Data;
	UINT8 pw, ampl, ipi=Channel_IPI;
	UNS8 y_index;
        
        if(ipi<5)
          ipi = 5;
        else if (ipi>100) 
          ipi=100;
        
        
        PORTA |= 0x40; //PA.6
        
        // now change range
        chan += 1; // add one as artifact of scheduler (range here is 1,4)
	
	if(  !isStimCycleDone( BIT0 ))
	{
		
			y_index = (chan - 1) * 2;
			
			/* get required pw and ampl values */
			switch( d->nodeState )
			{
				default:
				
				case Waiting:
				
					pw   = 0;
					ampl = 0;
                                        
                                        // zero out the profiler interface values
                                        // note that stimTask is not currently called for non-synced modes.
                                        Chan1_SetValues[0] = 0x00;  
                                        Chan2_SetValues[0] = 0x00;
                                        Chan3_SetValues[0] = 0x00;
                                        Chan4_SetValues[0] = 0x00;
                                        Chan1_SetValues[1] = 0x00;  
                                        Chan2_SetValues[1] = 0x00;
                                        Chan3_SetValues[1] = 0x00;
                                        Chan4_SetValues[1] = 0x00;	
                                        
					break;
                                        
				case Stopped:
				
					pw   = 0;
					ampl = 0;
					break;
                                                                        
		                                       
				case Mode_Patient_Control:
				        // X_Network is zero based index - do linear interpolation
					interpChannelWaveform( chan - 1, X_Network[chan - 1], &pw, &ampl );
					break;
				case Mode_Patient_Manual:
				
					interpChannelWaveform( chan - 1, X_Network[chan - 1], &pw, &ampl );
					break;
				case Mode_X_Manual:
					interpChannelWaveform( chan - 1, X_Network[chan - 1], &pw, &ampl );
					break;
				
                                case Mode_Produce_X_Manual:
				case Mode_Y_Manual:
                                case Mode_Record_X: //mode RecordX now supports stim during recording
                                  switch (chan)
                                  {
                                    case 1:
                                    {
                                      pw   = Chan1_SetValues[0];                             
                                      ampl = Chan1_SetValues[1];
                                      break;                             
                                    }
                                  case 2:
                                    {
                                      pw   = Chan2_SetValues[0];                             
                                      ampl = Chan2_SetValues[1];
                                      break;                                     
                                    }
                                  case 3:
                                    {
                                      pw   = Chan3_SetValues[0];                             
                                      ampl = Chan3_SetValues[1];
                                      break;                                     
                                    }
                                  case 4:
                                    {
                                      pw   = Chan4_SetValues[0];                             
                                      ampl = Chan4_SetValues[1];
                                      break;                                     
                                    }
                                  }
			}
                        
                        
                        // safety feature -- don't exceed amplitude max
                        // default is 2mA for use with a nerve cuff.
	                if ( ampl > Channel_Config_AmpMax[y_index + 1])
                        {
                          ampl = Channel_Config_AmpMax[y_index + 1];
                        }
                        
                        
			frame.flag = configPulseChannel( chan, ampl, pw, ipi );
			
                        
			Y_Current[ y_index ]     = pw;
			Y_Current[ y_index + 1 ] = ampl;
                        
		
	}
        PORTA &=~0x40; //PA.6
      

}

void InitStimTaskValues(void)
{
  UNS8 i;
  Chan1_SetValues[0] = 0x00;  
  Chan2_SetValues[0] = 0x00;
  Chan3_SetValues[0] = 0x00;
  Chan4_SetValues[0] = 0x00;
  Chan1_SetValues[1] = 0x00;  
  Chan2_SetValues[1] = 0x00;
  Chan3_SetValues[1] = 0x00;
  Chan4_SetValues[1] = 0x00;
  
  for (i = 0; i < NUM_CHANNELS; i++)
  {
    
    Y_Current[ i * 2]     = 0;
    Y_Current[ i * 2 + 1 ] = 0;
    
    startPulse[i] = 0;
    syncCount[i] = 0;
  }
  
  syncPulse = 0;
  //VOS disabled in scheduler following completion of discharge
  //any pulses shceduled from the last received SYNC will still occur
}

//============================
//    LOCAL CODE
//============================



/**
 * @brief Linearly interpolates the PW and amp given an x value for the specified channel.  
 *        Interpolation is performed between 2 points in the odPattern struct.  x values
 *        must be monotonically increasing (first point should be 0, last point 255)
 * @param chan 0-based channel number
 * @param x pattern input
 * @param *pw output pulse width
 * @param *ampl output amplitude
 */

static void interpChannelWaveform( UINT8 chan, UINT8 x, UINT8 *pw, UINT8 *ampl )
{
	// do a linear interpolation assuming x values increase to the right.
	// chan = 0-3
	
	UNS8 *xPts;
	UINT8 *pwPts, *amplPts;
	UINT8 x1, x2;
	INT32 y1, y2;
	UINT8 pn, pmax;
        
      
          
        pmax    = odPattern[ chan ].num;
	xPts 	= odPattern[ chan ].x;
	pwPts 	= odPattern[ chan ].pw;
	amplPts	= odPattern[ chan ].ampl;
	
        //force PW and Amplitude to 0 in case the interpolation fails to set the value
        // (e.g. if current x value is greater than last x-value )
        *pw   = 0;
	*ampl = 0;

          
        if( pmax < 2 || pmax > PATTERN_ARRAYSIZE )
	{
          return; /* illegal curve */
	}
	
	for( pn=0 ; pn < pmax - 1; pn++ )
	{
		x1 = *(xPts + pn);
		x2 = *(xPts + pn + 1); 
		
		if( x2 > x1 && x >= x1 && x <= x2 )
		{
			/* x is within the curve so calc pw and ampl */
			y1 = *(pwPts + pn );
			y2 = *(pwPts + pn + 1 );
			
			*pw = y1 + ((y2 - y1) * (x - x1) / (x2 - x1)) ;
			
			y1 = *(amplPts + pn );
			y2 = *(amplPts + pn + 1 );
			
			*ampl = y1 + ((y2 - y1) * (x - x1) / (x2 - x1)) ;
			
			break;
		}
	}
}


//============================
//    INTERRUPT SERVICE ROUTINES
//============================


//============================
//    HARDWARE SPECIFIC CODE
//============================


