
//    pulseGen: .c    .

/*	HOW IT WORKS:  Stim runs within the 1msec Timer0 interrupt, to assure timeliness.
	Here we generate the pulse waveforms on each channel per frame rates.
	Use the resident config functions to configure pulses and frame rates.

	THE ISR USES SPI TO CONFIG AMPLITUDE DAC SO BLOCK INTERRUPTS AROUND BACKGROUND
	DAC USAGE^^;
*/


#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "pulseGen.h"
#include "objdict.h"
#include "objacces.h"
#include "scheduler.h"
#include "stimTask.h"
#include "can_AVR.h"
#include "objdictdef.h"
#include "iar.h"
#include "app.h"

// -------- DEFINITIONS ----------

#define TIME_CUSHION 10
#define OPEN_DISCHARGE_SWITCH()	CLR_BITS( PORTB, BIT4 )  //equivalent to PIN_STIM_EN_TRUE()
#define VOS_UP_TIME         2 //time in ms for power supply to stablize after raising VOS, before first scheduled pulse
#define MIN_DISCHARGE_TIME  4 //time in ms required after a pulse before opening discharge switch 




// --------   DATA   ------------
volatile UINT8 tickCount[4] = {0,0,0,0}, syncCount[4] = {0,0,0,0}, startPulse[4] = {0,0,0,0}, setupComplete[4] = {0,0,0,0};
UINT8 vosTiming = 0;
static UINT32 sysTimer;
UNS8 numScheduledStimChannels = 0;



/*!
** 
**
** @param none
** 
** @return
**/
void InitScheduler(void)
{

        /* create 1msec system tick on TMR0*/
        #if (FOSC == 8000)
          OCR0A = (UNS8) 124;  // (124+1)*8us = 1.000 ms
        #endif
        #if (FOSC == 4000) 
          OCR0A = (UNS8) 62;  // (62+1)*16us = 1.008 ms
        #endif
        #if (FOSC == 2000) 
          OCR0A = (UNS8) 30;  // (30+1)*16us = 0.992 ms
        #endif
        #if (FOSC == 1000) 
          OCR0A = (UNS8)124;  // (124+1)*8us = 1.000 ms
        #endif
          
	TCCR0A = B(WGM01) ;					// CTC no output pin
	TIMSK0 = B(OCIE0A) ;					// enable timer OC interrupt
	
        #if (FOSC == 1000)
          TCCR0A |= B(CS01);		                // START clock at 1MHz/8 (8us)
        #elif
          TCCR0A |= B(CS01) | B(CS00) ;		                // START clock at 8MHz/64 (8us), 4MHz/64 (16us), 2MHz/64 (32us)
        #endif
       
        InitSchedulerOD();  
}

void InitSchedulerOD(void)
{
     UNS8 i;

    // find the number of channels that are scheduled to stim
    numScheduledStimChannels = 0;
    vosTiming = 0xFF;
    for (i = 0; i < 4; i++)
    {
      //channel will stim if the stim is scheduled not before setup and not after one period following sync
      if (StimTiming[i] < 0xFF)
        numScheduledStimChannels++;
      
      if (StimTiming[i] < vosTiming)
        vosTiming = StimTiming[i];
    }
    
    if (vosTiming > 1)
      vosTiming-=VOS_UP_TIME;
    else
      vosTiming=0;
}


/**
 *@brief Resynchronizes MCU timer 0 based on arrival of CAN sync message
 *    Normally SyncPush (OD entry) should be set to 0, so TCNT0 gets reset.  
 *    SyncPush is in 16us increments.  If non zero, it will give this module a 
 *    headstart relative to other modules.
 *    invalid SyncPush (later than output compare = 62) won't trigger a resync.
 *
*/
void SyncScheduler(void)
{

  if (SyncPush < OCR0A)
  {
    TCNT0 = SyncPush; 
  }
  
}

//============================
//    TIMER FOR ACCEL & TEMP
//============================
/**
 * @brief gets system time
 * @return t time
 */
UINT32 getSystemTime( void )
{
	UINT32 t;
	
	DISABLE_INTERRUPTS();
	
	 t = sysTimer;
	 
	ENABLE_INTERRUPTS();
	
	return t;
	
}

//============================
//    INTERRUPT SERVICE ROUTINES
//============================
#pragma vector=TIMER0_COMP_vect
// 	This is a 1msec continuous tick to initiate pulses and 
	//	maintain system time.

__interrupt void stimTick_ISR(void)
{
  UNS8 i;
  static UNS8 channelsCompleted=0, initStimVOS=0, tick=0, dischargeCounter=0;
 
  //tick is used for AUTOSYNCS and controlling discharge switch.  Resets to 0 on SYNCs and AUTOSYNCSs 
           //(rolls over at 8bit=255ms if no SYNC/AUTOSYNC
  
  sysTimer++ ; //used for 1ms accuracy system clock, free running (rolls over at 32bit = 1,193 hours)

  //PORTE |= BIT0; //DEBUG ONLY set PE1 high
  
  /* Check if SYNC received.  AUTOSYNCing is only enabled in Patient Operation Mode.  If AutoSyncTime = 255, tick cannot exceed it*/ 
  if ( syncPulse || (tick > AutoSyncTime && getState( &ObjDict_Data ) == Mode_Patient_Control ) )
  {
      if( syncPulse)  //real SYNC pulse
      {
        AutoSyncCount = 0;
      }
      else  // AUTOSYNC pulse
      {
        AutoSyncCount++;
        TotalAutoSyncCount++;
        if (AutoSyncCount > MaxAutoSyncCount)
        {
          MaxAutoSyncExceededCount++;
          tick = 0;
          return; 
        }
      }

      PORTE |= BIT1; //DEBUG ONLY set PE1 high
      
      //Initialize this SYNC period 
      syncPulse = 0; 
      channelsCompleted = 0;
      initStimVOS = 1;
      tick = 0;
      
      //Per channel Initializations
      for( i=0; i<NUM_CHANNELS; i++)
      {
        
        if ( ++syncCount[i] >= SyncInterval[i])
        {
          startPulse[i] = 1;
          tickCount[i] = 0;
          syncCount[i] = 0;
          setupComplete[i] = 0;
        }
      }

      PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
      
     
  }
  
  tick++;  
    
  //First, increment all the channel counters
  for( i=0; i<NUM_CHANNELS; i++)
  {
    if (  startPulse[i] )
    {
      tickCount[i]++; //increment counter       
    }
  }
  
  if (!setupVOSComplete) //VOS still ramping up to MinVOS
    return;
  
  if( initStimVOS && tick >= vosTiming)
  {
      PORTE |= BIT1; //DEBUG ONLY set PE1 high
      if(SetupAnode)
        ENABLE_ANFON(); //JML note: This SHOULD happen before configuring VOS.  
                        //Otherwise there is a very sharp transient when anode is connected
                        //See notes on Anode Transient
      
      configVOS(1);  //Ramp up to StimVOS
      initStimVOS = 0; 

      PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
      return; //don't allow anything else to happen during this ISR tick
  }      
  

  if(++dischargeCounter > DischargeTime) 
  {
    if(dischargeCounter > MIN_DISCHARGE_TIME)
    {
      OPEN_DISCHARGE_SWITCH();
      if( getState( &ObjDict_Data ) == Waiting || getState( &ObjDict_Data ) == Stopped )
      {
        configVOS(0);
      }
    }
  }
    

  for( i=0; i<NUM_CHANNELS; i++)
  {
    if ( startPulse[i])
    {
      if (tickCount[i] >= StimTiming[i] )
      {
         if(setupComplete[i]) 
         {
           PORTE |= BIT1; //DEBUG ONLY set PE1 high
           StimPulse(i);  //only stim if setup was completed
           PORTE &=~ BIT1; //DEBUG ONLY set PE1 low

           startPulse[i] = 0;  //done with this channel
           dischargeCounter = 0; //reset time needed for discharge
            
           ActualStimTiming[i] = tickCount[i]; //indicate actual stim time
           if(tickCount[i] > MaxActualStimTiming[i])
             MaxActualStimTiming[i] = tickCount[i];
           
           //if this was our last scheduled channel, turn down VOS
           if(++channelsCompleted==numScheduledStimChannels) 
           { 
             PORTE |= BIT1; //DEBUG ONLY set PE1 high
             if(SetupAnode)
                DISABLE_ANFON();
             
             configVOS(2); //reduce VOS to MinVOS
             PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
           }
           
         }
         return; //don't allow anything else to happen during this ISR tick
      }
    }
  }
  //PORTE &=~ BIT0; //DEBUG ONLY set PE1 low
}