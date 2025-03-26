/**
 * @file   app.c
 * @author JDC
 * @brief entry point
 * @details System Resource Usage:
 * - tmr3: used to time canfestival operations.
 * - tmr3 isr.
*/

/******************************************************************************
Project description:

// Svn Version:   $Rev: 165 $     $Date: 2020-07-24 13:55:17 -0400 (Fri, 24 Jul 2020) $

******************************************************************************/
#include "sys.h"
#include "canfestival.h"
#include "runcanserver.h"
#include "stimTask.h"
#include "pulseGen.h"
#include "app.h"
#include "eedata.h"
#include "objdict.h"
#include "scheduler.h"
#include "acceltemp.h"

#define CO_ENABLE_LSS

/******************************DATA*****************************************/
UNS16 volatile Delay = 20000;
UNS16 blink = 0;
UNS8  tempNodeID = 2;
UNS8 lastRequestedAddress = 0;
UNS8 commandByte = 0;
/***************************************************************************/

/******************************PROTOTYPES***********************************/
void sys_init( void );
void initNodeIDSerialNumber( void );
void ReadMemory( void );
UNS8 ReadLocalFlashMemory(void);
UNS8 ReadEprom(void);
UNS8 WriteEEProm ( void );
/***************************************************************************/

/**
 * @brief Application entry point 
 */
void main( void )
{      
        sys_init();
        
	if (CheckRestoreFlag()) 
        {
	  //Load custom settings from EEPROM into the OD
          RestoreValues();
        }
	else
        {
          //Use default settings from OD rather than what is stored in EEPROM
          //Then, overwrite EEPROM with these default settings
          SaveValues();
        }
        
        initStimTask();
        InitScheduler();  
        initAccelerometer();
	initTimer();     // timer for CAN Festival interrupts and alarms
	InitCANServerTask(); // also sets up CAN network -- 
        //needs to run after RestoreValues
        initDiagnostics();
        
	while (TRUE)
	{
		updateAccelerometer(); 
                
                updateStimTask();
                
                updateDiagnostics(); 

                runTemperatureTask();
                
		RunCANServerTask();
                
                Status_TestValue++;           
              

                  
                if (blink++ > 125)
                {
                  if (getState( &ObjDict_Data ) == Waiting) // reduce overhead on AI processing
                  {
                    ReadMemory();
                  }
                
	          PORTG &=~ BIT2;  //turn off heartbeat LED 
                  blink = 0;
                }  
                else if (blink > 100)
                {
                  PORTG |= BIT2;  //turn on heartbeat LED 
                }
                
                //PORTE |= BIT0;  //DEBUG indicate sleep
                SMCR |= BIT0; //Set Sleep Enable in Idle Mode
                asm("SLEEP");
                SMCR &=~BIT0; //Disable sleep
                //PORTE &=~ BIT0;  //DEBUG indicate wakeup

		
	}
	
}



//==================================
//    SYSTEM SPI AND TIMER
//==================================

#define SPI_DONE()			(BITS_TRUE( SPSR, B(SPIF) ))

/**
 * @brief R/W one byte via the SPI port, full duplex.  
 * @param d data byte to write	 
 */
UINT8 txRxSpi( UINT8 d )
{
    //Assumes SPCR & SPSR are configured.
	SPDR = d;		// put tx data
    while( !SPI_DONE() );	// wait for xfer done
    return( SPDR );	        // get rx data 
}


UINT8 isTimedOut( UINT32 *tRef, UINT32 tAlarm )
{
	
	if( (getSystemTime() - *tRef) > tAlarm )
	{
		return 1;
	}
	
	else
		return 0;
}


void resetTimeOut( UINT32 *tRef )
{
	*tRef = getSystemTime();
}

//==================================
//    APP SPECIFIC CANFESTIVAL
//==================================

/**
 * @brief Lets application process SYNC message if necessary
 * @param *m 8 byte message sent with SYNC objects from PM
 */
void processSYNCMessageForApp(Message* m)
{
    UNS8 i = 0;
    for (i = 0; i < m->len; i++)
      CommandValues[i] = m->data[i]; // used to guarantee an 8 byte array
    
    for (i = 0; i < 4; i++)
    {
      if(X_ChannelMap[i] > 0 && X_ChannelMap[i] <= m->len)
        X_Network[i] = CommandValues[X_ChannelMap[i]-1]; //JML: X_ChannelMap 1-based 
    }
    

}


/**
 * @brief Lets application process NMT_Start_Nodes (unused)
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void StartNodesFunc(CO_Data* d, Message *m)
{
  if ( d->nodeState == Hibernate) 
    setState(d,Waiting);
  else
    setState(d,Unknown_state);
}

/**
 * @brief Lets application process NMT_Stop_Nodes
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void StopNodesFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Stopped); 
  InitStimTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Waiting
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterWaitingFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Waiting); 
  StopWatchDog( d );
  InitStimTaskValues();
  //No VOS required
}

/**
 * @brief Lets application process NMT_Enter_Patient_Operation
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterPatientOperationFunc(CO_Data* d, Message *m)
{
  UNS8 Param1;
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    Param1 = (*m).data[2];
    UpdateActivePatterns ( Param1 , 1 ); // initialize pattern data before turning on state
    setState(d,Mode_Patient_Control);
    //Status_profileSelect = (Param1 & 0x0F);  // set profile to run
    //Control_CurrentGroup = Param1;
    configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Lets application process NMT_Enter_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterXManualFunc(CO_Data* d, Message *m)
{         
  UNS8 Param1;
  if (d->nodeState == Waiting) //must be in waiting
  {
    Param1 = (*m).data[2];
    UpdateActivePatterns ( Param1 , 1 ); // initialize pattern data before turning on state
    setState(d,Mode_X_Manual);
    //Status_profileSelect = (Param1 & 0x0F);  // set profile to run //JML: not sure this is used anywhere
    //Control_CurrentGroup = Param1;  //JML: not sure this is used anywhere
    StartWatchDog(d, 10000);
    configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Lets application process NMT_Enter_Y_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterYManualFunc(CO_Data* d, Message *m)
{  
  if (d->nodeState == Waiting) //must be in waiting
  {
    setState(d,Mode_Y_Manual);
    StartWatchDog(d, 10000);
    configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Lets application process NMT_Enter_Stop_Stim
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterStopStimFunc(CO_Data* d, Message *m)
{  
  //no restrictions on state entry
  setState(d,Stopped); 
  InitStimTaskValues();
  //No VOS required
}

/**
 * @brief Lets application process NMT_Enter_Patient_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterPatientManualFunc(CO_Data* d, Message *m)
{
  UNS8 Param1;
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    Param1 = (*m).data[2];
    UpdateActivePatterns ( Param1 , 1 ); // initialize pattern data before turning on state 
    setState(d, Mode_Patient_Manual);
     //Status_profileSelect = (Param1 & 0x0F);  // set profile to run
     //Control_CurrentGroup = Param1;
     StartWatchDog(d, 10000);
     configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Lets application process NMT_Produce_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterProduceXManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
     setState(d, Mode_Produce_X_Manual);
     //Status_profileSelect = (Param1 & 0x0F);  // set profile to run
     //Control_CurrentGroup = Param1;
     configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Lets application process NMT_Enter_Record_X
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 */
void EnterRecordXFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting)  //must be in waiting
  {
    setState(d, Mode_Record_X); 
    configVOS(3); //ramp up to MinVOS
  }
}

/**
 * @brief Command processor to let CE read various memory locations
 * @param none
 */
void ReadMemory(void)
{
      // clear status
        if (memorySelect > 0 )
          statusByteMemory = 0;
        
        if (memorySelect == 1)
          statusByteMemory = ReadLocalFlashMemory();
        else if (memorySelect == 4)
          statusByteMemory = ReadEprom();
        else if (memorySelect == 8)
          statusByteMemory = WriteEEProm();
        else if (memorySelect != 0)
          statusByteMemory = 4;
      
        memorySelect = 0;
       
}
/**
 * @brief Read flash memory in CPU
 * @param none
 */
UNS8 ReadLocalFlashMemory(void)
{
  UNS8 RMDataSize = sizeof(ReadMemoryData);
  UNS16 recordSize = FLASH_RECORD_SIZE;
  
  if (AddressRequest > (MAX_FLASH_MEMORY - recordSize))
    return 2;
  
  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  { 
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));   
    ReadLocalFlashData( AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8)AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);
    
    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }   
  return 1;
}
/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote reading of
 *        values in the AVR eeprom memory
 */
UNS8 ReadEprom(void)
{
  UNS8 RMDataSize = sizeof(ReadMemoryData);
  UNS16 recordSize = EEPROM_RECORD_SIZE;
  
  if (AddressRequest > (MAX_EEPROM_MEMORY - recordSize))
    return 2;
  
  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  { 
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));   
    EEPROM_read( (UNS16)AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8)AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);
    
    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }   
  return 1;
}

/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote writing of
 *        a single value in the AVR eeprom memory
 */
UNS8 WriteEEProm ( void )
{
  /*
  UNS16 recordSize;
  
  if (writeByteMemory > EEPROM_RECORD_SIZE)
    recordSize = EEPROM_RECORD_SIZE;
  else
    recordSize = writeByteMemory;
  
  if (AddressRequest > (MAX_EEPROM_MEMORY - recordSize))
    return 2;
  
  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  { 
    EEPROM_write((UNS16)AddressRequest, ReadMemoryData, recordSize);
    
    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    
    return 0;
  }
  else
    return 1;  
  
  */
  UNS8 status = 0;
  
  if (AddressRequest > MAX_EEPROM_MEMORY) // nominal address space
    return 2;
  else
    EEPROM_write((UINT16)AddressRequest, &writeByteMemory, 1);
  
  return status; 

}



//==================================
//    SYSTEM INIT
//==================================

/**
 * @brief Sets NodeId and S/N based on settings from bootloader.  Also has important role 
 * of delaying continuation based on node number
 */
void initNodeIDSerialNumber( void )
{
    UINT32 delay = START_DELAY_MS*1000UL; //convert to clock cycles, still running at Bootloader clock speed (1MHz)

    tempNodeID = *(UINT8   __farflash  *)0x1DF00;
    if (tempNodeID <= 0x0F && tempNodeID > 0)
    {
      Status_NodeId = tempNodeID;
    }
    
    setNodeId(&ObjDict_Data, Status_NodeId);
    
    ObjDict_obj1018_Serial_Number = *(UINT8 __farflash *)0x1FFFE + \
      ((UINT16)(*(UINT8 __farflash *)0x1FFFD) << 8);
    
    
    //startup delay is START_DELAY_MS*(nodeID-1), 
    delay = delay*(Status_NodeId-1)/13;
    
    //PORTE |= BIT1; //DEBUG ONLY set PE1 high
    while( delay-- ); //this line takes 13 clock ticks per iteration if delay is uint32
    //PORTE &=~ BIT1; //DEBUG ONLY set PE1 low
}

/**
 * @brief Initialize the IO and clock settings
 */
void sys_init( void )
{       
        
	// disable the wdt if it was enabled from the reset
        DISABLE_INTERRUPTS();
        WDTCR = (1 << WDCE)  | (1 << WDE);	
	WDTCR = 0; // clear the WDT enable bit

/*      //OLD PIN SETTINGS  
        // initialize io pins to support PON default conditions
	PORTA = 0x0f;		// chan selects
	DDRA  = 0xff;		// 1=output dir

	//SPI SS PB0 must be configed as output pin or master mode could fail
	PORTB = 0x11;		// miso=in, stim pulse control
	DDRB  = 0xf7;		// 1=output dir

	PORTC = 0x23;		// dac, vos, tempr selects >>(JML from AXC) was 0x03, PC.2 must be low.  PC.5 must be high
	DDRC  = 0xff;		// 1=output dir

	PORTE = 0xf3;		// tx, rx selects >>(JML from AXC) was 0x02...switch to 0xf3
	DDRE  = 0x02;		// 1=output dir >>(JML from AXC)was 0xfe...PE.2 and PE.3 must be inputs, PE.1 must be output 
        
        PORTG = 0x00;		// heartbeat
	DDRG  = 0x02;
*/


        //SEE PRJ-NNPS-SYS-SPEC-21 Bootloader and App Specification, Remote Module, Pin Settings
        //Set unconnected pins (N.C.) to input with Pull-ups enabled
        //DDRx:  0=input, 1=output
        //PORTx: 0=low, 1=high (output), 0=no pullup, 1=pullup (input)
        
	PORTA = 0xFF;		// pulled-up or set high
	DDRA  = 0x0F;		// nOUTEN1-4 are outputs

	PORTB = 0x97;		// MISO not pulled-up, STIMTE, nSTIMLE set low
	DDRB  = 0xF6;		// All outputs except MISO and N.C.

	PORTC = 0xF7;		// VOSEN low, all others pulled-up or set high. Note: ANFON now enabled by default!
	DDRC  = 0x2E;		// nDACS, ANFON, VOSEN, nDACS2 are outputs

	PORTD = 0xB8;		// SCL, SDA, ACCINT, and RXCAN not pulled up
	DDRD  = 0x20;		// TXCAN is output

	PORTE = 0xF3;		// HIZVT and STCC analog, so not pulled-up (see DIDR1)
	DDRE  = 0x00;		// All inputs
	
	PORTF = 0xF0;		// ADC inputs not pulled up, see DIDR0
	DDRF  = 0x40;		// TDO is output

	PORTG = 0x1B;		// HEARTBEAT low
	DDRG  = 0x04;		// HEARTBEAT is output
	
	DIDR0 = 0x0F;		// Disconnect digital I/O pins from ADC inputs (see PORTF)
	DIDR1 = 0x03;		// Disconnect digital I/O pins from STCC and HIZVT
	
        
//	  //>>For Debug ONLY!
//	  //remap PORTA as outputs, low
//	  PORTA = 0x0F;		
//	  DDRA  = 0xFF;	
        DDRE |= 0x73; //set PE0,1,4,5,6 as output
        PORTE &= ~0x73; //set PE0,1,4,5,6 low
//        PORTA |= 0x40; //JML test pin 6
//        //<<For Debug ONLY!
       
        CANGCON &=~ B(ENASTB); //put CAN in standby while adjusting clock
        //while(CANGSTA & B(ENFG)){}; //wait until disabled;
        //CAN will be reenabled by canInit routine after bittimings have been set for new clock speed
        
        initNodeIDSerialNumber(); //initialize serial number, node number
                                  //delay based on node number before increasing clock
        
        CLKPR = B(CLKPCE); // enable scale clock, Interrupts must be off.
                           // Next step takes 4 clock cycles
        #if (FOSC == 8000)
           CLKPR = 0;// write sysclk prescaler 
        #endif
        #if (FOSC == 4000)                    
           CLKPR = 1;// write sysclk prescaler 
        #endif
        #if (FOSC == 2000)                     
           CLKPR = 2;// write sysclk prescaler 
        #endif  
        #if (FOSC == 1000)                     
           CLKPR = 3;// write sysclk prescaler
        #endif
        
        ENABLE_INTERRUPTS();  
        
	
}

