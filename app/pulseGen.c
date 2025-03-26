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
#include "app.h"
#include "objdict.h"


// -------- DEFINITIONS ----------

#define MIN_PERIOD				20
#define MAX_PERIOD				1000
#define MAX_AMPLITUDE	   		200


#define CONFIG_SPI_DAC()		SPCR = B(SPE) | B(MSTR)			// 8MHz/4 
#define SPI_DONE()			(BITS_TRUE( SPSR, B(SPIF) ))

#define PIN_STIM_EN_TRUE()		CLR_BITS( PORTB, BIT4 )
#define PIN_STIM_EN_FALSE()		SET_BITS( PORTB, BIT4 )

#define PIN_VOS_EN_TRUE()		SET_BITS( PORTC, BIT3 )
#define PIN_VOS_EN_FALSE()		CLR_BITS( PORTC, BIT3 ) 

#define PIN_DAC_CS_TRUE()		CLR_BITS( PORTC, BIT1 )
#define PIN_DAC_CS_FALSE()		SET_BITS( PORTC, BIT1 )

#define PIN_OUT_EN_TRUE(n)		CLR_BITS( PORTA, (n) )
#define PIN_OUT_EN_FALSE(n)		SET_BITS( PORTA, (n) )

#define PIN_DAC2_CS_TRUE()		CLR_BITS( PORTC, BIT5 )
#define PIN_DAC2_CS_FALSE()		SET_BITS( PORTC, BIT5 )


#define SET_AMPLITUDE_DAC(h,l)	{	CONFIG_SPI_DAC();	/* config spi port */	\
					PIN_DAC_CS_TRUE();	/* select chip  */	\
					SPDR = (h);		/* send high byte  */	\
					  while( !SPI_DONE() );	/* wait */	\
					SPDR = (l);		/* send low byte */	\
					  while( !SPI_DONE() );	/* wait */	\
					PIN_DAC_CS_FALSE();	/* deselect chip */	}


#define RESET_PULSER()		{	TCCR1B = 0; 					/* stop timer */		\
					TCCR1A = B(COM1A1) | B(COM1B1) | B(COM1C1);	/* low on match */		\
					TCCR1C = B(FOC1A) | B(FOC1B); 			/* force le, te low */	        \
					TCCR1C = B(FOC1C);				/* force rechg low seq */	\
					TCNT1  = 0;					/* init time */			}


#define START_PULSER(le,te,re)	{	TCCR1A |=  B(COM1A0);			/* le high on match */			\
					TCCR1C  =  B(FOC1A);			/* force le  */				\
					TCCR1A &= ~B(COM1A0);			/* le low on match */			\
					TCCR1A |=  B(COM1B0) | B(COM1C0);	/* on match: te hi, rechg hi */	        \
					OCR1A = (le);				/* set le match time */			\
					OCR1B = (te);				/* set te match time */			\
					OCR1C = (re);				/* set rechg match time */		\
					TIFR1   = B(OCF1B) | B(OCF1C);		/* clear match flags */			\
					TCCR1B |= B(CS10);			/* start timer without prescalar */	}


#define SET_VOS_DAC(c,h,l)	{	CONFIG_SPI_DAC();	/* config spi port */	\
					PIN_DAC2_CS_TRUE();     /* select chip*/	\
                                        SPDR = (c);		/* send command byte*/	\
					  while( !SPI_DONE() );	/* wait */	        \
                                        SPDR = (h);	        /* sendhigh byte*/	\
					  while( !SPI_DONE() );	/*  wait */	        \
					SPDR = (l);	        /* send low byte*/	\
					  while( !SPI_DONE() );	/* wait */	        \
					PIN_DAC2_CS_FALSE();	/* deselect chip */	}

#define VOS_MINIMUM  4*60 //4V,  VOS_MINIMUM is recovered from VIN, so is 


#define IS_TE_DONE()			BITS_TRUE( TIFR1, B(OCF1B) )
#define IS_RE_DONE()			BITS_TRUE( TIFR1, B(OCF1C) )
#define PULSER_COUNTS   TCNT1           /*Current time count of timer used for pulser*/


//bitbanged LE time is 
//8MHz ~31us
//4MHz ~15us
//2MHz ~7us
//1MHz ~3us
//Total LE_OFFSET >=34us (Using 40us)
#if (FOSC==8000)
  #define LE_OFFSET  296 // 37 us
#endif
#if (FOSC==4000)
  #define LE_OFFSET  132 // 33 us
#endif
#if (FOSC==2000)
  #define LE_OFFSET  50 // 25 us
#endif
#if (FOSC==1000)
  #define LE_OFFSET  9 // 9 us
#endif
//at 4MHz, the setup time is 7us before timer starts, so 3us gives 10us total for DAC to stabilize
//at 1MHz, the setup time is 30us, so DAC has 33us to stabilize.  Needs to be > 0, otherwise timer event won't occur  
#define REGMEAS_OFFSET			(10)*(FOSC/1000) 		// 10 usec?



// --------   DATA   ------------
UINT8 setupVOSComplete = 0;


static UINT8 const outEnablePin[ MAX_PULSE_CHAN ] = 
{
	BIT0, BIT1, BIT2, BIT3
};




// -------- PROTOTYPES ----------
static void updateDacBits( UINT8 chan );
static UINT16 calcDacBits( UINT8 iSet );

 
//============================
//    GLOBAL CODE
//============================
/**
 * @brief initialize pulse generation: sets and enables compliance voltage
 */
void initPulseGenerator( void )
{
	UINT8 chan;
	
	RESET_PULSER();
	         
	//zero pulse amplitude (DAC)
        //command = 0: Load input register; DAC register immediately updated (also exit shutdown).
        SET_AMPLITUDE_DAC( 0, 0 );  	
        
	/* build dac bits per channel */
	for( chan=0 ; chan<MAX_PULSE_CHAN ; chan++ )
	{
		updateDacBits( chan );
	}

	
}

/**
 * @brief sets and eneables/disables VOS, Output Stage (compliance) voltage 
 * @param stim: 
 *          0: not in a stim mode, disable VOS
 *          1: in stim mode and stim active, enable and set VOS 
 *          2: in stim mode but stim not active (go down from StimVOS), 
 *          3: in stim mode but stim not active (go up from Off)
 *  Note VOS  will be approximately equal to VIN if MinVOS < VIN
 */
void configVOS( UINT8 stim )
{
    UINT8 highbyte, lowbyte;
    UINT16 vos, vosStep;

    switch (stim)
    { 
      case 0: //exit stim mode
        PIN_STIM_EN_TRUE(); //open discharge switch prior to disabling VOS!
        PIN_VOS_EN_FALSE();
        setupVOSComplete = 0;
        break;
        
      case 1: //going up from minVOS (Assuming VOS already Enabled) if necessary
        if(Channel_MinVOS != Channel_StimVOS) 
        {
            vos = Channel_MinVOS;
            vosStep = (Channel_StimVOS - vos)/StimVOSsteps;
                  
            for(UINT8 step=0; step < StimVOSsteps-1; step++)
            {
              vos += vosStep;
              
              highbyte = (UINT8)((vos & 0x0FF0) >> 4);
              lowbyte =  (UINT8)((vos & 0x000F) << 4);
              
              SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
            }
            
            highbyte = (UINT8)((Channel_StimVOS & 0x0FF0) >> 4);
            lowbyte =  (UINT8)((Channel_StimVOS & 0x000F) << 4);
            
            SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
        }
        break;
    
      case 2: //go down from StimVOS if necessary
        if(Channel_MinVOS != Channel_StimVOS) 
        {
          highbyte = (UINT8)((Channel_MinVOS & 0x0FF0) >> 4);
          lowbyte =  (UINT8)((Channel_MinVOS & 0x000F) << 4);
        
          SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
        }
        break;
      
      case 3: //going up from off (Assuming VOS is not already enabled)
        vos = VOS_MINIMUM;
        vosStep = (Channel_MinVOS - vos)/MinVOSsteps;
        
        //Turn on VOS at Minimum.  Then start stepping up
        highbyte = (UINT8)((vos & 0x0FF0) >> 4);
        lowbyte =  (UINT8)((vos & 0x000F) << 4);
        SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
        PIN_VOS_EN_TRUE();
          
        for(UINT8 step=0; step < MinVOSsteps-1; step++)
        {
          vos += vosStep;
          
          highbyte = (UINT8)((vos & 0x0FF0) >> 4);
          lowbyte =  (UINT8)((vos & 0x000F) << 4);
          
          SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
        }
         
        //set final value
        highbyte = (UINT8)((Channel_MinVOS & 0x0FF0) >> 4);
        lowbyte =  (UINT8)((Channel_MinVOS & 0x000F) << 4);
        
        SET_VOS_DAC( 0x3 << 4, highbyte, lowbyte);
        
        setupVOSComplete = 1;
        break;
    }
    
}


UINT8 configPulseChannel( UINT8 chan, UINT8 ampl, UINT8 width, UINT8 ipi )
{
	// RETURN 0=ok, else errors
	// chan = 1-4
        // this function controls the actual pulse on a per channel basis
  
	struct PulseDef *pulse;
	UINT8 status = 0;
	UINT16 dacBits;
	
	if( ampl <= MAX_AMPLITUDE
	&&	chan > 0 
	&&	chan <= MAX_PULSE_CHAN )
	{
		chan-- ;
		pulse = &frame.pulseDef[ chan ];
		dacBits = calcDacBits( ampl );			//minimize time interrupts are off

		DISABLE_INTERRUPTS();
		
		pulse->amplitude  = ampl;
		pulse->duration   = width *(FOSC/1000);
		pulse->ipInterval = ipi *(FOSC/1000);
		pulse->dacBits.w  = dacBits;
		
		ENABLE_INTERRUPTS();
	}
	else
		status = 1;


	return status;
}


//UINT8 configPulsePeriod( UINT16 period )
//{
//	// RETURN 0=ok, else errors
//
//	UINT8 status=0;
//
//
//	if( period >= MIN_PERIOD
//	&&	period <= MAX_PERIOD )
//	{
//		DISABLE_INTERRUPTS();
//		
//		frame.period = period;
//		
//		ENABLE_INTERRUPTS();
//	}
//	else
//		status = 1;
//
//
//	return status;
//}

UINT8 isStimCycleDone( UINT8 mask )
{
	// Test&clr 'end-of-Period' flag: each caller should use a different mask bit.
	UINT8 status;
	
	
	DISABLE_INTERRUPTS();
	
	status = frame.flag & mask;
	CLR_BITS( frame.flag, mask );
	
	ENABLE_INTERRUPTS();
	
	return status;
}



//============================
//    LOCAL CODE
//============================
static void updateDacBits( UINT8 chan )
{
	// convert dac bits once when changes
	struct PulseDef *pulse = &frame.pulseDef[ chan ];
	
	
	pulse->dacBits.w = calcDacBits( pulse->amplitude ); 
}

static UINT16 calcDacBits( UINT8 iSet )
{
 	//(JML from AXC) PG4B resistor value 56 replaces 68 
        //PG4A
        // MAX5355 dac: calculate formatted dac bits (Bits << 3) as:
	// bits = iSet * 1024 * 68R * 65536 * 8 / 18000 (/65536 implied);
	// Return: result in MS16bits (/65536 implied) using 32bit math;
	// where iSet is in 100uA units.
	// After >>16, add 4 for rounding, before masking off 3lsbs.
        //PG4B
	// bits = iSet * 1024 * 56R * 65536 * 8 / 18000 (/65536 implied);
  
	UINT32 dacBits;


	//dacBits = iSet * 2028179UL ; //PG4A (JML)
        dacBits = iSet *  2462788UL ; //PG4B (JML) = above*68/56. Why not 1670265UL according to equation?

	return ((dacBits >> 16) + 4) & 0x1ff8;
}

/*!
** @file   pulseGen.c
** @author Jay Hardway
** @date   April 10, 2010
**
** @brief StimPulse directly controls the IO pins to shape the pulse.
**
**
*/


volatile void StimPulse(UINT8 channel)
{
	//channel -= 4; // sets channel from zero to three

	struct PulseDef *pulse;
	UINT8 outPin;
        UINT8 analogComp;
	UINT16 leEdge, trEdge, recharge, regMeas ;
	
	/* generate output pulses */
	if( frame.pulseDef[ channel ].ipInterval > 0 && frame.pulseDef[ channel ].duration > 0) // check for a non-zero pulse
	{
		/* the time is right and the pulse is enabled, so send it */
		pulse = &frame.pulseDef[ channel ];
		outPin = outEnablePin[ channel ];

		PIN_STIM_EN_TRUE();

		/* meas: dacSel = 12usec */
		SET_AMPLITUDE_DAC( 	pulse->dacBits.b[1], \
					pulse->dacBits.b[0]  );

		/* configure edge timer hardware */
		leEdge   = LE_OFFSET;
		trEdge   = leEdge + pulse->duration;
		recharge = trEdge + pulse->ipInterval;
                
                if(pulse->duration > REGMEAS_OFFSET)
                  regMeas  = leEdge + pulse->duration - REGMEAS_OFFSET;
                else
                  regMeas = 0;

		START_PULSER( leEdge, trEdge, recharge );

		PIN_OUT_EN_TRUE( outPin );
						
                //Is pulse in regulation?
                //if regMeas=0, this takes 10 clock cycles.  This happens before LE. 
                //if regMeas>0, this takes 10-17 clock cycles after the triggered measurment time
                //Therefore, REGMEAS_OFFSET must be > 17 clock cycles
                //note analogComp is invalid if regMeas=0;
                while( PULSER_COUNTS < regMeas );       //#clocks=7-8 per iter
                analogComp = ACSR;                      //#clocks=2
                
                while( !IS_TE_DONE() );

		PIN_OUT_EN_FALSE( outPin );

		SET_AMPLITUDE_DAC( 0, 0 ); //not necessary on PG4C

		while( !IS_RE_DONE() );

		PIN_STIM_EN_FALSE();

		RESET_PULSER();
                
                //If the pulse was greater than REGMEAS_OFFSET, then check if pulse 
                //was in regulation:
                //AIN0(HIZVT)>AIN1(STCC)-->ACO is set
                //If pulse was shorter, store a 2
                if(regMeas)
                {
                  if(BITS_TRUE(analogComp, B(ACO))) 
                    Channel_InRegulation[ channel ] = 1;  
                  else
                    Channel_InRegulation[ channel ] = 0;  
                }
                else
                {
                  Channel_InRegulation[ channel ] = 2;   
                }
                
	}
	
	
	
}





//============================
//    HARDWARE SPECIFIC CODE
//============================


