//    stimTask: .h     HEADER FILE.

#ifndef STIMTASK_H
#define STIMTASK_H


#define DISABLE_ANFON()         (PORTC &=~ BIT2)
#define ENABLE_ANFON()          (PORTC |= BIT2)

// -------- DEFINITIONS ----------


// --------   DATA   ------------


// -------- PROTOTYPES ----------
void initStimTask( void );
void updateStimTask( void );
volatile void runStimTask( UNS8 );
void updateProfileMemory( void );
void UpdateActivePatterns ( UNS8, UNS8 );
void InitStimTaskValues( void );
void ClearActivePattern( UNS8 ch );
void ClearAllActivePatterns( void );
void TransferPatternEEPROM ( UNS8 patternID, UNS8 write );


#endif
 
