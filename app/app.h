//    file: app.h     HEADER FILE.

#ifndef APP_H
#define APP_H


#include "sys.h"
#include "can.h"
#include "data.h"


// -------- DEFINITIONS ----------

#define TIMEOUT_ms(n)	((n))		
#define TIMEOUT_sec(n)	(TIMEOUT_ms((n) * 1000L))	
#define START_DELAY_MS  4UL	//to be multiplied by nodeID on startup

// --------   DATA   ------------


// -------- PROTOTYPES ----------

UINT32 getSystemTime( void );
UINT8 isTimedOut( UINT32 *tRef, UINT32 tAlarm );
void resetTimeOut( UINT32 *tRef );
void processSYNCMessageForApp(Message* m);
UINT8 txRxSpi( UINT8 d );

void StartNodesFunc(CO_Data* d, Message *m);
void StopNodesFunc(CO_Data* d, Message *m);
void EnterWaitingFunc(CO_Data* d, Message *m);
void EnterPatientOperationFunc(CO_Data* d, Message *m);
void EnterXManualFunc(CO_Data* d, Message *m);
void EnterYManualFunc(CO_Data* d, Message *m);
void EnterStopStimFunc(CO_Data* d, Message *m);
void EnterPatientManualFunc(CO_Data* d, Message *m);
void EnterProduceXManualFunc(CO_Data* d, Message *m);
void EnterRecordXFunc(CO_Data* d, Message *m);

#endif
 
