//    stimTask: .h     HEADER FILE.

#ifndef SCHEDULER_H
#define SCHEDULER_H


// -------- DEFINITIONS ----------
#define NUM_CHANNELS        4

// --------   DATA   ------------

extern volatile unsigned char syncPulse;
extern unsigned char vosTiming;
extern volatile unsigned char setupComplete[4], startPulse[4];

// -------- PROTOTYPES ----------
void InitScheduler( void );
void InitSchedulerOD(void);
void SyncScheduler(void);



#endif
 