// Doxygen
/*!
** @defgroup scanmanagement Scan Management of CAN link
** @ingroup userapi
*/
/*!
** defgroup canscanner CAN Scanner
** @ingroup scanmanagement
*/
/*!
** @file   runCANserver.h
** @author Coburn
** @date   2/14/2010
** 
**
** @brief Header file for runCANserver. 
** @ingroup canscanner
**
*/      
#ifndef RUNCANSERVER_H
#define RUNCANSERVER_H

/**********************************************/
/*              Prototypes                    */
/**********************************************/

void InitCANServerTask( void );
void RunCANServerTask( void );
UNS8 processBOOT( CO_Data* d, Message *m );

#endif