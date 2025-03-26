/*
This file is part of CanFestival, a library implementing CanOpen Stack. 


Copyright (C): Edouard TISSERANT and Francis DUPIN
modified by JDCC

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
 * @file   sync.c
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief <BRIEF> sync.c
 */

/** 
 * @defgroup sync Synchronisation Object
 *  SYNC object is a CANopen message forcing the receiving nodes to sample the inputs mapped into synchronous TPDOS.
 *  Receiving this message cause the node to set the outputs to values received in the previous synchronous RPDO.
 *  @ingroup comobj
 */

#include "data.h"
#include "sync.h"
#include "canfestival.h"
#include "sysdep.h"
#include "ObjDict.h"
#include "app.h"

/* Prototypes for internals functions */
void SyncAlarm(CO_Data* d, UNS32 id);
UNS32 OnCOB_ID_SyncUpdate(CO_Data* d, const indextable * unsused_indextable, 
	UNS8 unsused_bSubindex);


/**
 * @ingroup sync
 * @brief <BRIEF> SyncAlarm                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure
 * @param id not used 
 */    
void SyncAlarm(CO_Data* d, UNS32 id)
{
	sendSYNC(d) ;
}


/**
 * @ingroup sync
 * @brief This is called when Index 0x1005 is updated.                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure    
 * @param unsused_indextable                                                                       
 * @param unsused_bSubindex 
 * @return
 */  
UNS32 OnCOB_ID_SyncUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	startSYNC(d);
	return 0;
}

/**
 * @ingroup sync
 * @brief <BRIEF> startSYNC                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure                                                                                        
 */  
void startSYNC(CO_Data* d)
{
	if(d->syncTimer != TIMER_NONE){
		stopSYNC(d);
	}

	RegisterSetODentryCallBack(d, 0x1005, 0, &OnCOB_ID_SyncUpdate);
	RegisterSetODentryCallBack(d, 0x1006, 0, &OnCOB_ID_SyncUpdate);

	if(*d->COB_ID_Sync & 0x00000080ul && *d->Sync_Cycle_Period)
	{
		d->syncTimer = SetAlarm(
				d,
				0 /*No id needed*/,
				&SyncAlarm,
				MS_TO_TIMEVAL(*d->Sync_Cycle_Period), 
				MS_TO_TIMEVAL(*d->Sync_Cycle_Period));
	}
}

/**
 * @ingroup sync
 * @brief <BRIEF> stopSYNC                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure                                                                                        
 */   
void stopSYNC(CO_Data* d)
{
    RegisterSetODentryCallBack(d, 0x1005, 0, NULL);
    RegisterSetODentryCallBack(d, 0x1006, 0, NULL);
	d->syncTimer = DelAlarm(d->syncTimer);
}


/** 
 * @ingroup sync
 * @brief Transmit a SYNC message on CAN bus
 * @param *d Pointer on a CAN object data structure
 * @return
 */
UNS8 sendSYNCMessage(CO_Data* d)
{
  Message m;
  
  MSG_WAR(0x3001, "sendSYNC ", 0);
  
  m.cob_id = UNS16_LE(*d->COB_ID_Sync);
  m.rtr = NOT_A_REQUEST;
  m.len = 0;
  
  return canSend(d->canHandle,&m);
}


/** 
 * @ingroup sync
 * @brief Transmit a SYNC message and trigger sync TPDOs
 * @param *d Pointer on a CAN object data structure
 * @return
 */
UNS8 sendSYNC(CO_Data* d)
{
  UNS8 res;
  res = sendSYNCMessage(d);
  processSYNC(d, NULL) ; 
  return res ;
}

/** 
 * @ingroup sync
 * @brief This function is called when the node is receiving a SYNC message (cob-id = 0x80).
 *  - Modified from original CANFestival code to include reading a message from the SYNC object
 *  - Check if the node is in OERATIONAL mode. (other mode : return 0 but does nothing).
 *  - Get the SYNC cobId by reading the dictionary index 1005, check it does correspond to the received cobId
 *  - Trigger sync TPDO emission 
 * @param *d Pointer on a CAN object data structure
 * @return 0 if OK, 0xFF if error 
 */
UNS8 processSYNC(CO_Data* d, Message* m)
{

  UNS8 res;
  
  if(m) //if there is a non-NULL message, let the App use the message
    processSYNCMessageForApp(m);
  
  MSG_WAR(0x3002, "SYNC received. Proceed. ", 0);
  
  (*d->post_sync)(d);

  /* only operational state allows PDO transmission */
  if(! d->CurrentCommunicationState.csPDO) 
    return 0;

  res = _sendPDOevent(d, 1 /*isSyncEvent*/ );
  
  /*Call user app callback*/
  (*d->post_TPDO)(d);
 
  return res;
  
}

/**
 * @ingroup sync
 * @brief <BRIEF> _post_sync                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure
 */  
void _post_sync(CO_Data* d){}

/**
 * @ingroup sync
 * @brief <BRIEF> _post_TPDO                                                                                                                                                                                              
 * @param *d Pointer on a CAN object data structure
 */ 
void _post_TPDO(CO_Data* d){}
