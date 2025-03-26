/*
  This file is part of CanFestival, a library implementing CanOpen
  Stack.

  Copyright (C): Edouard TISSERANT and Francis DUPIN
  modified by JDC Consulting. Master copy kept at
  https://jdc-consulting.unfuddle.com/svn/jdc-consulting_mastercodecanfestival/
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
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
  USA
*/

/**
 * @file   nmtSlave.c
 * @author Edouard TISSERANT and Francis DUPIN, modified by JDC
 * @brief This file is used only with the operational slave -
 * usually the Remote Modules.
*/

/** 
 * @defgroup nmtslave NMT Slave
 *  @brief The NMT Slave methods are called automatically when a NMT message from Master are received. 
 *  @ingroup networkmanagement
 */

#include "nmtSlave.h"
#include "states.h"
#include "canfestival.h"
#include "sysdep.h"
#include "objdict.h"
#include "eedata.h"
#include "app.h"
#include "stimTask.h"
#include "acceltemp.h"
#include "scheduler.h"


/** 
 * @ingroup nmtslave
 * @brief 
 * Manage the reception of a NMT message from the master.
 * @param *d Pointer to the CAN data structure
 * @param *m Pointer to the message received
 * @return 
 *  -  0 if OK 
 *  - -1 if the slave is not allowed, by its state, to receive the message
 */
void processNMTstateChange(CO_Data* d, Message *m)
{
  
  // check to see if the system can handle external messages
  if( d->nodeState != Hibernate ||
      d->nodeState != BootCheckReset ) 
  {

    MSG_WAR(0x3400, "NMT received. for node :  ", (*m).data[1]);

    // Check if this NMT-message is for this node 
    // byte 1 = 0 : all the nodes are concerned (broadcast)  
    if( ( (*m).data[1] == 0 ) || ( (*m).data[1] == *d->bDeviceNodeId ) )
    {
      switch( (*m).data[0]) /* NMT command specifier (cs) in def.h*/
      { 
        // command interpreter
        
        case NMT_Start_Nodes:
          StartNodesFunc(d, m);
          break;
          
        case NMT_Stop_Nodes:
          StopNodesFunc(d, m);
          break;
          
        case NMT_Enter_Wait_Mode: 
          EnterWaitingFunc(d, m);
          break;
          
        case NMT_Enter_Patient_Operation:
          EnterPatientOperationFunc(d, m);
          break;  
          
        case NMT_Enter_X_Manual:
          EnterXManualFunc(d, m);
          break;
          
        case NMT_Enter_Y_Manual:
          EnterYManualFunc(d, m);
          break;
          
        case NMT_Enter_Stop_Stim:  
          EnterStopStimFunc(d, m);
          break;
          
        case NMT_Enter_Patient_Manual:
          EnterPatientManualFunc(d, m);
          break;
          
        case NMT_Enter_Produce_X_Manual:
          EnterProduceXManualFunc(d, m);
          break;
          
        case NMT_Enter_Record_X:
          EnterRecordXFunc(d, m);
          break;
          
        case NMT_Start_Sync:
          startSYNC(d);
          break;
          
        case NMT_Stop_Sync:
          stopSYNC(d);
          break;
          
        case NMT_Start_PDO:
          PDOInit(d);
          break;
          
        case NMT_Stop_PDO:
          PDOStop(d);
          break;
          
        case NMT_Reset_Watchdog:
          if ( (*m).data[1] == *d->bDeviceNodeId) // must be targeted to this node
              StartWatchDog(d, 10000);
          break;  
          
        case NMT_Reset_Node:
           if(d->NMT_Slave_Node_Reset_Callback != NULL)
              d->NMT_Slave_Node_Reset_Callback(d);
           if ((*m).data[1] == *d->bDeviceNodeId)
              setState(d,Waiting);
          break;
          
        case NMT_Reset_OD_Defaults: 
          if (d->nodeState == Waiting || d->nodeState == Stopped )
           ResetToODDefault();
          break;
          
        case NMT_Reset_Module: 
          if (d->nodeState == Waiting || d->nodeState == Stopped )
           ResetModule();
          break;
          
        case NMT_Clear_CAN_Errors:
          CAN_FormErrors = 0;
          CAN_StuffErrors = 0;
          CAN_BitErrors = 0;
          CAN_OtherErrors = 0;
          CAN_TotalErrors = 0;
          CAN_Receive_BEI = 0;
          CAN_Receive_Messages = 0;
          CAN_Transmit_Messages = 0;
          break;   
          
       case NMT_Erase_Serial_Eprom:
          if (d->nodeState == Waiting) 
            EraseEprom((*m).data[2]);  //param1 specifies all EEPROM(0), RestoreSpace(1) or PatternSpace(2)
          break;

       case NMT_Do_Save_Cmd:   
          if (d->nodeState == Waiting)
            SaveValues();		
          break;
        
       case NMT_Do_Restore_Cmd:  
          if (d->nodeState == Waiting)
            RestoreValues();
          break;
       
        case NMT_Init_NV_Memory: //JML: why? what for
            //if (d->nodeState == Waiting)
          break;
          
        case NMT_Group_On:
          UpdateActivePatterns((*m).data[2] ,1); //param1 specifies function group
          break;
          
        case NMT_Group_Off:
          UpdateActivePatterns((*m).data[2] ,0); //param1 specifies function group
          break;
          
        case NMT_All_Groups_Off:
          ClearAllActivePatterns();
          break;
          
        case NMT_Load_Pattern:  
          //must be in waiting and targeted to this node
          if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting)) 
            TransferPatternEEPROM((*m).data[2], 1); //param1 specifies pattern
          break;
          
        case NMT_Read_Pattern:  
          //must be in waiting and targeted to this node
          if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting))
            TransferPatternEEPROM((*m).data[2], 0); //param1 specifies pattern
          break;
        
        case NMT_Enter_Low_Power:
          //must be in waiting and targeted to this node
          //assume that only way to come out of sleep mode is power cycle
          if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting))
          {
            if((*m).data[2]==1)
            {
              DDRE |= BIT7; //set 3v3 shutoff line as output
              PORTE &= ~BIT7; //set 3v3 shutoff line low to trigger shutoff
            }
            else
            {
              //do other power savings things here before sleeping
              ADCSRA &= ~BIT7; //turn off ADC
              sleepAccelerometer();
              TWCR &= ~BIT2; //turn off I2C
              PORTG &= ~BIT2; //turn off Heartbeat LED
              
              SMCR |= BIT2|BIT0; //Set Sleep Enable in Power-down mode
              asm("SLEEP");
             }
          }
          break;
            
        case NMT_Reset_Comunication:
           if ( ((*m).data[1] == *d->bDeviceNodeId) && (d->nodeState == Waiting))
           {
            UNS8 newNodeId = (*m).data[2]; //Param1
           
              if(d->NMT_Slave_Communications_Reset_Callback != NULL)
                 d->NMT_Slave_Communications_Reset_Callback(d);
      #ifdef CO_ENABLE_LSS
              
              if(newNodeId != d->lss_transfer.nodeID)
                 d->lss_transfer.nodeID = newNodeId;
      #endif
                
              Status_NodeId = newNodeId;
              SaveValues();
              ResetModule();
           }
           break;
           
      case NMT_Enable_ForceAnodeOn:
        ENABLE_ANFON();
        break;
        
      case NMT_Disable_ForceAnodeOn:
        DISABLE_ANFON();
        break;
        
      case NMT_Update_Scheduler:
        InitSchedulerOD();
        break;

      }/* end switch */

    }
  }
}


/** 
 * @ingroup nmtslave
 * @brief Transmit the boot-Up frame when the slave is moving from initialization
 * state to pre_operational state.
 * @param *d Pointer on the CAN data structure
 * @return canSend(bus_id,&m)
 */
UNS8 slaveSendBootUp(CO_Data* d)
{
  Message m;

#ifdef CO_ENABLE_LSS
  if(*d->bDeviceNodeId==0xFF)return 0;
#endif

  MSG_WAR(0x3407, "Send a Boot-Up msg ", 0);

  /* message configuration */
  {
	  UNS16 tmp = NODE_GUARD << 7 | *d->bDeviceNodeId; 
	  m.cob_id = UNS16_LE(tmp);
  }
  m.rtr = NOT_A_REQUEST;
  m.len = 1;
  m.data[0] = 0x00;

  return canSend(d->canHandle,&m);
}


//=====================
//	WATCH DOG ROUTINES
//=====================

static CO_Data *wdogCoData;

/** 
 * @ingroup nmtslave
 * @brief Starts watchdog on slave
 * @param *d Pointer on the CAN data structure
 * @param timebase
 */
void StartWatchDog( CO_Data* d, UNS16 timebase )
{
  	//setAppTimeOut1( timebase );
	
	wdogCoData = d;
  
}

/** 
 * @ingroup nmtslave
 * @brief Stops watchdog on slave
 * @param *d Pointer on the CAN data structure
 */
void StopWatchDog( CO_Data* d )
{
  	//setAppTimeOut1( 0 );
  
  	wdogCoData = d;
  
}

/** 
 * @ingroup nmtslave
 * @brief <BRIEF> setNodeStateToStopped
 */
void setNodeStateToStopped( void )
{
	if( wdogCoData != NULL )
	{
		setState( wdogCoData, Stopped);
	}
	
}



