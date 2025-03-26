/**
 * @file eedata.h (common to MES and Stim)
 * @author Jay Hardway, JDC 
 * @brief Header file for eedata.c
*/

#ifndef EEDATA_H
#define EEDATA_H


// -------- DEFINITIONS ----------
#define MAX_BYTES_PER_SUBINDEX  32  //TODO: Not sure what happens if subindex exceeds this 
#define MAX_EEPROM_MEMORY       0x1000 //(4KB)
#define EEPROM_RECORD_SIZE      32
#define EEPROM_ERASE_SIZE       32 //must be divisible into 4096 (4KB)

#define MAX_FLASH_MEMORY        0x020000 //(128KB)
#define FLASH_RECORD_SIZE       32

// --------   DATA   ------------


// -------- PROTOTYPES ----------

UNS8 CheckRestoreFlag(void);
void SaveValues( void );
void RestoreValues( void );
void ResetToODDefault(void);
void ResetModule(void);
void EraseEprom(UNS8);
UNS8 ReadLocalFlashData( UNS32 nvAddress, UNS8 * data, UNS8 numData );
void EEPROM_read(UNS16 address, UNS8 * data, UNS16 length);
void EEPROM_write(UNS16 address, UNS8 * data, UNS16 length);


#endif
 

