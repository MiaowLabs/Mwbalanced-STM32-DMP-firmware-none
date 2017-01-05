#ifndef _DATAFLASH_H
#define _DATAFLASH_H


void ProgramFlash(uint32_t addr, char* data, uint16_t len);
void ReadFlash(uint32_t addr, char* data, uint16_t len);
void PIDInit(void);
void PIDWrite(char flag);
void PIDRead(void);
void PIDReset(char flag);


#endif

