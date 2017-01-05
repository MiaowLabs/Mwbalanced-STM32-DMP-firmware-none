#ifndef _COMMON_H
#define _COMMON_H

void GetUniqueID(char* ID);
void SetBlueToothName(void);

unsigned char XOR_Get(char * str, unsigned char  len);
unsigned char XOR_Check(char * str, unsigned char  len,unsigned char checksum);
unsigned char Sum_Get(char *dat,char len);




#endif
