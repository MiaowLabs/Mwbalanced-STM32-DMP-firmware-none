#ifndef _COMMUNICATE_H
#define _COMMUNICATE_H


typedef enum _PIDType{AnglePID, SpeedPID,}PIDType;

extern float sppData1,sppData2;
extern char StatusFlag;

unsigned char XOR_Get(char * str, unsigned char  len);
void Parse(char *dataInput);
void ResponseIMU(void);
void ResponsePID(PIDType type);

void ResponseStatus(void);


#endif

