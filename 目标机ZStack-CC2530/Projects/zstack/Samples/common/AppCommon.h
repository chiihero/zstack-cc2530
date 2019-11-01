#ifndef __APPCOMMON_H__
#define __APPCOMMON_H__

int8 ZXBeeBegin(void);
int8 ZXBeeAdd(char* tag, char* val);
char* ZXBeeEnd(void);

extern int usr_process_command_call(char *ptag, char *pval, char *pout);

#endif