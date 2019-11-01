#include <ioCC2530.h>
#include <stdio.h>
#include "dht11.h"

#define       PIN_CFG     (P0SEL  &= ~0x20)

#define       PIN_OUT       (P0DIR |= 0x20)
#define       PIN_IN        (P0DIR &= ~0x20)
#define       PIN_CLR       (P0_5 = 0)
#define       PIN_SET       (P0_5 = 1)
#define       PIN_R         (P0_5)

#define       COM_IN          PIN_IN  
#define       COM_OUT         PIN_OUT
#define       COM_CLR         PIN_CLR
#define       COM_SET         PIN_SET
#define       COM_R           PIN_R

static void DelaymS (unsigned int dly);

unsigned char sTemp;
unsigned char sHumidity;

#pragma optimize=none      
static void DelaymS (unsigned int dly)                   
{
    unsigned int i;
    for ( ; dly>0; dly--)
    {
            for (i=0; i<1032; i++);        
    }
}


#pragma optimize=none
char dht11_read_bit(void)
{
  int i = 0;
  
  while (!COM_R);
  for (i=0; i<200; i++) {
    if (COM_R == 0) break;
  }
  if (i<30)return 0;  //30us
  return 1;
}

#pragma optimize=none
unsigned char dht11_read_byte(void)
{
  unsigned char v = 0, b;
  int i;
  for (i=7; i>=0; i--) {
    b = dht11_read_bit();
    v |= b<<i;
  }
  return v; 
}



void dht11_io_init(void)
{
  P0SEL  &= ~0x20;          //P1Ϊ��ͨ I/O ��
  COM_OUT;
  COM_SET;  
}
unsigned char dht11_temp(void)
{
  return sTemp;
}
unsigned char dht11_humidity(void)
{
  return sHumidity;
}

void dht11_update(void)
{
  int flag = 1;
  unsigned char dat1, dat2, dat3, dat4, dat5, ck;
  
  //��������18ms 
  COM_CLR;
  DelaymS(18);
  COM_SET;
  
  flag = 0;
  while (COM_R && ++flag);
  if (flag == 0) return;
  
  //������������������ ������ʱ20us
  //������Ϊ���� �жϴӻ���Ӧ�ź�  
  //�жϴӻ��Ƿ��е͵�ƽ��Ӧ�ź� �粻��Ӧ����������Ӧ����������	  	    
  flag = 0;
  while (!COM_R && ++flag);
  if (flag == 0) return;
  flag = 0;
  while (COM_R && ++flag);
  if (flag == 0) return;
  
  
  dat1 = dht11_read_byte();
  
  dat2 = dht11_read_byte();
  
  dat3 = dht11_read_byte();
   
  dat4 = dht11_read_byte();  
  
  dat5 = dht11_read_byte();            
  
  ck = dat1 + dat2 + dat3 + dat4;
  
  if (ck == dat5) {
    sTemp = dat3;
    sHumidity = dat1;        
  }
  
//  printf("ʪ��: %u.%u%% �¶�: %u.%u�� \r\n", dat1,dat2, dat3,dat4);   
}
