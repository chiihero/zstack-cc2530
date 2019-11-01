#include <ioCC2530.h>
#include "ys-srf05.h"

#define   PIN_EN_CFG      (P0SEL &= ~0x20, P0DIR |= 0x20)
#define   PIN_ECHO_CFG    (P0SEL &= ~0x02, P0DIR &=~0x02)
#define   WAVE_EN_PIN     P0_5
#define   WAVE_INPUT_PIN  P0_1

static char clkidx;
static unsigned int clk;
static unsigned clks[] = {32000, 16000, 8000, 4000, 2000, 1000, 500, 250};


#pragma optimize=none
static void Delay_10us(void)
{ 
      {
        char i = 10;
        i--;
        i--;
        i--;
        i--;
        i--;
        i--;
       }
}

void srf05Init(void)
{
  PIN_EN_CFG;
  PIN_ECHO_CFG;
  
  WAVE_EN_PIN = 0;
  
  clkidx = (CLKCONCMD>>3)&0x07;
  
  clk = (clks[clkidx]/128);
}

static void srf05Start(void)
{
  WAVE_EN_PIN = 1;
  Delay_10us();
  Delay_10us();
  WAVE_EN_PIN = 0;
}

#pragma optimize=none
unsigned int srf05Distance(void)
{
  int i = 0;
  float cnt = 0;
  unsigned int d;  
  
  T1CNTL = 0x00;
  T1CNTH = 0x00;
  srf05Start();
  while ((0 == WAVE_INPUT_PIN) && ++i);
  if (i == 0) return -1;
  T1CTL = 0x0D; //128div

  i = 0;
  while (WAVE_INPUT_PIN && ++i);
  T1CTL = 0x00;
  if (i == 0) return -1;
  
  cnt = (T1CNTH<<8) | (T1CNTL);
  d = (int)((cnt) / clk * 17);
  
  return d;
}

