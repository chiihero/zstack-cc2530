/*******************************************************************************
*文件名称：IIC.c
*文件说明：IIC功能实现函数
*文件作者：embest
*******************************************************************************/

/*----------------------------------------------------------------------------*/
/*                               includes files	                              */
/*----------------------------------------------------------------------------*/
#include <ioCC2530.h>
#include "IIC.h"
   
/*******************************************************************************
*函数名称：IICInit
*函数说明：IIC初始化
*函数作者：embest
*******************************************************************************/
void IICInit(void)
{
#ifdef SENSOR_REV01 
  P0SEL &=~ (SDABIT);
  P1SEL &=~ (SCLBIT);
  P0SEL &=~ (SA0BIT);
  P0DIR |= SDABIT;
  P1DIR |= SCLBIT;
  P0DIR |= SA0BIT;
  SA0=0;                          //SA0为0，时候
#endif
#ifdef SENSOR_REV02 
  P0SEL &=~ (SDABIT);
  P0SEL &=~ (SCLBIT);
  P0SEL &=~ (SA0BIT);
  P0DIR |= SDABIT;
  P0DIR |= SCLBIT;
  P0DIR |= SA0BIT;
  SA0=0; 
#endif
}

void Start(void)
{
  SDA=1;
  Delay1us();
  SCL=1;
  ActiveTime();                   //delay about 5us
  SDA=0;
  ActiveTime(); 
  SCL=0;
  ActiveTime(); 
}

void RepeatStart(void)                //without stop signal before
{
  SDA=1;
  Delay1us();
  SCL=1;
  ActiveTime();                   //delay about 5us
  SDA=0;
  ActiveTime(); 
  SCL=0;
  ActiveTime(); 
} 


void Stop(void)                   //SCL高状态的时候，是有效的控制状态，然后通过SDA的翻转来表示启动和停止
{
  SDA=0;
  Delay1us();
  SCL=1;
  ActiveTime();  
  SDA=1;
  ActiveTime(); 
  SCL=0;
  ActiveTime(); 
}

unsigned char ChkACK(void)        
{
  SDA=1;
  ActiveTime();
  SCL=1;
  ActiveTime();
  if(!SDA)                  
  {
    SCL=0;
    ActiveTime();
    return 0;
  }else{
    SCL=0;
    ActiveTime();
    return 1;
  }
}

void SendAck(void)
{ 
  SDA=0;      
  Delay1us();
  SCL=1;
  ActiveTime();
  SCL=0;
  ActiveTime();
}


void SendNoAck(void)
{
  SCL=0;
  ActiveTime();
  SDA=1;
  Delay1us();
  SCL=1;
  ActiveTime();
}

void Send8bit(unsigned char AData)//发送八位字节，先发送高位，在传输低位
{
  unsigned char i=8;
  
  for(i=8; i>0;i--)                //详细研究
  {
    if(AData&0x80)
      SDA=1;
    else
      SDA=0;
    Delay1us();
    SCL=1;
    ActiveTime();
    SCL=0;
    AData<<=1; 
    ActiveTime();                   //mannual
  }
}
//**************8zhuyi SDA 切换**************************/
unsigned char Read8bit(unsigned char ack)        //ack=1表示需要向MMA传输停止位，为0则表示不需要向MMA传输停止位
{
  unsigned char temp=0;
  unsigned char  i;
  unsigned char label;
  label=ack;
  SDA=1;
  Delay1us();
  for(i=8;i>0;i--)
  {
    //ActiveTime();                 //保险起见，回头可以删掉
    SCL=1;
    ActiveTime();
    if(SDA)
      temp+=0x01;
    else
      temp&=0xfe;
    SCL=0;
    ActiveTime();
    if(i>1)
      temp<<=1;
  }
  if(label)
  { 
    SendAck();
  }
  else
  {
    SendNoAck();
  }                               //改成输出模式
  return temp;
}


/*********used for cc2530 &MMA7660***********************************/
/*****************************************************************
当MCU向MMA传输数据的时候，每一个数据都需要获得一个ACK，而从MMA接收数据的时候，对于
连续数据，前面的数据需要向MMA传输一个ACK，最后的一个数据不用向MMA传输ACK，直接停止
**********************************************************************/
unsigned char RegWrite(unsigned char address, unsigned char reg,unsigned char val)
{
  Start();                        //Send Start
  Send8bit(address);              //Send IIC "Write" Address
  if(ChkACK())
    return ERR;
  Send8bit(reg);                  // Send Register
  if(ChkACK())
    return ERR;
  Send8bit(val);                  // Send Value
  if(ChkACK())
    return ERR;
  Stop();                         // Send Stop
  return SUCCESS;
}

/***********************************************************
重复启动情况：常用语主设备与从设备在不同模式之间切换的场合
比如前面一次传输结束过后，无需stop信号，而转而进入接收状态
这样子操作的话，不释放总线，一直处于busy状态
**************************************************************/

unsigned char RegRead(unsigned char address, unsigned char reg)             
{
  unsigned char  b;
  Start();                              
  Send8bit(address);    
  if(ChkACK())
		return ERR;
  Send8bit(reg);        
  if(ChkACK())
    return ERR;
  RepeatStart();                  //从MMA里面读取数据必须采用这种方式               
  Send8bit(address+1);            //重新发送设备地址位，并且该读写属性为读取
  if(ChkACK())
    return ERR;
  b=Read8bit(0);                  //不带应答信号                         
  Stop();                            
    return b;
}

void Delay5us(void)
{ 
      {
        char i = 10;
        i--;
        i--;
        i--;
       }
 
}

void Delay1us(void)              
{ 
      {
        char i = 10;
        i--;
        i--;
       }
}

