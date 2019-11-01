/*******************************************************************************
*�ļ����ƣ�IIC.c
*�ļ�˵����IIC����ʵ�ֺ���
*�ļ����ߣ�embest
*******************************************************************************/

/*----------------------------------------------------------------------------*/
/*                               includes files	                              */
/*----------------------------------------------------------------------------*/
#include <ioCC2530.h>
#include "IIC.h"
   
/*******************************************************************************
*�������ƣ�IICInit
*����˵����IIC��ʼ��
*�������ߣ�embest
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
  SA0=0;                          //SA0Ϊ0��ʱ��
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


void Stop(void)                   //SCL��״̬��ʱ������Ч�Ŀ���״̬��Ȼ��ͨ��SDA�ķ�ת����ʾ������ֹͣ
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

void Send8bit(unsigned char AData)//���Ͱ�λ�ֽڣ��ȷ��͸�λ���ڴ����λ
{
  unsigned char i=8;
  
  for(i=8; i>0;i--)                //��ϸ�о�
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
//**************8zhuyi SDA �л�**************************/
unsigned char Read8bit(unsigned char ack)        //ack=1��ʾ��Ҫ��MMA����ֹͣλ��Ϊ0���ʾ����Ҫ��MMA����ֹͣλ
{
  unsigned char temp=0;
  unsigned char  i;
  unsigned char label;
  label=ack;
  SDA=1;
  Delay1us();
  for(i=8;i>0;i--)
  {
    //ActiveTime();                 //�����������ͷ����ɾ��
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
  }                               //�ĳ����ģʽ
  return temp;
}


/*********used for cc2530 &MMA7660***********************************/
/*****************************************************************
��MCU��MMA�������ݵ�ʱ��ÿһ�����ݶ���Ҫ���һ��ACK������MMA�������ݵ�ʱ�򣬶���
�������ݣ�ǰ���������Ҫ��MMA����һ��ACK������һ�����ݲ�����MMA����ACK��ֱ��ֹͣ
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
�ظ�������������������豸����豸�ڲ�ͬģʽ֮���л��ĳ���
����ǰ��һ�δ��������������stop�źţ���ת���������״̬
�����Ӳ����Ļ������ͷ����ߣ�һֱ����busy״̬
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
  RepeatStart();                  //��MMA�����ȡ���ݱ���������ַ�ʽ               
  Send8bit(address+1);            //���·����豸��ַλ�����Ҹö�д����Ϊ��ȡ
  if(ChkACK())
    return ERR;
  b=Read8bit(0);                  //����Ӧ���ź�                         
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

