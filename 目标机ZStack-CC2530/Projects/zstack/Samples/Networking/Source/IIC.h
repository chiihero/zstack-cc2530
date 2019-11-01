/*----------------------------------------------------------------------------*/
/*                            macro declaration 	                      */
/*----------------------------------------------------------------------------*/
#ifndef _IIC_H
#define _IIC_H

#define SENSOR_REV02

#ifdef SENSOR_REV01
#define SDABIT  (1<<7)
#define SCLBIT  (1<<2)
#define SA0BIT  (1<<5)
#define SDA  P0_7
#define SCL  P1_2
#define SA0  P0_5
#endif


#ifdef SENSOR_REV02
#define SDABIT  (1<<1)
#define SCLBIT  (1<<6)
#define SA0BIT  (1<<5)
#define SDA  P0_1
#define SCL  P0_6
#define SA0  P0_5
#endif

#define ActiveTime()     (Delay5us())


//数据类型定义
//typedef unsigned char	uint8;	
//typedef signed char		int8;
//typedef unsigned short	uint16;
//typedef signed short	int16;
//typedef unsigned long	uint32;
//typedef signed long		int32;

/*----------------------------------------------------------------------------*/
/*                                  macro label 	                      */
/*----------------------------------------------------------------------------*/
/**********macro label***********************/
#define ERR      (0x01)
#define SUCCESS  (0x00)
void RepeatStart(void);
void SendNoAck(void);

void Delay5us(void);
void Delay1us(void);
void IICInit(void);
void Start(void);
void RepeatStart();
void Stop(void);          //SCL高状态的时候，是有效的控制状态，然后通过SDA的翻转来表示启动和停止
unsigned char ChkACK(void);
void Send8bit(unsigned char Data);
unsigned char RegWrite(unsigned char address, unsigned char reg,unsigned char val);
unsigned char RegWriteN(unsigned char address, unsigned char reg1,unsigned char N,unsigned char *array);
void SendAck(void);
unsigned char Read8bit(unsigned char ack);
unsigned char RegRead(unsigned char address, unsigned char reg);
unsigned char RegReadN(unsigned char address, unsigned char reg1,unsigned char N);
#endif