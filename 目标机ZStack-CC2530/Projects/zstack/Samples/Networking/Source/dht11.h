#ifndef __DHT11_H__
#define __DHT11_H__

char dht11_read_bit(void);
unsigned char dht11_read_byte(void);
void dht11_io_init(void);
unsigned char dht11_temp(void);
unsigned char dht11_humidity(void);
void dht11_update(void);

#endif