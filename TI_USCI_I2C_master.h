#if 0

#ifndef USCI_LIB
#define USCI_LIB

#define SDA_PIN 0x40                                  // msp430x261x UCB1SDA pin
#define SCL_PIN 0x80                                  // msp430x261x UCB1SCL pin

void TI_USCI_I2C_receiveinit(unsigned char slave_address, unsigned char prescale);
void TI_USCI_I2C_transmitinit(unsigned char slave_address, unsigned char prescale);


void TI_USCI_I2C_receive(unsigned char byteCount, unsigned char *field);
void TI_USCI_I2C_transmit(unsigned char byteCount, unsigned char *field);


unsigned char TI_USCI_I2C_slave_present(unsigned char slave_address);
unsigned char TI_USCI_I2C_notready();


#endif

#endif