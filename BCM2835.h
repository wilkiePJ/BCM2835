// BCM2835 low level driver, written by Phil Wilkins April 2015
// Referenced Data sheet and www.peieter-jan.com/node/15
// making into C++ Object Orientated version. 
// Version 1.0 release 1

#ifndef BCM2835_H
#define BCM2835_H

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <bitset>
#include <stdlib.h>
#include <vector>


#define HIGH 1
#define LOW  0
#define HIGH16 0xFFFF0000
#define LOW16 0xFFFF

#define BCM2835_PHYS_BASE_ADDR 0x3F000000      //0x20000000 page 6 data sheet is incorrect for Pi 2 - see left for actual
#define GPIO_PHYSADDR_OFFSET 0x200000	       //offset explained on pg 6 data sheet
#define I2C_PHYSADDR_OFFSET 0x804000           //page 28 - BSC1 is used for I2C 2 for HDMI

#define GPIO_BASE_ADDR (BCM2835_PHYS_BASE_ADDR + GPIO_PHYSADDR_OFFSET) //start point for GPIO registers
#define I2C_BASE_ADDR  (BCM2835_PHYS_BASE_ADDR + I2C_PHYSADDR_OFFSET)  //start point for I2C registers.

//GPIO Register Offsets
enum class GPIOFLAGS {INPUT,OUTPUT,ALT5,ALT4,ALT0,ALT1,ALT2,ALT3};

#define GPIO_FSEL_BANKS 5
#define GPIO_REG_OFFSET 0x4
#define GPIO_SET0 7   	    //write high registers
#define GPIO_SET1 8
#define GPIO_CLR0 10        //write low registers
#define GPIO_CLR1 11
#define GPIO_PLEV0 13       // read registers
#define GPIO_PLEV1 14


//I2C Register Addresses page 28 - all 32 bit in size
#define I2C_CONTROL    0 // control register - used to enable and start reads and writes
#define I2C_STATUS     1 // status register - to view status
#define I2C_DLEN       2 // contols length on write, or how many butes to read depending on S register.
#define I2C_SLAVEADDR  3 // Register to set the slave to talk to.
#define I2C_FIFO       4 // I/O - write to or read from.
#define I2C_DIV        5  // Clock Divider use this to set the baud rate
#define I2C_DEL        6  // Data Delay
#define I2C_CLKT       7  // Clock Stretch Timeout

//I2C Bit definitions
// C Register: 32 bit
#define I2CEN  (1 << 15)       //enable controller - 1bit - 15
#define INTR   (1 << 10)       //iterrupt RX - 1bit - 10
#define INTT   (1 << 9)        //interrupt tx - 1bit - 9
#define INTD   (1 << 8)        //interrupt on done 1bit 8
#define ST     (1 << 7)        //start transfer 1bit 7
#define CLEAR  (1 << 4)        //clear FIFO - 2bits
#define READ   1      	       //R/W bit - 1bit
// S Register - status Bits
#define CLKT (1 << 9) // stretch timeout error
#define ERR  (1 << 8) // Acknowledge error
#define RXF  (1 << 7) // FIFO Full
#define TXE  (1 << 6) // FIFO Empty#
#define RXD  (1 << 5) // FIFO contains data
#define TXD  (1 << 4) // FIFO can accept data
#define RXR  (1 << 3) // FIFO needs to be read
#define TXW  (1 << 2) // FIFO needs to be written to
#define DONE (1 << 1) // Transfer complete
#define TA   1	      // Transmitting	
// execute commands
#define STARTWRITE (I2CEN | CLEAR | ST  )
#define STARTREAD  (I2CEN | CLEAR | ST | READ)
#define CLEAR_STATUS (CLKT | ERR | DONE) 
// core clock for setting baud - data sheet is incorrect correct value below.
#define I2C_CORE_CLOCK 250000000 //250 Mhz
#define POSTSTART_DATA_DELAY 1500
#define SPI_BASE_ADDR

#define ARM_MMU_PFSIZE 4096 //256 32bit entries to a page - sourced from ARM website.


class BCM2835
{

private:
	int m_memHandle; //file handle
	unsigned long m_phys_addr;
	void *m_map;  // memory location descriptor
	
	bool m_openMemHandle();
	bool m_mapMemory();
	bool m_unMapMemory();	

protected:		
	volatile unsigned int *m_VirAddr; //address returned by mmap		

public:
	void configureDevice(unsigned long phys_addr);
	void closeDevice();
	BCM2835() {};
	virtual ~BCM2835() {};	
};

class I2C : public BCM2835
{
private:
	int m_slaveaddr;
        
	bool transComplete();
protected:

public:
	void setBaud(int baud);	
	void setSlaveAddr(int addr);
	int writeByte(const int &data);
	int  readByte();
	int writeData(const std::vector<int> &data, size_t len);
	int readData(std::vector<int> &data);
	int writeReg8(const int Reg, const int data);
	int  readReg8(const int Reg);
	int  writeReg16(const int Reg, const int data);
	int  readReg16(const int Reg);
	void showRegs();
	I2C() 
	{ 
		configureDevice(I2C_BASE_ADDR);
	};
	~I2C() 
	{
		closeDevice();
	};
};

class GPIO : public BCM2835
{
private: 
	bool mI2Cenable;	//bool to prevent access to I2C GPIO pins when I2C toggled on.	
	
	void m_configurePin(const int pin, GPIOFLAGS flags);

public:
	void m_toggleI2C();
	bool toggle_GPIO_PullUp(const int pin,int flag);
	void m_writeGPIO(const int pin,int value);
	int m_readGPIO(const int pin);
	GPIO();
	~GPIO();
};


#endif
