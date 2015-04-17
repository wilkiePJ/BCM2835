//BCM2835 Driver routine - I2C and GPIO functionality
//uses the registers directly and bypasses the drivers to address issues with comms with Arduino.
//Written by Phil Wilkins April 2015
//Version 1.0 - release 1

#include "BCM2835.h"

void BCM2835::configureDevice(unsigned long phys_addr)
{
	m_phys_addr=phys_addr;
	if(!m_openMemHandle())
		std::cout << "BCM2835: Error Opening /dev/mem." << std::endl;
	if(!m_mapMemory())
		std::cout << "BCM2835: Error mapping memory" << std::endl;
}

void BCM2835::closeDevice()	
{
	if(!m_unMapMemory())
		std::cout << "BCM2835: Error unmapping memory" << std::endl;
	close(m_memHandle);
}

bool BCM2835::m_openMemHandle()
{
		
	if((m_memHandle=open("/dev/mem", O_RDWR |O_SYNC))<0)
	{
		return(false);
	}
        else return(true);	
	
}

bool BCM2835::m_mapMemory()
{

	m_map=mmap(NULL,sysconf(_SC_PAGESIZE),PROT_READ | PROT_WRITE, MAP_SHARED,m_memHandle,m_phys_addr);
	if(m_map == MAP_FAILED)
	{
		perror("mmap");
		return(false);
	}
	else m_VirAddr=(volatile unsigned int *)m_map; 
	return(true);
}

bool BCM2835::m_unMapMemory()
{
	
	if((munmap(m_map,sysconf(_SC_PAGESIZE)))<0)
	{
		std::cout << "Err:" << errno << std::endl;
		return(false);
	}
	else return(true);
}

GPIO::GPIO()
{
	mI2Cenable=false;
	configureDevice(GPIO_BASE_ADDR); //passes the address of the pointer
}

GPIO::~GPIO()
{
	closeDevice();
}


void GPIO::m_configurePin(const int pin,GPIOFLAGS flags)
{
	int GPFSELBank=0, GPFSELPin=0;	

	GPFSELBank=pin/10;		     //which pin register am i asking for
	GPFSELPin=pin%10;	             // which pin inside the register am I asking for

	//set the address to the appropriate select register	
	switch(flags)
	{
		case GPIOFLAGS::INPUT: *(m_VirAddr+GPFSELBank) &= ~(7 << (GPFSELPin*3));  
					break;
		case GPIOFLAGS::OUTPUT: *(m_VirAddr+GPFSELBank) &= ~(7 << (GPFSELPin*3)); // input before output.
					*(m_VirAddr+GPFSELBank) |= (1 << (GPFSELPin*3));
					break;
		default:	        //called when any of the ALT GPIOFLAGS are asked for.
					*(m_VirAddr+GPFSELBank) &= ~(7 << (GPFSELPin*3));		
					*(m_VirAddr+GPFSELBank) |= ((int)flags << (GPFSELPin*3));
					break;
	}

	return;
}

void GPIO::m_toggleI2C()
{
	if(mI2Cenable)
	{
	
		m_configurePin(2,GPIOFLAGS::INPUT);
		m_configurePin(3,GPIOFLAGS::INPUT);
		mI2Cenable=false;
	}
	else
	{
		m_configurePin(2,GPIOFLAGS::ALT0);
		m_configurePin(3,GPIOFLAGS::ALT0);
		mI2Cenable=true;
	}	
	return;
}

bool GPIO::toggle_GPIO_PullUp(const int pin,int flag)
{
	return(true);
}

// FUNCTION - sets the GPIO PIN high or low.
void GPIO::m_writeGPIO(const int pin,int value)
{
	m_configurePin(pin,GPIOFLAGS::OUTPUT);
	switch(value)
	{
		case HIGH: if(pin>32)
			   {
				*(m_VirAddr+GPIO_SET1)=(1 <<(pin-32));
			   }
			   else
			   {
				*(m_VirAddr+GPIO_SET0)=(1 << pin);
			   }				
			   break;
		case LOW: if(pin>32)
			   {
				*(m_VirAddr+GPIO_CLR1)=(1 <<(pin-32));
			   }
			   else
			   {
				*(m_VirAddr+GPIO_CLR0)=(1 << pin);
			   }				
			   break;
	}
	return;

}

int GPIO::m_readGPIO(const int pin)
{
	int value=0;

	m_configurePin(pin,GPIOFLAGS::INPUT);
	if(pin>32)
	{
		value=(int)*(m_VirAddr+GPIO_PLEV1);
		value &= (1 << (pin-32));
	}
        else
	{
		value=(int)*(m_VirAddr+GPIO_PLEV0);
		value &= (1 << pin);
	}				
	return(value);
}

bool I2C::transComplete()
{
	int status=-1,i=0;
		
	for(i=0; i<10; i++)
	{
		status=*(m_VirAddr+I2C_STATUS) & DONE;			
		if(status==DONE) break;	
		usleep(1000);
	}

	if(i>50)
	 	return(false); 
			else return(true);
}

void I2C::setSlaveAddr(int addr)
{
	m_slaveaddr=addr;
	return;
}

void I2C::setBaud(int baud)
{
	*(m_VirAddr+I2C_DIV)=(I2C_CORE_CLOCK/baud);
	return;
}

//FUNCTION: Write one byte of data to the I2C device
//returns -1 if fails otherwise number of bytes that have been written
int I2C::writeByte(const int &data)
{
	int bytesWritten=0;

	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr;  //set  the A register to set the slave address  	
	*(m_VirAddr+I2C_DLEN)=1;	  	 //set length to 1 byte
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;    //clear status registers	
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;     //send start condition 		
	*(m_VirAddr+I2C_FIFO)=data;		 //put data into the FIFO register 
 	//Allow to finish
	if(transComplete())
	{
		bytesWritten=*(m_VirAddr+I2C_DLEN) & 0xFFFF;		
		 return(bytesWritten); 
	} 
	else return(-1);
}

//FUNCTION: Read one byte of data from the I2C device
//returns -1 if fails otherwise the data read.
int  I2C::readByte()
{
	int data=0,bytes_read=0;

	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr;   //set slave address		
	*(m_VirAddr+I2C_DLEN)=1; 		  //set read buffer to one byte 	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	  //clear the status registers	
	*(m_VirAddr+I2C_CONTROL)=STARTREAD;	  //send start condition
	do{ 
		usleep(10); 
	}while((*(m_VirAddr+I2C_STATUS) & TA)==1);
	do{				
			usleep(10);
			data=(*(m_VirAddr+I2C_FIFO));
			bytes_read++;
	}while(((*(m_VirAddr+I2C_STATUS) & RXD)==32) && ((*(m_VirAddr+I2C_STATUS) & RXR)==8));
	if(bytes_read==0) return(-1); else return(data);
}

//FUNCTION: showRegs - returns the contents of the status register.
void I2C::showRegs()
{
	//display registers - this is for debugging and tuning of the interface.
	std::cout << "S Register" << std::endl;
	std::cout << "CLKT: " << (*(m_VirAddr+I2C_STATUS) & CLKT);
	std::cout << " ERR:" << (*(m_VirAddr+I2C_STATUS) & ERR);
	std::cout << " RXF:" << (*(m_VirAddr+I2C_STATUS) & RXF);
	std::cout << " TXE:" << (*(m_VirAddr+I2C_STATUS) & TXE);
	std::cout << " RXD:" << (*(m_VirAddr+I2C_STATUS) & RXD);
	std::cout << " TXD:" << (*(m_VirAddr+I2C_STATUS) & TXD);
	std::cout << " RXR:" << (*(m_VirAddr+I2C_STATUS) & RXR);
	std::cout << " TXW:" << (*(m_VirAddr+I2C_STATUS) & TXW);
	std::cout << " DONE:" << (*(m_VirAddr+I2C_STATUS) & DONE); 
	std::cout << " TA:" << (*(m_VirAddr+I2C_STATUS) & TA) << std::endl;
	
	//UNCOMMENT OUT BELOW TO SEE CDIV BD DEL TIMING REGISTER VALUES	
	//std::cout << "CDIV Register" << std::endl;
	//std::cout << "CDIV:" << (*(m_VirAddr+I2C_DIV)) << std::endl;
	//std::cout << "Del Register" << std::endl;
	//std::cout << "FEDL" << (*(m_VirAddr+I2C_DEL) & HIGH16) << std::endl;
	//std::cout << "REDL:" << (*(m_VirAddr+I2C_DEL) & LOW16) << std::endl;

}

//FUNCTION: writeData - write multiple bytes of data to I2C device.
// best used with the Arduino returns -1 on error otherwise the number of bytes written.
int I2C::writeData(const std::vector<int> &data, size_t len)
{
	unsigned int bytesWritten=0;

	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr; //set  the A register to set the slave address 	
	*(m_VirAddr+I2C_DLEN)=len;		//set length to specified size
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;   //clear status registers
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;	//send start condition 	
	usleep(POSTSTART_DATA_DELAY);
	while((bytesWritten!=len))	
		{	
			usleep(10);
			//put data into the FIFO register 			
			*(m_VirAddr+I2C_FIFO)=data[bytesWritten];
			bytesWritten++;
		}
	if(bytesWritten==0) return(-1); else return(bytesWritten);
}

//FUNCTION: readData - reads multiple bytes of data from an I2C device best used with Arduino.
//requires a vector input to recieve the data. Returns the number of bytes that have been read.
int I2C::readData(std::vector<int> &data)
{
	int bytesRead=0;

	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr; //set slave address	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	//clear the status registers
	*(m_VirAddr+I2C_DLEN)=16; 		//set read buffer to the max size of the FIFO	
	*(m_VirAddr+I2C_CONTROL)=STARTREAD;	//send start condition 
	do{ 
		usleep(10); 
	}while((*(m_VirAddr+I2C_STATUS) & TA)==1);				//required pause for timing purposes.
	while(((*(m_VirAddr+I2C_STATUS) & RXD)==32) && ((*(m_VirAddr+I2C_STATUS) & RXR)==8))	
		{	
			usleep(10);
			data.push_back(*(m_VirAddr+I2C_FIFO));
			bytesRead++;
		}
	
	if(bytesRead==0) return(-1); else return(bytesRead);
}

//FUNCTION: writeReg8 - writes an 8 bit value to a specific register on an I2C device
//returns -1 on failure otherwise 1
int I2C::writeReg8(const int Reg, const int data)
{
	int bytesWritten=0;
	
	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr;  //set  the A register to set the slave address
	*(m_VirAddr+I2C_DLEN)=1;		 //set length to 1 byte	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	 //clear status register	
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;	 //send start condition  	
	*(m_VirAddr+I2C_FIFO)=Reg;		 //send register requested
	usleep(POSTSTART_DATA_DELAY);
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;     //now send data to be writte 	
	*(m_VirAddr+I2C_FIFO)=data;		 //put data into the FIFO register
	if(transComplete())
	{
		bytesWritten=*(m_VirAddr+I2C_DLEN) & 0xFFFF;		
		 return(bytesWritten); 
	} 
	else return(-1);

}

//FUNCTION: readReg8- reads an 8 bit value from a specific register from an I2C device.
//returns -1 on failure otherwise the data read.
int  I2C::readReg8(const int Reg)
{
	int data=0,bytes_read=0;
	
	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr; //set  the A register to set the slave address
	*(m_VirAddr+I2C_DLEN)=1;		//set length to 1 byte	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	//clear status registers	
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE; 	//send start condition 
	*(m_VirAddr+I2C_FIFO)=Reg; 		//put data into the FIFO register 
	usleep(POSTSTART_DATA_DELAY);
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	
	*(m_VirAddr+I2C_CONTROL)=(I2CEN | ST | READ);
	do{ 
		usleep(10); 
	}while((*(m_VirAddr+I2C_STATUS) & TA)==1);
	do{	
			usleep(10);
			data=(*(m_VirAddr+I2C_FIFO));
			bytes_read++;
	}while(((*(m_VirAddr+I2C_STATUS) & RXD)==32) && ((*(m_VirAddr+I2C_STATUS) & RXR)==8));
	if(bytes_read==0) return(-1); else return(data);
}

//FUNCTION: writeReg16 - writes a 16 bit value to a specific register on an I2C device
//returns -1 on failure otherwise 2
int  I2C::writeReg16(const int Reg, const int data)
{
	unsigned int bytesWritten=0;

	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr;  //set  the A register to set the slave address
	*(m_VirAddr+I2C_DLEN)=1;		 //set length to 1 byte	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	 //clear status register	
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;	 //send start condition  	
	*(m_VirAddr+I2C_FIFO)=Reg;		 //send register requested
	usleep(POSTSTART_DATA_DELAY);
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	
	*(m_VirAddr+I2C_DLEN)=2;
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE;     //now send data to be write 	
	while((bytesWritten!=2))	
		{	
			usleep(10);
			//put data into the FIFO register 			
			switch(bytesWritten)
			{			
				case 0: *(m_VirAddr+I2C_FIFO)=(data & 0xFF); break;
				case 1: *(m_VirAddr+I2C_FIFO)=(data >> 8); break;
			}			
			bytesWritten++;
		}
	if(bytesWritten==0) return(-1); else return(bytesWritten);	
	return(0);
}

//FUNCTION: readReg16 - reads a 16 bit value to a specific register on an I2C device
//returns -1 on failure otherwise returns the value after converting it to a signed number.
int  I2C::readReg16(const int Reg)
{
	int dataBuff[2],bytes_read=0;
	
	*(m_VirAddr+I2C_SLAVEADDR)=m_slaveaddr; //set  the A register to set the slave address
	*(m_VirAddr+I2C_DLEN)=1;		//set length to 1 byte	
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	//clear status registers	
	*(m_VirAddr+I2C_CONTROL)=STARTWRITE; 	//send start condition 
	*(m_VirAddr+I2C_FIFO)=Reg
; 		//put data into the FIFO register 
	usleep(POSTSTART_DATA_DELAY);		// wait for device to settle.
	*(m_VirAddr+I2C_STATUS)=CLEAR_STATUS;	
 	*(m_VirAddr+I2C_DLEN)=2;	
	*(m_VirAddr+I2C_CONTROL)=STARTREAD;
	do{ 						//wait until finished transmitting
		usleep(10); 
	}while((*(m_VirAddr+I2C_STATUS) & TA)==1);

	while(((*(m_VirAddr+I2C_STATUS) & RXD)==32))
	{						
			usleep(10);
			dataBuff[bytes_read]=(*(m_VirAddr+I2C_FIFO) & 0xFF);
			bytes_read++;
	}
	
	if(bytes_read==0) return(-1); else
	{ 
		int output=0;
		//check the output and convert it to signed number.
		output=((dataBuff[0]) | (dataBuff[1] << 8));	
		if(output > 0x7FFF)
			return((0xFFFF << 16) | ((dataBuff[0]) | (dataBuff[1] << 8)));
				else
				 return(((dataBuff[0]) | (dataBuff[1] << 8)));	
	}
}

