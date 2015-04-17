//Test.cpp - test main to test the driver.

#include "BCM2835.h"
#include <string.h>

int main(void)
{
        int data=0,bytes_read=0, selection=0, w_mode=0,bytes_written=0;
	std::vector<int> Buffer,sendBuffer;
	char test[100];
	
	GPIO gpio;		//GPIO controller object
	I2C  i2c;

	system("clear");

	std::cout << "GPIO Hardware Driver Test v1.0" << std::endl;
	std::cout << "-------------------------------" << std::endl;	
	std::cout << "GPIO and I2C configured - Testing GPIO Output:" << std::endl;
	
	std::cout << "Outputting HIGH - LED on" << std::endl;
	gpio.m_writeGPIO(17,HIGH);
	usleep(500000);
	std::cout << "Outputting LOW - LED off" << std::endl;
	gpio.m_writeGPIO(17,LOW);
	usleep(500000);	
	
	std::cout << "Enabling GPIO I2C Pins on the PI." << std::endl;	
	gpio.m_toggleI2C();		//enable I2C Mode also prevents changes to I2C pins
	
	std::cout << "Setting Slave Address of the Arduino:" << std::endl;	
	i2c.setSlaveAddr(0x2a);		//set the slave address of the device
	
	std::cout << "Baud Set to 90kHz to prevent stretching.." << std::endl;		
	i2c.setBaud(90000); 		//set baud to 90Khz to prevent clock stretching.
	std::cout << "Select Mode: (1) Sensor Read (BYTE) | (2) Read text (DATA) | (3) Send Text (DATA) | (4) Quit" 
												<< std::endl;	
	std::cout << "| (5) readReg8 test | (6) writeReg8 test | (7) readReg16 test" << 
						" | (8) writeReg16 Test" << std::endl;
	std::cin >> selection;

	switch(selection)
	{

		case 1:		w_mode=0;
				i2c.writeByte(w_mode)!=-1 ? std::cout << "SUCCESS - Mode set to send byte on Arduino" << std::endl 
			  	: std::cout << "FAILURE! - TRY AGAIN!" << std::endl;
				usleep(1500);
				for(;;)
				{
					if((data=i2c.readByte())==-1)
					{
						std::cout << "READ FAIL!" << std::endl;
						return(-1);
					}
					else
					{
						switch(data)
						{		
							case 1: std::cout << "\rCLEAR!  " << std::flush;
								break;
							case 0: std::cout << "\rBARRIER!" << std::flush;
								break;
							default:std::cout << "ERROR! " << data << std::endl;
								break;
						}
					}
					usleep(25*1000);
				}
				break;
	
		case 2:		w_mode=10;
				i2c.writeByte(w_mode)!=-1 ? std::cout << "SUCCESS - Mode set to send data on Arduino" << std::endl 
			  	: std::cout << "FAILURE! - TRY AGAIN!" << std::endl;
				bytes_read=i2c.readData(Buffer); //read a number of bytes from the device and return number of 										bytes read.
				for(int i=0; i<bytes_read; i++)
				{	
					std::cout << (char)Buffer[i];
				}
				std::cout << std::endl;
				break;
	
		case 3:		sprintf(test,"Hello Arduino!");
				for(unsigned int i=0; i<strlen(test); i++)
				{
					sendBuffer.push_back(test[i]);
				}
				bytes_written=i2c.writeData(sendBuffer,sendBuffer.size());
				std::cout << "Written " << bytes_written << " bytes to the Arduino:" 
								 << std::endl;
				break;

		case 5:		data=i2c.readReg8(0);
				std::cout << "Read: " << data << " from the Arduino:" << std::endl;
				break;
		case 6:		bytes_written=i2c.writeReg8(0,10);
				std::cout << "Written " << bytes_written << " bytes to the Arduino:" 
								 << std::endl;
				break;
		case 7:		data=i2c.readReg16(20);
				std::cout << "Read: " << data << " from the Arduino:" 
								 << std::endl;
				break;
		case 8:		bytes_written=i2c.writeReg16(0,720);
				std::cout << "Written: " << bytes_written << " to the Arduino:" 
								 << std::endl;
				break;
		default:	break;
	}
	
	
	
	//std::cout << "Writing string to the Arduino:" << TestString << std::endl;

	return(0);
}
