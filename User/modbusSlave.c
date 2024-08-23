/*
 * modbusSlave.c
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#include "modbusSlave.h"


extern uint8_t RXDATA[256];
extern uint8_t TXDATA[256];


void sendData (uint8_t *data, int size)
{
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	UART_Write(UART0, data, size+2);
}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TXDATA[0] = RXDATA[0];       // slave ID
	TXDATA[1] = RXDATA[1]|0x80;  // adding 1 to the MSB of the function code
	TXDATA[2] = exceptioncode;   // Load the Exception code
	sendData(TXDATA, 3);         // send Data... CRC will be calculated in the function
}


uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RXDATA[2]<<8)|RXDATA[3]);  // start Register Address

	uint16_t numRegs = ((RXDATA[4]<<8)|RXDATA[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TXDATA buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TXDATA[0] = SLAVE_ID;  // slave ID
	TXDATA[1] = RXDATA[1];  // function code
	TXDATA[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TXDATA Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TXDATA buffer
	{
		TXDATA[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TXDATA[indx++] = (Holding_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TXDATA, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

