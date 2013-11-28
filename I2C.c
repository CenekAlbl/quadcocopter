/*
 * I2C.c
 *
 *  Created on: 22.5.2013
 *      Author: Vincek
 */

#include "includes.h"
#include "I2C.h"


void setup_I2C()
{

                    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
                    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

                    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
                    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

					SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

					I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), 1); // 1 : 400Khz, 0 : 100Khz
}

int write_to_I2C(uint8_t device, uint8_t address, uint8_t data){

	// Wait until master module is done transferring.
	while(I2CMasterBusy(I2C_PORT))
	{
	};


	//which device to address, 0 for send, 1 for receive
	I2CMasterSlaveAddrSet(I2C_PORT, device, 0);

	//what data to write
	I2CMasterDataPut(I2C_PORT, address);

	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait until master module is done transferring.
	while(I2CMasterBusy(I2C_PORT)){};

	// Check for errors.
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 1;
	}


	I2CMasterDataPut(I2C_PORT, data);


	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_FINISH);


	while(I2CMasterBusy(I2C_PORT))
	{
	};


	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 1;
	}

	return 0;
}

int read_from_I2C(uint8_t device, uint8_t address, uint8_t * buffer, int num_bytes){
	uint8_t bytes_cnt;
	uint16_t MasterOptionCommand;
	// Wait until master module is done transferring.
	while(I2CMasterBusy(I2C_PORT))
	{
	};
	// which device to address, , 0 for send, 1 for receive
	I2CMasterSlaveAddrSet(I2C_PORT, device, 0);

	I2CMasterDataPut(I2C_PORT, address);

	// Initiate send of data from the master.
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);

	while(I2CMasterBusy(I2C_PORT))
	{
	};


	int err = I2CMasterErr(I2C_PORT);

	if(err != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}

	I2CMasterSlaveAddrSet(I2C_PORT, device, 1);

	MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_START;

	for(bytes_cnt = 0; bytes_cnt < num_bytes; bytes_cnt++)
	{
		// The second and intermittent byte has to be read with CONTINUE control word
		if(bytes_cnt == 1)
				MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

		// The last byte has to be send with FINISH control word
		if(bytes_cnt == num_bytes - 1)
				MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;

		// Re-configure to SINGLE if there is only one byte to read
		if(num_bytes == 1)
				MasterOptionCommand = I2C_MASTER_CMD_SINGLE_RECEIVE;

		// Initiate read of data from the slave.
		I2CMasterControl(I2C_PORT, MasterOptionCommand);

		// Wait until master module is done reading.
		while(I2CMasterBusy(I2C_PORT))
		{
		};


		// Check for errors.
		int err = I2CMasterErr(I2C_PORT);

			if(err != I2C_MASTER_ERR_NONE)
			{
				return 0;
			}

		// Move byte from register
			uint8_t temp = I2CMasterDataGet(I2C_PORT);
				buffer[bytes_cnt] = temp;
    }

	return bytes_cnt;

}




