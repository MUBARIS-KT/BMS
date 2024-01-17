/*
 * ic.c
 *
 *  Created on: May 06, 2023
 *      Author: MUBARIS KT
 */

#include "main.h"
#include "stm32f1xx_hal.h" 	 //prefix used in the HAL (Hardware Abstraction Layer) library provided by STMicroelectronics for their STM32F1 series microcontrollers.
#include "ic.h"   			 //Header file of IC BQ76940
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1 ;

float cell_voltage;

int ADC_OFFSET,PACK_VOLTAGE, ADC_GAIN;
int m, p;

uint8_t cell_Balancing[15];
uint8_t i=0,j=0;

//----------------------------------------------------------------------------


/**
 * Initializes the BQ76940 device.
 *
 * This function performs the initialization steps for the BQ76940 IC.
 * It writes a specific value to the system status register to initiate the initialization process.
 */

void BQ76940_Init()
{

	uint8_t buf[2];

	// Set the initialization bit in the system status register
	buf[0] = 1<<5;
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);

}

//----------------------------------------------------------------------------


/**
 * Retrieves voltage data for all cells.
 *
 * This function reads the voltage data for all cells from the BQ76940 device and stores
 * the values in the provided array V_CELL. It calculates the pack voltage based on the
 * voltage readings of individual cells and stores it in PACK_VOLTAGE variable.
 *
 * @param V_CELL An array to store the voltage data of individual cells.
 */

void GET_Cell_DATA(int V_CELL[15])
{

	uint8_t Raw_Data[30];

	// Read the voltage data for all cells
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, VC1_HI, 1, Raw_Data, 30, 1000);

	// Process the raw data and calculate the cell voltages
	for (i=0 ,j=0 ;i<30 ;i=i+2 ,j++ )
	{
		//ADC_GAIN=0.3827;
		 V_CELL[j] = (int)(0.9967665 * (((Raw_Data[i] << 8 | Raw_Data[i + 1])) * (.3827)) + (ADC_OFFSET));
	}
	// Calculate the pack voltage
	m = 0;
	for (p = 0; p <= 14; p++)
	{
	   m = m + V_CELL[p] ;
	}
	PACK_VOLTAGE = m ;

}

//----------------------------------------------------------------------------


/**
 * Retrieves the voltage data of a single cell.
 *
 * This function reads the voltage data of a specific cell from the BQ76940 device and
 * returns the voltage value as a floating-point number.
 *
 * @param cell The cell register address to retrieve the voltage data from.
 * @return The voltage value of the specified cell.
 */

float GET_Single_Cell_DATA(uint16_t cell)
{

	uint8_t buf[2];

	// Read the voltage data of the specified cell
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, cell, 1, buf, 2, 100);
	return (buf[1]<<8 | buf[0]);

}

//----------------------------------------------------------------------------


int Get_Pack_Voltage(void)
{
	uint8_t buf[2];
	int Pack_Voltage;
HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, BAT_HI, 1, buf, 2, 100);
Pack_Voltage = buf[1]<<8 | buf[0];
}


//----------------------------------------------------------------------------


int Get_CC_Reading (void)
{
	uint8_t buf[2],buf1[1],buf2[1];
	int CC_Reading;

	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CC_HI, 1, buf1, 1, 100);
	buf2[0] = buf1[0] | 0x40 ;
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2 , 1, buf2, 1, 100);


	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CC_HI, 1, buf, 1, 100);
	CC_Reading = ((buf[1]<<8 |buf[0])*8.44);//2's complement *8.44 is in datasheet.Dont know the used value is 2's complement or not.

	return CC_Reading;

}

//----------------------------------------------------------------------------

/**
 * Retrieves the status of the BQ76940 device.
 *
 * This function reads the values of the system control register 1 and system status register
 * of the BQ76940 device and returns the status value.
 *
 * @return The status value of the BQ76940 device.
 */

int GET_Status(void)
{

	uint8_t buf[2];

	// Read the values of the system control register 1 and system status register
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL1, 1, buf, 2, 100);

	//ctrl1=buf[0];
	//ctrl2=buf[1];
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);
	return  buf[0];

}

//----------------------------------------------------------------------------


/**
 * @brief Reads ADC calibration data from the BQ76940 device.
 *        Updates global variables ADC_Gain1, ADC_OFFSET, ADC_Gain2, and ADC_GAIN.
 */

void GET_ADC_Calib_DATA()
{

	int ADC_Gain1 =0;
	uint8_t buf[3];

	 // Read ADC Gain 1 value from IC BQ76940 through I2C communication.
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, ADCGAIN1, 1, buf, 1, 100);
	ADC_Gain1=buf[0];

	// Read ADC OFFSET value from BQ76940
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, ADCOFFSET, 1, buf, 1, 100);
	ADC_OFFSET=buf[0];

	int ADC_Gain2 =0;

	// Read ADC Gain 2 value from BQ76940
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, ADCGAIN2, 1, buf, 1, 100);

	ADC_Gain2=buf[0];

	 int temp = ADC_Gain1 & 0xC;  // Extract relevant bits for temp calculation
	 temp=temp>>2;                // Shift the extracted bits to the right position


	 int temp0= ADC_Gain2 & 0xE0;  	// Extract relevant bits for temp0 calculation
	 temp0=temp0>>5;				// Shift the extracted bits to the right position


	 // Calculate ADC_GAIN using extracted bits from ADC_Gain1 and ADC_Gain2
	 ADC_GAIN =365+(temp<<3 | temp0);
}

//----------------------------------------------------------------------------


/**
 * Enables discharge functionality.
 *
 * This function reads the value from the specified address using I2C communication,
 * sets the discharge enable bit, and writes the modified value back to the address.
 * This enables the discharge functionality for the device.
 */

void Enable_Discharge(void)
{

	uint8_t buff[1], buf[3];

	// Read SYS_CTRL2 from the specified address using I2C communication
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buf, 1, 100);

	// Set the discharge enable bit in the read value
	buff[0]=buf[0] | 0x02;

	// Write the modified value back to the specified address using I2C communication
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buff, 1, 100);

}

//----------------------------------------------------------------------------


/**
 * Enables charge functionality.
 *
 * This function reads the value from the specified address using I2C communication,
 * sets the charge enable bit, and writes the modified value back to the address.
 * This enables the charge functionality for the device.
 */

void Enable_Charge(void)
{

	uint8_t buff[1],buf[3];

	// Read SYS_CTRL2 from the specified address using I2C communication
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buf, 1, 100);

	// Set the charge enable bit in the read value
	buff[0]=buf[0] | 0x01;

	// Write the modified value back to the specified address using I2C communication
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buff, 1, 100);

}

//----------------------------------------------------------------------------


/**
 * Disables discharge functionality.
 *
 * This function reads the value from the specified address using I2C communication,
 * clears the discharge enable bit, and writes the modified value back to the address.
 * This disables the discharge functionality for the device.
 */

void Disable_Discharge(void)
{

	uint8_t buff[1],buf[3];

	// Read SYS_CTRL2 from the specified address using I2C communication
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buf, 1, 100);

	// Clear the discharge enable bit in the read value
	buff[0]=buf[0] & 0xFD;

	// Write the modified value back to the specified address using I2C communication
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buff, 1, 100);

}

//----------------------------------------------------------------------------


/**
 * Disables charge functionality.
 *
 * This function reads the value from the specified address using I2C communication,
 * clears the charge enable bit, and writes the modified value back to the address.
 * This disables the charge functionality for the device.
 */

void Disable_Charge(void)
{

	uint8_t buff[1], buf[3];

	// Read SYS_CTRL2 from the specified address using I2C communication
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buf, 1, 100);

	// Clear the charge enable bit in the read value
	buff[0]=buf[0] & 0xFE;

	// Write the modified value back to the specified address using I2C communication
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL2, 1, buff, 1, 100);

}



//----------------------------------------------------------------------------


/**
 * Enables the cell balance FET for a specified cell.
 *
 * This function enables the cell balance FET for the specified cell by setting the corresponding bit
 * in the appropriate register. It also performs adjacent balancing check to ensure that adjacent cells
 * are not being balanced simultaneously.
 *
 * @param cellno The number of the cell for which the cell balance FET should be enabled.
 */

void Enable_Cell_BalanceFET(uint8_t cellno)
{

	uint8_t temp=1, buff[2], buf[2];

	// Set the cell balancing flag for the specified cell
	cell_Balancing[cellno-1]=1;

	// Read CELLBAL1 register using I2C communication
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL1, 1, buf, 1, 100);
	if(!check_adjacent_balancing())
	{
		if(cellno<=5)
		{
			// Enable cell balance FET for cell 1-5
			HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL1, 1, buf, 1, 100);
			temp=temp<<(cellno-1);
			buff[0]=buf[0]|temp;
			HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL1, 1, buff, 1, 100);
		}
		else if ((cellno<=10)&&(cellno>5))
		{
			// Enable cell balance FET for cell 6-10
			HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL2, 1, buf, 1, 100);
			cellno=cellno-6;
			temp=temp<<cellno;
			buff[0]=buf[0]|temp;
			HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL2, 1, buff, 1, 100);
		}
		else
		{
			// Enable cell balance FET for cell 11-15
			HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL3, 1, buf, 1, 100);
			cellno=cellno-11;
			temp=temp<<cellno;

			buff[0]=buf[0]|temp;
			HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL3, 1, buff, 1, 100);
		}
	}
	else
	{
		// Adjacent balancing not allowed, disable cell balance FET for the specified cell
		cell_Balancing[cellno-1]=0;
	}
}

//----------------------------------------------------------------------------


/**
 * Disables the cell balance FET for a specified cell.
 *
 * This function disables the cell balance FET for the specified cell by clearing the corresponding bit
 * in the appropriate register.
 *
 * @param cellno The number of the cell for which the cell balance FET should be disabled.
 */

void Disable_Cell_BalanceFET(uint8_t cellno)
{

	uint8_t temp=1, buf[2], buff[2];


	// Clear the cell balancing flag for the specified cell
	cell_Balancing[cellno-1]=0;

	if(cellno<=5)
	{
        // Disable cell balance FET for cell 1-5
		HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL1, 1, buf, 1, 100);
		temp=temp<<(cellno-1);
		temp=~temp;
		buff[0]=buf[0]&temp;
		HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL1, 1, buff, 1, 100);
	}

	else if ((cellno<=10)&(cellno>5))
	{
		// Disable cell balance FET for cell 6-10
		HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL2, 1, buf, 1, 100);
		cellno=cellno-6;
		temp=~temp;
		buff[0]=buf[0]&temp;
		HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL2, 1, buff, 1, 100);
	}

	else
	{
		 // Disable cell balance FET for cell 11-15
		HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, CELLBAL3, 1, buf, 1, 100);
		cellno=cellno-11;
		temp=~temp;
		buff[0]=buf[0]&temp;
		HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, CELLBAL3, 1, buff, 1, 100);
	}

}

//----------------------------------------------------------------------------


/**
 * Checks if adjacent cells are being balanced.
 *
 * This function iterates through the cell balancing flags for each cell and checks if adjacent cells
 * are both being balanced. If adjacent cells are found to be balanced, it disables the cell balance FET
 * for the first cell and returns 1. If no adjacent balancing cells are found, it returns 0.
 *
 * @return 1 if adjacent cells are being balanced and a cell balance FET is disabled, 0 otherwise.
 */

int check_adjacent_balancing()
{

	for(int i=0;(i<(15-1));i++)
		{
			// Check if adjacent cells are both being balanced
			if((cell_Balancing[i]&&cell_Balancing[i+1])==1)
			{
				// Disable the cell balance FET for the first cell
				Disable_Cell_BalanceFET(i);

				return 1;
			}
		}
		return 0;

}

//----------------------------------------------------------------------------


/**
 * Checks if the device is alive.
 *
 * This function reads the system status register of the device and checks the ALIVE bit to determine
 * if the device is alive. If the ALIVE bit is set, indicating the device is alive, it returns 0. Otherwise,
 * it returns 1, indicating the device is not alive.
 *
 * @return 0 if the device is alive, 1 if the device is not alive.
 */

int IsALIVE()
{

		uint8_t buf[2], temp;

		// Read the system status register to get the ALIVE bit
		HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);
		temp=buf[0]&0x20;
		temp=temp>>5;

		// Check if the ALIVE bit is set
		if(temp)
			return 0;  // Device is alive
		else
			return 1;  // Device is not alive

}

//----------------------------------------------------------------------------


/**
 * Checks for faults in the system.
 *
 * This function reads the system status register of the device and checks for various fault conditions,
 * including Over Current (OCD), Short Circuit Detection (SCD), Over Voltage (OV), and Under Voltage (UV).
 * It returns an integer value representing the type of fault detected:
 *   - 0: No fault
 *   - 1: Over Current Detection (OCD) fault
 *   - 2: Short Circuit Detection (SCD) fault
 *   - 3: Over Voltage (OV) fault
 *   - 4: Under Voltage (UV) fault
 *
 * @return Integer value representing the type of fault detected (0 if no fault).
 */

int Check_Fault()
{

	uint8_t buf[2];
	uint8_t SCD, UV, OV, OCD;

	// Read the system status register to get the fault flags
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);

	// Over Current (OCD) fault detection
	OCD = (buf[0]&0x1);
	if (OCD==1) return 1;   // Over Current (OCD) fault detected

	// Short Circuit Detection (SCD) fault detection
	SCD = (buf[0]&0x2)>>1;
	if (SCD==1) return 2;   // Short Circuit Detection (SCD) fault detected

	// Over Voltage (OV) fault detection
	OV  = (buf[0]&0x4)>>2;
	if (OV==1) return 3;   // Over Voltage (OV) fault detected

	// Under Voltage (UV) fault detection
	UV  = (buf[0]&0x8)>>3;
	if (UV==1) return 4;   // Under Voltage (UV) fault detected

	return 0;   // No fault detected

}

//----------------------------------------------------------------------------


/**
 * Clears all the fault flags in the system.
 *
 * This function reads the system status register of the device, sets all the fault flags to clear,
 * and writes the updated value back to the register.
 * This effectively clears all the fault conditions that might have been previously triggered.
 */

void Clear_ALL_Fault()
{

	uint8_t buf[2];

	// Read the system status register to get the current fault flags
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);

	// Set all the fault flags to clear (1) by ORing with 0xF
	buf[0] = buf[0] | 0xF;

	// Write the updated value back to the system status register
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_STAT, 1, buf, 1, 100);

}

//----------------------------------------------------------------------------


/**
 * Initiates a shutdown sequence for the device.
 *
 * This function performs a series of operations to properly shut down the device.
 * It reads the system control register, sets the shutdown sequence bits,
 * and writes the updated value back to the register to initiate the shutdown.
 * It introduces delays between the steps for proper sequencing.
 */

void Shutdown()
{

	uint8_t buf[1], buff[1];

	// Read the system control register to get the current value
	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, SYS_CTRL1, 1, buf, 1, 100);

	// Set the shutdown sequence bits
	buff[0] = buf[0]|0x0;
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL1, 1, buff, 1, 100);
	HAL_Delay(100);

	buff[0] = buf[0]|0x1;
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL1, 1, buff, 1, 100);
	HAL_Delay(100);

	buff[0] = buf[0]|0x2;
	HAL_I2C_Mem_Write(&hi2c1, BQ76940_ADDRESS, SYS_CTRL1, 1, buff, 1, 100);

}

//----------------------------------------------------------------------------


int GET_Temp_data(int TEMP_DATA[3])
{

	uint8_t buf[6];    // Buffer to store the read data

	HAL_I2C_Mem_Read(&hi2c1, BQ76940_ADDRESS, TS1_HI , 1, buf, 6, 1000);

	for (i=0 ,j=0 ;i<6 ;i=i+2 ,j++ )
	{
		 TEMP_DATA[j] = buf[i] << 8 | buf[i + 1];
	}
}


//----------------------------------------------------------------------------


int UVP_ON (int cell_voltage)
{
	int min_vol;
	if (cell_voltage <  min_vol)
	  Enable_Charge();
}


//----------------------------------------------------------------------------


/*initialise function for over voltage protection
 *
*/
int OVP_ON (int voltage_mv)
{
	int max_vol;
	if (cell_voltage > max_vol)
		Disable_Charge;

}

//----------------------------------------------------------------------------


