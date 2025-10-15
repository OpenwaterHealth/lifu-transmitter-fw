/*
 * i2c_master.c
 *
 *  Created on: Jan 17, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "i2c_protocol.h"
#include "i2c_master.h"
#include "if_commands.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//#define MAX_FOUND_ADDRESSES 5


uint8_t selected_slave = 0xFF;
uint8_t found_address_count = 0;
uint8_t found_addresses[MAX_FOUND_ADDRESSES];

void I2C_scan_local(void)
{
    // Reset the global array and counter
    memset(found_addresses, 0, MAX_FOUND_ADDRESSES );
    found_address_count = 0;

    for (uint8_t address = 0x0; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(LOCAL_I2C_DEVICE, address << 1, 2, 200); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
            found_addresses[found_address_count] = address;
            found_address_count++;
        	// printf("%2x ", address);
        }else{
        	// printf("-- ");
        }
        // if (address > 0 && (address + 1) % 16 == 0) printf("\r\n");
    }

    // printf("\r\n\r\n");
    // fflush(stdout);

}

void I2C_scan_global(void)
{
    // Reset the global array and counter
    memset(found_addresses, 0, MAX_FOUND_ADDRESSES );
    found_address_count = 0;

    for (uint8_t address = 0x0; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(GLOBAL_I2C_DEVICE, address << 1, 2, 200); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
            found_addresses[found_address_count] = address;
            found_address_count++;
        	printf("%2x ", address);
        }else{
        	printf("-- ");
        }
        if (address > 0 && (address + 1) % 16 == 0) printf("\r\n");
    }

    printf("\r\n\r\n");
    fflush(stdout);

}
uint8_t send_buffer_to_slave_local(uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(LOCAL_I2C_DEVICE) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Sending Packet %d Bytes\r\n", buf_len);
    if(HAL_I2C_Master_Transmit(LOCAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), pBuffer, buf_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}


	return 0;
}

uint8_t send_buffer_to_slave_global(uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len)
{
	// Check if the I2C handle is valid
    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(GLOBAL_I2C_DEVICE);
    if (state != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Sending Packet %d Bytes\r\n", buf_len);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(GLOBAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), pBuffer, buf_len, 100);
    if(status != HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        return 2; // timeout
	}


	return 0;
}

uint8_t read_buffer_of_slave_global(uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len)
{
    uint8_t rx_len = max_len + HEADER_SIZE;
    
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(GLOBAL_I2C_DEVICE) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Status Packet %d Bytes\r\n", rx_len);

#if 0

	if(HAL_I2C_Master_Receive(GLOBAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}

#else

    if(HAL_I2C_Mem_Read(GLOBAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), 0x00, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
    }

#endif

	return rx_len;
}

uint8_t read_data_register_of_slave_local(uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(LOCAL_I2C_DEVICE) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Data Packet %d Bytes\r\n", rx_len);

    if(HAL_I2C_Mem_Read(LOCAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), 0x01, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
    }

	return rx_len;
}

uint8_t read_data_register_of_slave_global(uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(GLOBAL_I2C_DEVICE) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Data Packet %d Bytes\r\n", rx_len);

    if(HAL_I2C_Mem_Read(GLOBAL_I2C_DEVICE, (uint16_t)(slave_addr << 1), 0x01, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
    }

	return rx_len;
}

uint16_t I2C_read_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr)
{
	uint16_t reg_value = 0x0000;
    HAL_StatusTypeDef status;

    uint8_t data_to_send[2];
    data_to_send[0] = (uint8_t)(reg_addr >> 8); // Register address high byte
    data_to_send[1] = (uint8_t)(reg_addr & 0xFF); // Register address low byte


    uint8_t data_to_receive[2];

    // Start I2C communication
    status = HAL_I2C_Master_Transmit(LOCAL_I2C_DEVICE, i2c_addr << 1, data_to_send, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    // Receive the data from the CDCE6214 chip
    status = HAL_I2C_Master_Receive(LOCAL_I2C_DEVICE, i2c_addr << 1, data_to_receive, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    reg_value = ((uint16_t)data_to_receive[0] << 8) | data_to_receive[1];

	return reg_value;
}


bool I2C_write_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr, uint16_t reg_val)
{
	bool b_res = true;
    HAL_StatusTypeDef status;
    uint8_t data_to_send[4];

    // Prepare the data to send, including slave address, register address, and data bytes
    data_to_send[0] = (uint8_t)(reg_addr >> 8); // Register address high byte
    data_to_send[1] = (uint8_t)(reg_addr & 0xFF); // Register address low byte
    data_to_send[2] = (uint8_t)(reg_val >> 8); // Data high byte
    data_to_send[3] = (uint8_t)(reg_val & 0xFF); // Data low byte

    // Start I2C communication and send the data
    status = HAL_I2C_Master_Transmit(LOCAL_I2C_DEVICE, i2c_addr << 1, data_to_send, 4, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        b_res = false;
    }

	return b_res;
}

float MAX31875_ReadTemperature(void) {
    uint8_t pointer_byte = 0x00;  // Temperature Register
    uint8_t temp_data[2];
    int16_t raw_temp;
    float temperature;

    // Set pointer register to Temperature Register
    HAL_I2C_Master_Transmit(LOCAL_I2C_DEVICE, MAX31875_ADDRESS << 1, &pointer_byte, 1, HAL_MAX_DELAY);

    // Read temperature data
    HAL_I2C_Master_Receive(LOCAL_I2C_DEVICE, MAX31875_ADDRESS << 1, temp_data, 2, HAL_MAX_DELAY);

    // Convert raw data to temperature
    raw_temp = (temp_data[0] << 8) | temp_data[1];
    temperature = (raw_temp >> 4) * 0.0625;  // For 12-bit resolution

    return temperature;
}
