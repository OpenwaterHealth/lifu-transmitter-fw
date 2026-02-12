/*
 * if_commands.c
 *
 *  Created on: Nov 15, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "module_manager.h"
#include "i2c_master.h"
#include "i2c_slave.h"
#include "i2c_protocol.h"
#include "trigger.h"
#include "tx7332.h"
#include "demo.h"
#include "thermistor.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

extern bool _enter_dfu;

extern TX7332 transmitters[2];
extern bool async_enabled;

static uint32_t id_words[3] = {0};
static char retTriggerJson[0xFF];

uint8_t send_buff[I2C_BUFFER_SIZE] = {0};
uint8_t receive_buffer[I2C_BUFFER_SIZE] = {0};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

static void process_i2c_read_buffer(UartPacket *uartResp, UartPacket* cmd, uint8_t module_id);
static void process_i2c_forward(UartPacket *uartResp, UartPacket* cmd, uint8_t module_id);

static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}


static void process_i2c_read_buffer(UartPacket *uartResp, UartPacket* cmd, uint8_t module_id)
{
	uint16_t rx_len = 0;
	uint8_t slave_addr = ModuleManager_GetModule(module_id)->i2c_address;
	I2C_TX_Packet ret_i2c_packet;

	memset(receive_buffer, 0, I2C_BUFFER_SIZE);
	if(module_id == 0){
		printf("No Module found\r\n");
		uartResp->id = cmd->id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd->command;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		return;
	}else{
		uartResp->id = cmd->id;
		uartResp->packet_type = cmd->packet_type;
		uartResp->command = cmd->command;
		if (cmd->packet_type == OW_TX7332){
			if (cmd->command == OW_TX7332_RREG) {
				uartResp->data_len = 4;
			} else {
				uartResp->data_len = 0;
			}
		} else {
			uartResp->data_len = cmd->data_len;
		}
	}

	rx_len = read_buffer_of_slave_global(slave_addr, receive_buffer, uartResp->data_len);
	printf("Received %d Bytes \r\n", rx_len);
	if(i2c_packet_fromBuffer(receive_buffer, &ret_i2c_packet)){
		uartResp->packet_type = ret_i2c_packet.reserved;
		uartResp->data_len = ret_i2c_packet.data_len;
		uartResp->data = (uint8_t *)ret_i2c_packet.pData;
	} else {
		uartResp->packet_type = OW_ERROR;
	}
}

static void process_i2c_forward(UartPacket *uartResp, UartPacket* cmd, uint8_t module_id)
{
	I2C_TX_Packet send_i2c_packet;
	uint16_t send_len = 0;
	uint8_t slave_addr = 0;
	int local_tx_idx = 0;

	if(module_id == 0){
		uartResp->id = cmd->id;
		uartResp->command = cmd->command;
		return;
	}

	memset(send_buff, 0, I2C_BUFFER_SIZE);

	slave_addr = ModuleManager_GetModule(module_id)->i2c_address;
	local_tx_idx = cmd->addr - (module_id * TX_PER_MODULE);

	if(local_tx_idx<0 || local_tx_idx>1){
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd->command;
	}else {
		// relay to one of the slaves
		send_i2c_packet.id = cmd->id;
		send_i2c_packet.cmd = cmd->command;
		send_i2c_packet.reserved = (uint8_t)local_tx_idx;
		send_i2c_packet.data_len = cmd->data_len;
		send_i2c_packet.pData = cmd->data;

		send_len = i2c_packet_toBuffer(&send_i2c_packet, send_buff);  // rebuild buffer

		if(send_buffer_to_slave_global(slave_addr, send_buff, send_len) != 0) { // send buffer to slave
			uartResp->packet_type = OW_ERROR;
		}else{
			HAL_Delay(250);
			process_i2c_read_buffer(uartResp, cmd, module_id);
		}
	}

}

static void ONE_WIRE_ProcessCommand(UartPacket *uartResp, UartPacket *cmd)
{
	uint8_t module_id = 0;

	switch (cmd->command)
	{
		case OW_CMD_PING:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			break;
		case OW_CMD_PONG:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			break;
		case OW_CMD_VERSION:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
            uartResp->data_len = sizeof(FW_VERSION_STRING);
            uartResp->data = (uint8_t*)FW_VERSION_STRING;
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = cmd->data_len;
			uartResp->data = cmd->data;
			break;
		case OW_CMD_TOGGLE_LED:
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			HAL_GPIO_TogglePin(SYSTEM_RDY_GPIO_Port, SYSTEM_RDY_Pin); //no led pins declared
			break;
		case OW_CMD_HWID:
			uartResp->command = OW_CMD_HWID;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			id_words[0] = HAL_GetUIDw0();
			id_words[1] = HAL_GetUIDw1();
			id_words[2] = HAL_GetUIDw2();
			uartResp->data_len = 16;
			uartResp->data = (uint8_t *)&id_words;
			break;
		case OW_CMD_GET_TEMP:
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			uartResp->data_len = 4;
			uartResp->data = (uint8_t *)&tx_temperature;
			break;
		case OW_CMD_GET_AMBIENT:
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			uartResp->data_len = 4;
			uartResp->data = (uint8_t *)&ambient_temperature;
			break;
		case OW_CMD_DISCOVERY:
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			if(cmd->reserved==0 || cmd->addr < 0x20 || cmd->addr > 0x25){
				printf("Error reserved or addr wrong\r\n");
				uartResp->packet_type = OW_ERROR;
				break;
			}

			if(get_configured() && get_module_ID() != 0) {
				// relay to next slave if it exists
				printf("discovery timeout\r\n");
				uartResp->packet_type = OW_TIMEOUT;
				break;
			}

			set_configured(true);
			set_module_ID(cmd->reserved);
			set_slave_address(cmd->addr);
			break;
		case OW_CMD_RESET:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
				uartResp->data_len = 0;

		    __HAL_LPTIM_CLEAR_FLAG(&RESET_TIMER, LPTIM_FLAG_ARRM | LPTIM_FLAG_CMPM |
		                                          LPTIM_FLAG_EXTTRIG | LPTIM_FLAG_DOWN |
		                                          LPTIM_FLAG_UP    | LPTIM_FLAG_ARROK);
			if( HAL_LPTIM_Counter_Start_IT(&RESET_TIMER, 1500000) != HAL_OK){
					uartResp->packet_type = OW_ERROR;
				}
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_DFU:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
				uartResp->data_len = 0;

				_enter_dfu = true;

				__HAL_LPTIM_CLEAR_FLAG(&RESET_TIMER, LPTIM_FLAG_ARRM | LPTIM_FLAG_CMPM |
													LPTIM_FLAG_EXTTRIG | LPTIM_FLAG_DOWN |
													LPTIM_FLAG_UP    | LPTIM_FLAG_ARROK);
				if(HAL_LPTIM_Counter_Start_IT(&RESET_TIMER, 1500000) != HAL_OK){
					uartResp->packet_type = OW_ERROR;
				}
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		default:
			uartResp->addr = 0;
			uartResp->reserved = 0;
			uartResp->data_len = 0;
			uartResp->reserved = OW_INVALID_PACKET;
			uartResp->packet_type = OW_ERROR;
			break;
	}
}

static void CONTROLLER_ProcessCommand(UartPacket *uartResp, UartPacket* cmd)
{
	uint8_t module_id = 0;

	switch (cmd->command)
	{
		case OW_CMD_PING:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_PONG:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			break;
		case OW_CMD_VERSION:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			cmd->data_len = sizeof(FW_VERSION_STRING); //passing amount to read if forwarding to slave
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
				uartResp->data_len = sizeof(FW_VERSION_STRING);
				uartResp->data = (uint8_t*)FW_VERSION_STRING;
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->id = cmd->id;
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = cmd->data_len;
			uartResp->data = cmd->data;
			break;
		case OW_CMD_TOGGLE_LED:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00)
			{
				uartResp->data_len = 0;
				uartResp->id = cmd->id;
				uartResp->command = cmd->command;
				HAL_GPIO_TogglePin(SYSTEM_RDY_GPIO_Port, SYSTEM_RDY_Pin); 
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_HWID:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			cmd->data_len = HW_ID_DATA_LENGTH; //passing amount to read if forwarding to slave
			if (module_id == 0x00)
			{
				uartResp->id = cmd->id;
				uartResp->command = cmd->command;
				uartResp->reserved = cmd->reserved;
				uartResp->addr = cmd->addr;
				id_words[0] = HAL_GetUIDw0();
				id_words[1] = HAL_GetUIDw1();
				id_words[2] = HAL_GetUIDw2();
				uartResp->data_len = HW_ID_DATA_LENGTH;
				uartResp->data = (uint8_t *)&id_words;
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_GET_TEMP:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			cmd->data_len = TEMPERATURE_DATA_LENGTH; //passing amount to read if forwarding to slave
			if (module_id == 0){
				uartResp->id = cmd->id;
				uartResp->command = cmd->command;
				uartResp->data_len = TEMPERATURE_DATA_LENGTH;
				uartResp->data = (uint8_t *)&tx_temperature;
			}else{
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_GET_AMBIENT:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			cmd->data_len = TEMPERATURE_DATA_LENGTH; //passing amount to read if forwarding to slave
			if (module_id == 0){
				uartResp->id = cmd->id;
				uartResp->command = cmd->command;
				uartResp->data_len = TEMPERATURE_DATA_LENGTH;
				uartResp->data = (uint8_t *)&ambient_temperature;
			}else{
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CTRL_START_SWTRIG:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;
			if(start_trigger_pulse() != TRIGGER_STATUS_RUNNING)
			{
				uartResp->packet_type = OW_ERROR;
			}
			break;
		case OW_CTRL_STOP_SWTRIG:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;
			if(stop_trigger_pulse() != TRIGGER_STATUS_READY)
			{
				uartResp->packet_type = OW_ERROR;
			}
			break;
		case OW_CTRL_SET_SWTRIG:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = 0;

			if(!set_trigger_data((char *)cmd->data, cmd->data_len))
			{
				uartResp->packet_type = OW_ERROR;
			}else{
				// refresh state
				if(!get_trigger_data(retTriggerJson, 0xFF))
				{
					uartResp->packet_type = OW_ERROR;
				}else{
					uartResp->data_len = strlen(retTriggerJson);
					uartResp->data = (uint8_t *)retTriggerJson;
				}
			}

			break;
		case OW_CTRL_GET_SWTRIG:
			// refresh state
			if(!get_trigger_data(retTriggerJson, 0xFF))
			{
				uartResp->packet_type = OW_ERROR;
				break;
			}
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			uartResp->reserved = cmd->reserved;
			uartResp->data_len = strlen(retTriggerJson);
			uartResp->data = (uint8_t *)retTriggerJson;
			break;
		case OW_CMD_RESET:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
				uartResp->data_len = 0;

				__HAL_LPTIM_CLEAR_FLAG(&RESET_TIMER, LPTIM_FLAG_ARRM | LPTIM_FLAG_CMPM |
													  LPTIM_FLAG_EXTTRIG | LPTIM_FLAG_DOWN |
													  LPTIM_FLAG_UP    | LPTIM_FLAG_ARROK);
				if( HAL_LPTIM_Counter_Start_IT(&RESET_TIMER, 1500000) != HAL_OK){
						uartResp->packet_type = OW_ERROR;
				}
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_DFU:
			module_id = ModuleManager_GetModuleIndex(cmd->addr);
			if (module_id == 0x00){
				uartResp->command = cmd->command;
				uartResp->addr = cmd->addr;
				uartResp->reserved = cmd->reserved;
				uartResp->data_len = 0;

				_enter_dfu = true;

				__HAL_LPTIM_CLEAR_FLAG(&RESET_TIMER, LPTIM_FLAG_ARRM | LPTIM_FLAG_CMPM |
													LPTIM_FLAG_EXTTRIG | LPTIM_FLAG_DOWN |
													LPTIM_FLAG_UP    | LPTIM_FLAG_ARROK);
				if(HAL_LPTIM_Counter_Start_IT(&RESET_TIMER, 1500000) != HAL_OK){
					uartResp->packet_type = OW_ERROR;
				}
			} else {
				process_i2c_forward(uartResp, cmd, module_id);
			}
			break;
		case OW_CMD_ASYNC:
			uartResp->command = cmd->command;
			uartResp->addr = cmd->addr;
			if(cmd->data_len == 1){
				async_enabled = cmd->data[0] == 1? true: false;
			}
			uartResp->reserved = async_enabled?1:0;
			uartResp->data_len = 0;
			break;
		default:
			uartResp->addr = 0;
			uartResp->reserved = OW_INVALID_PACKET;
			uartResp->data_len = 0;
			uartResp->packet_type = OW_ERROR;
			break;
	}

}

static void TX7332_ProcessCommand(UartPacket *uartResp, UartPacket* cmd)
{
	uint8_t module_id = 0;
	uint16_t reg_address = 0;
	uint32_t reg_value = 0;
	uint32_t reg_data_buff[REG_DATA_LEN] = {0};
	int reg_count = 0;

	uartResp->id = cmd->id;
	uartResp->command = cmd->command;

	switch (cmd->command)
	{
	case OW_TX7332_ENUM:
		// send array of tx chip counts 0,1,2,3,4,...
		uartResp->command = OW_TX7332_ENUM;
		uartResp->addr = cmd->addr;
		// Here we will have the array for all tx chips with 0,1 on the controller
		// and 2,3 on the first slave in the chain and so on
		uartResp->reserved = (uint8_t)get_tx_chip_count();
		uartResp->data_len = 0;
		uartResp->data = NULL;
		break;
	case OW_TX7332_DEMO:
		uartResp->command = OW_TX7332_DEMO;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		uartResp->addr = cmd->addr;
		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			write_demo_registers(&transmitters[cmd->addr]);
	    }else{
			process_i2c_forward(uartResp, cmd, module_id);
		}

		uartResp->reserved = (uint8_t)get_tx_chip_count();
		break;
	case OW_TX7332_WREG:
		uartResp->command = OW_TX7332_WREG;
		uartResp->addr = cmd->addr;
		uartResp->reserved = 0;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if(cmd->data_len != 6 || cmd->addr >= get_tx_chip_count()){
			uartResp->packet_type = OW_ERROR;
			break;
		}

		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_address = cmd->data[0] | (cmd->data[1] << 8);
			// Unpack 32-bit value (next 4 bytes, little-endian)
			reg_value = cmd->data[2] | (cmd->data[3] << 8) | (cmd->data[4] << 16) | (cmd->data[5] << 24);

			TX7332_WriteReg(&transmitters[cmd->addr], reg_address, reg_value);
		}
		else
		{
			process_i2c_forward(uartResp, cmd, module_id);
		}

		break;
	case OW_TX7332_RREG:
		uartResp->command = OW_TX7332_RREG;
		uartResp->addr = cmd->addr;
		uartResp->reserved = 0;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if(cmd->data_len != 2 || cmd->addr >= get_tx_chip_count()){
			uartResp->packet_type = OW_ERROR;
			break;
		}

		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_address = cmd->data[0] | (cmd->data[1] << 8);
			reg_value = 0;

			reg_value = TX7332_ReadReg(&transmitters[cmd->addr], reg_address);
			memset(reg_data_buff,0,REG_DATA_LEN*sizeof(reg_value));

			// Package response
			reg_data_buff[0] = reg_value;

			uartResp->data_len = sizeof(reg_value);
			uartResp->data = (uint8_t*)reg_data_buff;
		}else{
			process_i2c_forward(uartResp, cmd, module_id);
		}
		break;
	case OW_TX7332_WBLOCK:
		uartResp->command = OW_TX7332_WBLOCK;
		uartResp->addr = cmd->addr;
		uartResp->reserved = 0;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if(cmd->data_len <= 6 || cmd->addr >= get_tx_chip_count()){
			uartResp->packet_type = OW_ERROR;
			break;
		}

		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_address = cmd->data[0] | (cmd->data[1] << 8);
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_count = cmd->data[2];
			// byte [3] dummy byte
			// Check if the actual data length matches expected length
			if(cmd->data_len != (4 + (4 * reg_count)))
			{
				// printf("Invalid data size does not match \r\n");
				uartResp->packet_type = OW_ERROR;
				break;
			}


			memset(reg_data_buff,0,REG_DATA_LEN*sizeof(reg_value));

			memcpy((uint8_t*)reg_data_buff, &cmd->data[4], sizeof(uint32_t) * reg_count);
			if(!TX7332_WriteBulk(&transmitters[cmd->addr], reg_address, reg_data_buff, reg_count)){
				uartResp->packet_type = OW_ERROR;
				break;
			}
		}else{
			process_i2c_forward(uartResp, cmd, module_id);
		}

		break;
	case OW_TX7332_VWREG:
		uartResp->command = OW_TX7332_VWREG;
		uartResp->addr = cmd->addr;
		uartResp->reserved = 0;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if(cmd->data_len != 6 || cmd->addr >= get_tx_chip_count()){
			uartResp->packet_type = OW_ERROR;
			break;
		}

		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_address = cmd->data[0] | (cmd->data[1] << 8);
			// Unpack 32-bit value (next 4 bytes, little-endian)
			reg_value = cmd->data[2] | (cmd->data[3] << 8) | (cmd->data[4] << 16) | (cmd->data[5] << 24);

			if(!TX7332_WriteVerify(&transmitters[cmd->addr], reg_address, reg_value))
			{
				uartResp->packet_type = OW_ERROR;
			}
		}else{
			process_i2c_forward(uartResp, cmd, module_id);
		}

		break;
	case OW_TX7332_VWBLOCK:
		uartResp->command = OW_TX7332_VWBLOCK;
		uartResp->addr = cmd->addr;
		uartResp->reserved = 0;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if(cmd->data_len <= 6 || cmd->addr >= get_tx_chip_count()){
			uartResp->packet_type = OW_ERROR;
			break;
		}

		module_id = ModuleManager_GetModuleIndex(cmd->addr);

		if(module_id == 0x00) // local
		{
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_address = cmd->data[0] | (cmd->data[1] << 8);
			// Unpack 16-bit address (first 2 bytes, little-endian)
			reg_count = cmd->data[2];
			// byte [3] dummy byte
			// Check if the actual data length matches expected length
			if(cmd->data_len != (4 + (4 * reg_count)))
			{
				// printf("Invalid data size does not match \r\n");
				uartResp->packet_type = OW_ERROR;
				break;
			}


			memset(reg_data_buff,0,REG_DATA_LEN*sizeof(reg_value));

			memcpy((uint8_t*)reg_data_buff, &cmd->data[4], sizeof(uint32_t) * reg_count);
			if(!TX7332_WriteBulkVerify(&transmitters[cmd->addr], reg_address, reg_data_buff, reg_count)){
				uartResp->packet_type = OW_ERROR;
				break;
			}
		}else{
			process_i2c_forward(uartResp, cmd, module_id);
		}
		break;
	case OW_TX7332_DEVICE_COUNT:
	{
		uint8_t temp_module_count = get_module_count();
		uartResp->command = OW_TX7332_DEVICE_COUNT;
		uartResp->addr = 0;
		uartResp->reserved = 0;
		uartResp->data = &temp_module_count;
		uartResp->data_len = 1;
	}
		break;
	case OW_TX7332_RESET:
		uartResp->command = OW_TX7332_RESET;
		uartResp->addr = 0;
		uartResp->reserved = 0;
		uartResp->data = NULL;
		uartResp->data_len = 0;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->reserved = OW_INVALID_PACKET;
		uartResp->packet_type = OW_ERROR;
		break;
	}
}

bool process_if_command(UartPacket *cmd, UartPacket *resp)
{
	// I2C_TX_Packet i2c_packet;
	(void)print_uart_packet;

	resp->id = cmd->id;
	if(cmd->packet_type == OW_ONE_WIRE){
		resp->packet_type = OW_ONEWIRE_RESP;
	}else{
		resp->packet_type = OW_RESP;
	}
	resp->addr = 0;
	resp->reserved = 0;
	resp->data_len = 0;
	resp->data = 0;
	switch (cmd->packet_type)
	{
	case OW_ONE_WIRE:
		ONE_WIRE_ProcessCommand(resp, cmd);
		break;
	case OW_CMD:
	case OW_CONTROLLER:
		//process by the USTX Controller
		CONTROLLER_ProcessCommand(resp, cmd);
		break;
	case OW_TX7332:
		TX7332_ProcessCommand(resp, cmd);
		break;
	default:
		resp->data_len = 0;
		resp->reserved = OW_UNKNOWN_ERROR;
		resp->packet_type = OW_ERROR;
		break;
	}

	return true;

}
