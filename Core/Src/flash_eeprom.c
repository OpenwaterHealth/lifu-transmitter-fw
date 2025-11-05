/*
 * flash_eeprom.c
 *
 *  Created on: Mar 4, 2024
 *      Author: gvigelet
 */

#include "flash_eeprom.h"

/* Function to write data to Flash */
HAL_StatusTypeDef Flash_Write(uint32_t address, uint32_t* data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK){
    	return status;
    }


    for (uint32_t i = 0; i < size; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, address + (i * sizeof(uint32_t)), data[i]);
        if (status != HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();
    return status;
}

/* Function to read data from Flash */
HAL_StatusTypeDef Flash_Read(uint32_t address, uint32_t* data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint32_t*)(address + (i * sizeof(uint32_t)));
    }
    return HAL_OK;
}

/* Function to erase Flash memory */
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address) {
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t error;

    status = HAL_FLASH_Unlock();
    if(status != HAL_OK){
    	return status;
    }

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = start_address;
    erase_init.NbPages = (end_address - start_address) / FLASH_PAGE_SIZE;

    status = HAL_FLASHEx_Erase(&erase_init, &error);

    HAL_FLASH_Lock();
    return status;
}
