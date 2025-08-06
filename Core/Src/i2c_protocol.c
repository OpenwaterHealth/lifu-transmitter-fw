/*
 * i2c_protocol.c
 *
 *  Created on: Apr 1, 2024
 *      Author: gvigelet
 */


#include "i2c_protocol.h"
#include "i2c_slave.h"

#include "utils.h"
#include <stdio.h>


void i2c_tx_packet_print(const I2C_TX_Packet* packet) {
    printf("\r\nI2C TX PACKET\r\n\r\n");
    printf("Packet Length: 0x%02X\r\n", packet->pkt_len);
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Command (cmd): 0x%02X\r\n", packet->cmd);
    printf("Reserved: 0x%02X\r\n", packet->reserved);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->pData[i]);
    }
    printf("\r\nCRC: 0x%04X\r\n", packet->crc);
    printf("Packet Length: %d\r\n\r\n", packet->data_len + 8);

}

void i2c_status_packet_print(const I2C_STATUS_Packet* packet)
{
    printf("\r\nI2C STATUS PACKET\r\n\r\n");
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Command (cmd): 0x%02X\r\n", packet->cmd);
    printf("Status: 0x%02X\r\n", packet->status);
    printf("Reserved: 0x%02X\r\n", packet->reserved);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("\r\nCRC: 0x%04X\r\n", packet->crc);
    printf("Packet Length: %d\r\n\r\n", (int)sizeof(I2C_STATUS_Packet));

}

bool i2c_packet_fromBuffer(const uint8_t* buffer, I2C_TX_Packet* pTX) {
    bool ret = false;
    uint16_t crc = 0xFFFF;
    const uint8_t* pBuff = buffer;
    pTX->pkt_len = *buffer;
    buffer++;
    pTX->id = (uint16_t)((buffer[1] << 8) | buffer[0]); // Packet ID (little-endian)
    buffer += 2;
    pTX->cmd = *buffer; // Command ID
    buffer++;
    pTX->reserved = *buffer; // reserved used currently for passing data
    buffer++;
    pTX->data_len = *buffer;
    buffer++;
    pTX->pData = (uint8_t*)buffer;
    buffer += pTX->data_len;
    pTX->crc = (uint16_t)((buffer[1] << 8) | buffer[0]); // CRC (little-endian)

    // Calculate CRC
    crc = util_crc16(pBuff, pTX->data_len + 6);
    ret = crc == pTX->crc;
    return ret;
}

size_t i2c_packet_toBuffer(I2C_TX_Packet* pTX, uint8_t* buffer) {
    if (pTX == NULL || buffer == NULL) {
        // Handle error: invalid pointer
        return 0;
    }

    int i = 0;
    uint8_t* pBuff = buffer;

    pTX->crc = 0xFFFF;

    // packet length
    *buffer = pTX->data_len + 8;
    pTX->pkt_len = *buffer;
    buffer++;

    // Write Packet ID
    buffer[0] = (uint8_t)(pTX->id & 0xFF);
    buffer[1] = (uint8_t)((pTX->id >> 8) & 0xFF);
    buffer += 2;

    // Write Command ID
    *buffer = pTX->cmd;
    buffer++;

    // Write reserved data
    *buffer = pTX->reserved;
    buffer++;

    // Write Data Length
    *buffer = pTX->data_len;
    if (*buffer == 16 || *buffer == 3)
    {
        //flash_led();
    }
    buffer++;

    // Write Data
    if (pTX->pData) {
        for (i = 0; i < pTX->data_len; i++) {
            *buffer = pTX->pData[i];
            buffer++;
        }
    }

    // Calculate CRC
    pTX->crc = util_crc16(pBuff, buffer - pBuff);

    // Write CRC
    buffer[0] = (uint8_t)(pTX->crc & 0xFF);
    buffer[1] = (uint8_t)((pTX->crc >> 8) & 0xFF);
    buffer += 2;

    // Return the total packet size
    return (buffer - pBuff);
}

bool i2c_status_packet_fromBuffer(const uint8_t* buffer, I2C_STATUS_Packet* pTX) {
    bool ret = false;
    uint16_t crc = 0xFFFF;
    const uint8_t* pBuff = buffer;
    pTX->id = (uint16_t)((buffer[1] << 8) | buffer[0]); // Packet ID (little-endian)
    buffer += 2;
    pTX->cmd = *buffer; // Command ID
    buffer++;
    pTX->status = *buffer; // Status ID
    buffer++;
    pTX->reserved = *buffer; // Reserved
    buffer++;
    pTX->data_len = *buffer;
    if (pTX->data_len == 16 || pTX->data_len == 3)
    {
        //flash_led();
    }
    buffer++;
    pTX->crc = (uint16_t)((buffer[1] << 8) | buffer[0]); // CRC (little-endian)

    // Calculate CRC
    crc = util_crc16(pBuff, 6);
    ret = crc == pTX->crc;
    return ret;
}

size_t i2c_status_packet_toBuffer(I2C_STATUS_Packet* pTX, uint8_t* buffer) {
    if (pTX == NULL || buffer == NULL) {
        // Handle error: invalid pointer
        return 0;
    }

    uint8_t* pBuff = buffer;

    pTX->crc = 0xFFFF;

    // Write Packet ID
    buffer[0] = (uint8_t)(pTX->id & 0xFF);
    buffer[1] = (uint8_t)((pTX->id >> 8) & 0xFF);
    buffer += 2;

    // Write Command ID
    *buffer = pTX->cmd;
    buffer++;

    // Write Status ID
    *buffer = pTX->status;
    buffer++;

    // Write Reserved
    *buffer = pTX->reserved;
    buffer++;

    // Write Data Length
    *buffer = pTX->data_len;
    buffer++;

    // Calculate CRC
    pTX->crc = util_crc16(pBuff, buffer - pBuff);

    // Write CRC
    buffer[0] = (uint8_t)(pTX->crc & 0xFF);
    buffer[1] = (uint8_t)((pTX->crc >> 8) & 0xFF);
    buffer += 2;

    // Return the total packet size
    return (buffer - pBuff);
}
