/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bootloader.h
  * @brief          : Header for bootloader.c file.
  *                   This file contains the bootloader definitions and prototypes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported defines ----------------------------------------------------------*/
/* Memory layout definitions */
#define BOOTLOADER_SIZE             0x8000      /* 32KB for bootloader */
#define APPLICATION_ADDRESS         0x08008000  /* Application starts at 32KB offset */
#define PERMANENT_STORAGE_SIZE      0x4000      /* 16KB reserved for permanent data storage */
#define PERMANENT_STORAGE_ADDRESS   0x0803C000  /* Last 16KB of flash (256KB - 16KB) */
#define FLASH_END_ADDRESS           0x0803FFFF  /* Physical end of 256KB flash */
#define APPLICATION_END_ADDRESS     (PERMANENT_STORAGE_ADDRESS - 1)  /* 0x0803BFFF */
#define APPLICATION_SIZE            (APPLICATION_END_ADDRESS - APPLICATION_ADDRESS + 1)  /* 208KB */

/* Application valid flag storage (last 8 bytes before application) */
#define APP_VALID_FLAG_ADDRESS      0x08007FF8  /* Last 8 bytes of bootloader flash */
#define APP_VALID_MAGIC_NUMBER      0xDEADBEEF  /* Magic number indicating valid app */
#define APP_VALID_FLAG_COMPLEMENT   0x21524110  /* Bitwise NOT of magic number */

/* Bootloader timeout */
#define BOOTLOADER_TIMEOUT_MS       5000        /* 5 second timeout */

/* Heartbeat interval */
#define HEARTBEAT_INTERVAL_MS       1000         /* 1 second heartbeat interval */

/* Flash operation definitions */
/* Note: FLASH_PAGE_SIZE already defined in stm32l4xx_hal_flash.h */
#define FLASH_TIMEOUT               50000       /* Flash operation timeout */

/* CAN Protocol definitions - 29-bit Extended IDs */
#define CAN_BOOTLOADER_ID           0x18000700  /* Bootloader CAN ID (Extended) */
#define CAN_HOST_ID                 0x18000701  /* Host/PC CAN ID (Extended) */

/* CAN Command IDs */
#define CMD_ERASE_FLASH             0x01        /* Erase application flash */
#define CMD_WRITE_FLASH             0x02        /* Write data to flash (legacy) */
#define CMD_READ_FLASH              0x03        /* Read data from flash */
#define CMD_JUMP_TO_APP             0x04        /* Jump to application */
#define CMD_GET_STATUS              0x05        /* Get bootloader status */
#define CMD_SET_ADDRESS             0x06        /* Set write address */
#define CMD_WRITE_DATA              0x07        /* Write 4 bytes (buffers 2 chunks for 8-byte write) */

/* CAN Response IDs */
#define RESP_ACK                    0x10        /* Command acknowledged */
#define RESP_NACK                   0x11        /* Command not acknowledged */
#define RESP_ERROR                  0x12        /* Error occurred */
#define RESP_BUSY                   0x13        /* Bootloader busy */
#define RESP_READY                  0x14        /* Bootloader ready */
#define RESP_DATA                   0x15        /* Data response */
#define RESP_JUMP_INFO              0x16        /* Jump to application info */

/* Error codes */
#define ERR_NONE                    0x00
#define ERR_INVALID_COMMAND         0x01
#define ERR_INVALID_ADDRESS         0x02
#define ERR_FLASH_ERASE_FAILED      0x03
#define ERR_FLASH_WRITE_FAILED      0x04
#define ERR_INVALID_DATA_LENGTH     0x05
#define ERR_NO_VALID_APP            0x06
#define ERR_TIMEOUT                 0x07

/* Bootloader states */
typedef enum {
    BL_STATE_IDLE = 0,
    BL_STATE_ERASING,
    BL_STATE_WRITING,
    BL_STATE_READING,
    BL_STATE_VERIFYING,
    BL_STATE_JUMPING
} BootloaderState_t;

/* Bootloader status structure */
typedef struct {
    BootloaderState_t state;
    uint32_t current_address;
    uint32_t bytes_written;
    uint32_t total_bytes;
    uint8_t last_error;
} BootloaderStatus_t;

/* Exported functions prototypes ---------------------------------------------*/
void Bootloader_Init(CAN_HandleTypeDef *hcan);
void Bootloader_Main(void);
void Bootloader_ProcessCANMessage(void);
uint8_t Bootloader_JumpToApplication(void);
uint8_t Bootloader_CheckValidApplication(void);
void Bootloader_SendCANMessage(uint8_t cmd, uint8_t *data, uint8_t length);
uint8_t Bootloader_SetApplicationValidFlag(void);
uint8_t Bootloader_CheckApplicationValidFlag(void);
void Bootloader_ClearApplicationValidFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
