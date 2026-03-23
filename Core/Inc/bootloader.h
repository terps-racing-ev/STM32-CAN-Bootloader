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
  * Debug LED Indicators:
  * - LED1 (PB1): Bootloader Running - Always ON when bootloader is active
  * - LED2 (PA8): Valid Application - ON if valid app exists, OFF if no app
  * - LED3 (PB5): CAN Activity - Turns ON when any CAN message received from host
  *               Blinks 20 times rapidly before jumping to application
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
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/* LED Pin Definitions - Onboard LED at PB3 */
#define LED_PORT                    GPIOB
#define LED_PIN                     GPIO_PIN_3

/* Memory layout definitions */
/* Bootloader code must stay within pages 0-14 (30KB).
 * Page 15 is reserved for boot metadata. */
#define BOOTLOADER_CODE_SIZE        0x7800      /* 30KB for bootloader code */
#define BOOT_METADATA_ADDRESS       0x08007800  /* Start of page 15 (2KB metadata page) */
#define BOOT_METADATA_PAGE          15u

#define BANK_A_ADDRESS              0x08008000  /* Application bank A start */
#define BANK_B_ADDRESS              0x08022000  /* Application bank B start */
#define BANK_SIZE                   0x1A000     /* 104KB per bank */
#define BANK_A_END_ADDRESS          (BANK_A_ADDRESS + BANK_SIZE - 1)
#define BANK_B_END_ADDRESS          (BANK_B_ADDRESS + BANK_SIZE - 1)

#define PERMANENT_STORAGE_SIZE      0x4000      /* 16KB reserved for permanent data storage */
#define PERMANENT_STORAGE_ADDRESS   0x0803C000  /* Last 16KB of flash (256KB - 16KB) */
#define FLASH_END_ADDRESS           0x0803FFFF  /* Physical end of 256KB flash */

#define BOOT_METADATA_MAGIC         0xAB12AB12u
#define BOOT_METADATA_UPDATE_IDLE   0x00u
#define BOOT_METADATA_UPDATE_IN_PROGRESS 0xA5u

#define BOOT_BANK_A                 0u
#define BOOT_BANK_B                 1u
#define BOOT_BANK_INVALID           0xFFu

/* Bootloader timeout */
#define BOOTLOADER_TIMEOUT_MS       1000        /* 10 second timeout */

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
#define CMD_GET_ACTIVE_BANK         0x08        /* Get active/valid bank info */
#define CMD_SET_IMAGE_INFO          0x09        /* Set expected CRC32 + image size */
#define CMD_VERIFY_BANK             0x0A        /* Verify inactive bank CRC */

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
#define ERR_CRC_MISMATCH            0x08

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

  typedef struct {
    uint32_t magic;
    uint8_t active_bank;
    uint8_t bank_a_valid;
    uint8_t bank_b_valid;
    uint8_t reserved0;              /* Update transaction marker */
    uint32_t bank_a_crc;
    uint32_t bank_b_crc;
    uint32_t bank_a_size;
    uint32_t bank_b_size;
    uint32_t complement;
    uint32_t reserved1;
  } BootMetadata_t;

/* Exported functions prototypes ---------------------------------------------*/
void Bootloader_Init(CAN_HandleTypeDef *hcan);
void Bootloader_Main(void);
void Bootloader_ProcessCANMessage(void);
uint8_t Bootloader_JumpToApplication(void);
uint8_t Bootloader_CheckValidApplication(uint8_t bank);
void Bootloader_SendCANMessage(uint8_t cmd, uint8_t *data, uint8_t length);

/* LED control functions */
void Bootloader_LED_Init(void);
void Bootloader_LED_UpdateStatus(void);
void Bootloader_LED_FlashingStart(void);
void Bootloader_LED_FlashingStop(void);
void Bootloader_LED_FlashingUpdate(void);
void Bootloader_LED_BlinkBeforeJump(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
