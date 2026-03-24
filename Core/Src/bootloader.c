/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bootloader.c
  * @brief          : CAN Bootloader implementation for STM32L432
  ******************************************************************************
  * @attention
  *
  * This bootloader implements a CAN-based firmware update mechanism.
  * It allows flashing new application firmware via CAN bus messages.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bootloader.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static CAN_HandleTypeDef *hcan_bootloader;
static BootloaderStatus_t bootloader_status;
static uint8_t rx_data[8];
static uint8_t tx_data[8];
static CAN_TxHeaderTypeDef tx_header;
static uint32_t tx_mailbox;
static uint8_t flash_buffer[8];     /* Buffer for 8-byte flash writes (two 4-byte chunks) */
static uint16_t buffer_index = 0;   /* Current position in flash_buffer */
static volatile uint8_t jump_to_app_flag = 0;  /* Flag to trigger jump from main loop */
static volatile uint8_t can_command_received = 0;  /* Flag to disable auto-jump timeout */
static uint32_t last_heartbeat_time = 0;  /* Last heartbeat timestamp for resetting on commands */
static volatile uint8_t rx_msg_pending = 0;  /* Flag: message waiting for main loop processing */
static BootMetadata_t boot_metadata;
static uint8_t metadata_ready = 0;
static uint8_t jump_target_bank = BOOT_BANK_INVALID;
static uint32_t pending_image_crc = 0;
static uint32_t pending_image_size = 0;
static uint8_t pending_image_info_valid = 0;
static uint8_t pending_verified_bank = BOOT_BANK_INVALID;
static uint8_t bank_a_crc_ok = 0;  /* Startup CRC validation result for bank A */
static uint8_t bank_b_crc_ok = 0;  /* Startup CRC validation result for bank B */

/* Private function prototypes -----------------------------------------------*/
static void Bootloader_ConfigureCANFilter(void);
static uint8_t Bootloader_EraseBank(uint8_t bank);
static uint8_t Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint16_t length);
static uint8_t Bootloader_ReadFlash(uint32_t address, uint8_t *data, uint16_t length);
static void Bootloader_DeInit(void);
static void Bootloader_SendACK(void);
static void Bootloader_SendNACK(uint8_t error_code);
static void Bootloader_WaitForCANTransmission(void);
static uint8_t Bootloader_ReadMetadata(BootMetadata_t *meta);
static uint8_t Bootloader_WriteMetadata(const BootMetadata_t *meta);
static void Bootloader_SetDefaultMetadata(BootMetadata_t *meta);
static uint8_t Bootloader_GetActiveBank(void);
static uint8_t Bootloader_GetInactiveBank(void);
static uint32_t Bootloader_GetBankStartAddress(uint8_t bank);
static uint32_t Bootloader_GetBankEndAddress(uint8_t bank);
static uint8_t Bootloader_IsAddressInBank(uint32_t address, uint32_t length, uint8_t bank);
static uint8_t Bootloader_IsAddressInAnyBank(uint32_t address, uint32_t length);
static uint32_t Bootloader_ComputeCRC32(uint32_t start_address, uint32_t size);
static uint8_t Bootloader_JumpToBank(uint8_t bank);
static void Bootloader_FillReadyHeartbeatPayload(uint8_t code1);
static uint8_t Bootloader_IsBankMarkedValid(uint8_t bank);
static uint8_t Bootloader_IsBankCrcValid(uint8_t bank);
static void Bootloader_InvalidateBankMetadata(uint8_t bank);
static uint8_t Bootloader_IsUpdateInProgress(void);

static uint8_t Bootloader_IsUpdateInProgress(void)
{
    if (!metadata_ready)
    {
        return 0;
    }

    return (boot_metadata.reserved0 == BOOT_METADATA_UPDATE_IN_PROGRESS) ? 1 : 0;
}

static void Bootloader_FillReadyHeartbeatPayload(uint8_t code1)
{
    uint8_t flags = 0;
    uint8_t crc_health = 0;

    if (Bootloader_GetActiveBank() == BOOT_BANK_B)
    {
        flags |= (1u << 0);
    }
    if (boot_metadata.bank_a_valid)
    {
        flags |= (1u << 1);
    }
    if (boot_metadata.bank_b_valid)
    {
        flags |= (1u << 2);
    }
    if (metadata_ready)
    {
        flags |= (1u << 3);
    }
    if (can_command_received)
    {
        flags |= (1u << 4);
    }
    if (pending_image_info_valid)
    {
        flags |= (1u << 5);
    }
    if (pending_verified_bank != BOOT_BANK_INVALID)
    {
        flags |= (1u << 6);
    }
    if (jump_to_app_flag)
    {
        flags |= (1u << 7);
    }

    if (bank_a_crc_ok)
    {
        crc_health |= (1u << 0);
    }
    if (bank_b_crc_ok)
    {
        crc_health |= (1u << 1);
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = RESP_READY;
    tx_data[1] = code1;                                /* Version major or special marker */
    tx_data[2] = crc_health;                           /* Bank CRC health flags */
    tx_data[3] = (uint8_t)bootloader_status.state;     /* Bootloader state */
    tx_data[4] = bootloader_status.last_error;         /* Last error code */
    tx_data[5] = flags;                                /* Active/valid bank and diagnostic flags */
    tx_data[6] = (uint8_t)((bootloader_status.bytes_written >> 8) & 0xFF);
    tx_data[7] = (uint8_t)(bootloader_status.bytes_written & 0xFF);
}

static uint8_t Bootloader_ReadMetadata(BootMetadata_t *meta)
{
    const BootMetadata_t *flash_meta = (const BootMetadata_t *)BOOT_METADATA_ADDRESS;

    memcpy(meta, flash_meta, sizeof(BootMetadata_t));

    if (meta->magic != BOOT_METADATA_MAGIC)
    {
        return 0;
    }

    if (meta->complement != ~BOOT_METADATA_MAGIC)
    {
        return 0;
    }

    if (meta->active_bank != BOOT_BANK_A && meta->active_bank != BOOT_BANK_B)
    {
        return 0;
    }

    if (meta->bank_a_size > BANK_SIZE || meta->bank_b_size > BANK_SIZE)
    {
        return 0;
    }

    return 1;
}

static void Bootloader_SetDefaultMetadata(BootMetadata_t *meta)
{
    memset(meta, 0, sizeof(BootMetadata_t));

    meta->magic = BOOT_METADATA_MAGIC;
    meta->active_bank = BOOT_BANK_A;
    meta->bank_a_valid = 0;
    meta->bank_b_valid = 0;
    meta->reserved0 = BOOT_METADATA_UPDATE_IDLE;
    meta->complement = ~BOOT_METADATA_MAGIC;
}

static uint8_t Bootloader_WriteMetadata(const BootMetadata_t *meta)
{
    FLASH_EraseInitTypeDef erase_init;
    HAL_StatusTypeDef status;
    uint32_t page_error;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = BOOT_METADATA_PAGE;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return ERR_FLASH_ERASE_FAILED;
    }

    for (uint32_t i = 0; i < sizeof(BootMetadata_t); i += 8)
    {
        uint64_t data64 = 0;
        memcpy(&data64, ((const uint8_t *)meta) + i, 8);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_METADATA_ADDRESS + i, data64);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return ERR_FLASH_WRITE_FAILED;
        }
    }

    HAL_FLASH_Lock();
    return ERR_NONE;
}

static uint32_t Bootloader_GetBankStartAddress(uint8_t bank)
{
    return (bank == BOOT_BANK_B) ? BANK_B_ADDRESS : BANK_A_ADDRESS;
}

static uint32_t Bootloader_GetBankEndAddress(uint8_t bank)
{
    return (bank == BOOT_BANK_B) ? BANK_B_END_ADDRESS : BANK_A_END_ADDRESS;
}

static uint8_t Bootloader_GetActiveBank(void)
{
    if (!metadata_ready)
    {
        return BOOT_BANK_A;
    }
    return boot_metadata.active_bank;
}

static uint8_t Bootloader_GetInactiveBank(void)
{
    return (Bootloader_GetActiveBank() == BOOT_BANK_A) ? BOOT_BANK_B : BOOT_BANK_A;
}

static uint8_t Bootloader_IsAddressInBank(uint32_t address, uint32_t length, uint8_t bank)
{
    uint32_t bank_start = Bootloader_GetBankStartAddress(bank);
    uint32_t bank_end = Bootloader_GetBankEndAddress(bank);

    if (length == 0)
    {
        return 0;
    }

    if (address < bank_start || address > bank_end)
    {
        return 0;
    }

    if ((address + length - 1) > bank_end)
    {
        return 0;
    }

    return 1;
}

static uint8_t Bootloader_IsAddressInAnyBank(uint32_t address, uint32_t length)
{
    return Bootloader_IsAddressInBank(address, length, BOOT_BANK_A) ||
           Bootloader_IsAddressInBank(address, length, BOOT_BANK_B);
}

static uint32_t Bootloader_ComputeCRC32(uint32_t start_address, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *data = (const uint8_t *)start_address;

    for (uint32_t i = 0; i < size; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 1u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

static uint8_t Bootloader_IsBankMarkedValid(uint8_t bank)
{
    if (!metadata_ready)
    {
        return 0;
    }

    if (bank == BOOT_BANK_A)
    {
        return boot_metadata.bank_a_valid && (boot_metadata.bank_a_size > 0) && (boot_metadata.bank_a_size <= BANK_SIZE);
    }

    if (bank == BOOT_BANK_B)
    {
        return boot_metadata.bank_b_valid && (boot_metadata.bank_b_size > 0) && (boot_metadata.bank_b_size <= BANK_SIZE);
    }

    return 0;
}

static uint8_t Bootloader_IsBankCrcValid(uint8_t bank)
{
    uint32_t expected_crc;
    uint32_t image_size;
    uint32_t bank_start;
    uint32_t computed_crc;

    if (!Bootloader_IsBankMarkedValid(bank))
    {
        return 0;
    }

    if (bank == BOOT_BANK_A)
    {
        expected_crc = boot_metadata.bank_a_crc;
        image_size = boot_metadata.bank_a_size;
    }
    else if (bank == BOOT_BANK_B)
    {
        expected_crc = boot_metadata.bank_b_crc;
        image_size = boot_metadata.bank_b_size;
    }
    else
    {
        return 0;
    }

    bank_start = Bootloader_GetBankStartAddress(bank);
    computed_crc = Bootloader_ComputeCRC32(bank_start, image_size);

    return (computed_crc == expected_crc) ? 1 : 0;
}

static void Bootloader_InvalidateBankMetadata(uint8_t bank)
{
    uint8_t changed = 0;

    if (!metadata_ready)
    {
        return;
    }

    if (bank == BOOT_BANK_A)
    {
        if (boot_metadata.bank_a_valid || boot_metadata.bank_a_crc || boot_metadata.bank_a_size)
        {
            boot_metadata.bank_a_valid = 0;
            boot_metadata.bank_a_crc = 0;
            boot_metadata.bank_a_size = 0;
            bank_a_crc_ok = 0;
            changed = 1;
        }
    }
    else if (bank == BOOT_BANK_B)
    {
        if (boot_metadata.bank_b_valid || boot_metadata.bank_b_crc || boot_metadata.bank_b_size)
        {
            boot_metadata.bank_b_valid = 0;
            boot_metadata.bank_b_crc = 0;
            boot_metadata.bank_b_size = 0;
            bank_b_crc_ok = 0;
            changed = 1;
        }
    }

    if (changed)
    {
        (void)Bootloader_WriteMetadata(&boot_metadata);
        Bootloader_LED_UpdateStatus();
    }
}

/**
  * @brief  Initialize the bootloader
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void Bootloader_Init(CAN_HandleTypeDef *hcan)
{
    uint8_t metadata_valid;

    hcan_bootloader = hcan;
    
    /* Initialize bootloader status */
    memset(&bootloader_status, 0, sizeof(BootloaderStatus_t));
    bootloader_status.state = BL_STATE_IDLE;
    bootloader_status.current_address = BANK_A_ADDRESS;

    metadata_valid = Bootloader_ReadMetadata(&boot_metadata);
    if (!metadata_valid)
    {
        Bootloader_SetDefaultMetadata(&boot_metadata);
        /* Bank validity is host-authoritative and may only be set after
         * explicit verified update flow (CMD_SET_IMAGE_INFO/CMD_VERIFY_BANK/CMD_JUMP_TO_APP).
         * Do not infer validity from flash contents at startup. */
        Bootloader_WriteMetadata(&boot_metadata);
    }
    metadata_ready = 1;

    /* Validate stored CRC for each bank at startup.
     * Performed even if a bank is marked invalid, so the host can see
     * whether flash contents still match a previously-stored CRC. */
    if (boot_metadata.bank_a_size > 0 && boot_metadata.bank_a_size <= BANK_SIZE)
    {
        uint32_t crc = Bootloader_ComputeCRC32(BANK_A_ADDRESS, boot_metadata.bank_a_size);
        bank_a_crc_ok = (crc == boot_metadata.bank_a_crc) ? 1 : 0;
    }
    if (boot_metadata.bank_b_size > 0 && boot_metadata.bank_b_size <= BANK_SIZE)
    {
        uint32_t crc = Bootloader_ComputeCRC32(BANK_B_ADDRESS, boot_metadata.bank_b_size);
        bank_b_crc_ok = (crc == boot_metadata.bank_b_crc) ? 1 : 0;
    }
    
    /* Configure CAN filter */
    Bootloader_ConfigureCANFilter();
    
    /* Start CAN peripheral */
    if (HAL_CAN_Start(hcan_bootloader) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(hcan_bootloader, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Configure TX header */
    tx_header.StdId = 0;
    tx_header.ExtId = CAN_BOOTLOADER_ID;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    /* Initialize LED debug indicators */
    Bootloader_LED_Init();
    
    /* Update LED status to show application validity */
    Bootloader_LED_UpdateStatus();
}

/**
  * @brief  Configure CAN filter to accept bootloader messages
  * @retval None
  */
static void Bootloader_ConfigureCANFilter(void)
{
    CAN_FilterTypeDef can_filter;
    
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    
    /* Accept ONLY the host CAN ID (CAN_HOST_ID) with exact 29-bit match.
     * For 32-bit filter scale with extended IDs, the register layout is:
     *   Bits [31:3] = 29-bit Extended ID
     *   Bit  [2]    = IDE (1 = extended frame)
     *   Bit  [1]    = RTR
     *   Bit  [0]    = 0
     * This prevents other CAN bus traffic from filling the 3-deep hardware FIFO. */
    uint32_t filter_id   = (CAN_HOST_ID << 3) | 0x04;        /* IDE bit set */
    uint32_t filter_mask = (0x1FFFFFFFUL << 3) | 0x04;       /* Match all 29 ID bits + IDE */
    
    can_filter.FilterIdHigh   = (filter_id >> 16) & 0xFFFF;
    can_filter.FilterIdLow    = filter_id & 0xFFFF;
    can_filter.FilterMaskIdHigh = (filter_mask >> 16) & 0xFFFF;
    can_filter.FilterMaskIdLow  = filter_mask & 0xFFFF;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(hcan_bootloader, &can_filter) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Main bootloader loop
  * @retval None
  */
void Bootloader_Main(void)
{
    uint32_t timeout_start = HAL_GetTick();
    uint32_t last_heartbeat = HAL_GetTick();
    uint32_t last_led_toggle = HAL_GetTick();
    uint8_t timeout_expired = 0;
    
    /* Initialize the static heartbeat time variable */
    last_heartbeat_time = last_heartbeat;
    
    /* Send startup diagnostic heartbeat frame. */
    Bootloader_FillReadyHeartbeatPayload(0x01);
    Bootloader_SendCANMessage(RESP_READY, tx_data, 8);
    
    /* Main bootloader loop with timeout */
    while (1)
    {
        /* Rapid LED flash to indicate bootloader mode (100ms toggle = 5Hz blink) */
        if ((HAL_GetTick() - last_led_toggle) >= 100)
        {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
            last_led_toggle = HAL_GetTick();
        }
        
        /* Update local heartbeat from static variable (may be reset by command processing) */
        last_heartbeat = last_heartbeat_time;
        
        /* Send heartbeat message periodically only when idle */
        if (bootloader_status.state == BL_STATE_IDLE && 
            (HAL_GetTick() - last_heartbeat) >= HEARTBEAT_INTERVAL_MS)
        {
            /* Send periodic diagnostic heartbeat. */
            Bootloader_FillReadyHeartbeatPayload(0x01);
            Bootloader_SendCANMessage(RESP_READY, tx_data, 8);
            last_heartbeat = HAL_GetTick();
            last_heartbeat_time = last_heartbeat;  /* Update static variable */
        }
        
        /* Check for timeout to auto-jump to application */
        if (!timeout_expired && !can_command_received &&
            (HAL_GetTick() - timeout_start >= BOOTLOADER_TIMEOUT_MS))
        {
            timeout_expired = 1;

            uint8_t active_bank = Bootloader_GetActiveBank();
            uint8_t fallback_bank = Bootloader_GetInactiveBank();

            if (Bootloader_IsUpdateInProgress())
            {
                /* Interrupted update recovery: only boot the currently active bank,
                 * and only if metadata/CRC/vector checks all pass. */
                if (Bootloader_IsBankMarkedValid(active_bank) &&
                    Bootloader_IsBankCrcValid(active_bank) &&
                    Bootloader_CheckValidApplication(active_bank))
                {
                    Bootloader_FillReadyHeartbeatPayload(0xA5);
                    Bootloader_SendCANMessage(RESP_READY, tx_data, 8);
                    Bootloader_WaitForCANTransmission();
                    HAL_Delay(10);
                    Bootloader_LED_BlinkBeforeJump();
                    Bootloader_JumpToBank(active_bank);
                }

                /* If active bank is not valid/verified, stay in bootloader for recovery. */
                continue;
            }

            if (Bootloader_CheckValidApplication(active_bank))
            {
                /* Send timeout jump diagnostic frame. */
                Bootloader_FillReadyHeartbeatPayload(0xAA);
                Bootloader_SendCANMessage(RESP_READY, tx_data, 8);

                /* Wait for message to send */
                Bootloader_WaitForCANTransmission();
                HAL_Delay(10);

                /* Blink LEDs 5 times rapidly before jumping */
                Bootloader_LED_BlinkBeforeJump();

                /* Jump to application */
                Bootloader_JumpToBank(active_bank);
            }
            else if (Bootloader_CheckValidApplication(fallback_bank))
            {
                /* Allow fallback boot for recoverability, but do not mutate
                 * validity metadata outside the verified flashing flow. */
                Bootloader_LED_BlinkBeforeJump();
                Bootloader_JumpToBank(fallback_bank);
            }
            /* If no valid app, stay in bootloader mode indefinitely */
        }
        
        /* Check if we should jump to application (from CAN command) */
        if (jump_to_app_flag)
        {
            /* Send pre-jump diagnostic frame. */
            Bootloader_FillReadyHeartbeatPayload(0xFF);
            Bootloader_SendCANMessage(RESP_READY, tx_data, 8);
            
            /* Wait for this message to be sent */
            Bootloader_WaitForCANTransmission();
            
            /* Blink LEDs 5 times rapidly before jumping */
            Bootloader_LED_BlinkBeforeJump();
            
            /* Jump to application (this function should not return) */
            if (jump_target_bank == BOOT_BANK_INVALID)
            {
                jump_target_bank = Bootloader_GetActiveBank();
            }
            Bootloader_JumpToBank(jump_target_bank);
            
            /* If we get here, jump failed - reset the flag */
            jump_to_app_flag = 0;
        }
        
        /* Process received CAN messages (deferred from interrupt to avoid
         * blocking ISR during slow flash writes on a busy bus) */
        if (rx_msg_pending)
        {
            Bootloader_ProcessCANMessage();
            rx_msg_pending = 0;
        }
    }
}

/**
  * @brief  Process received CAN message
  * @retval None
  */
void Bootloader_ProcessCANMessage(void)
{
    uint8_t command;
    uint32_t address;
    uint16_t length;
    uint8_t result;
    uint8_t active_bank;
    uint8_t inactive_bank;

    /* Mark that a CAN command has been received - disable auto-jump timeout */
    can_command_received = 1;
    
    /* Turn on LED3 to indicate CAN activity from host */
    LED3_ON();
    
    /* Reset heartbeat timer to prevent heartbeat from being sent immediately after command */
    last_heartbeat_time = HAL_GetTick();

    /* Get the command byte */
    command = rx_data[0];
    active_bank = Bootloader_GetActiveBank();
    inactive_bank = Bootloader_GetInactiveBank();

    switch (command)
    {
        case CMD_GET_STATUS:
            /* Send status information */
            tx_data[0] = RESP_DATA;
            tx_data[1] = bootloader_status.state;
            tx_data[2] = bootloader_status.last_error;
            tx_data[3] = (bootloader_status.bytes_written >> 24) & 0xFF;
            tx_data[4] = (bootloader_status.bytes_written >> 16) & 0xFF;
            tx_data[5] = (bootloader_status.bytes_written >> 8) & 0xFF;
            tx_data[6] = bootloader_status.bytes_written & 0xFF;
            Bootloader_SendCANMessage(RESP_DATA, tx_data, 8);
            break;
            
        case CMD_ERASE_FLASH:
            /* Erase inactive application bank */
            bootloader_status.state = BL_STATE_ERASING;

            result = Bootloader_EraseBank(inactive_bank);
            if (result == ERR_NONE)
            {
                bootloader_status.bytes_written = 0;
                bootloader_status.current_address = Bootloader_GetBankStartAddress(inactive_bank);
                pending_image_info_valid = 0;
                pending_verified_bank = BOOT_BANK_INVALID;
                Bootloader_SendACK();
            }
            else
            {
                Bootloader_SendNACK(result);
            }
            bootloader_status.state = BL_STATE_IDLE;
            break;
            
        case CMD_SET_ADDRESS:
            /* Set the current write address */
            address = (rx_data[1] << 24) | (rx_data[2] << 16) |
                     (rx_data[3] << 8) | rx_data[4];

            if (Bootloader_IsAddressInBank(address, 1, inactive_bank))
            {
                bootloader_status.current_address = address;
                buffer_index = 0;  /* Reset buffer */
                Bootloader_SendACK();
            }
            else
            {
                Bootloader_SendNACK(ERR_INVALID_ADDRESS);
            }
            break;
            
        case CMD_WRITE_DATA:
            /* Write data to flash at current address */
            /* Expected format: [CMD] [0x04] [byte0] [byte1] [byte2] [byte3] ... */
            /* Receives 4 bytes per message, buffers two messages (8 bytes) before writing */
            bootloader_status.state = BL_STATE_WRITING;
            length = rx_data[1];  /* Data length - should always be 4 */
            
            if (length != 4)
            {
                Bootloader_SendNACK(ERR_INVALID_DATA_LENGTH);
                bootloader_status.state = BL_STATE_IDLE;
                buffer_index = 0;  /* Reset buffer on error */
                break;
            }
            
            /* Copy 4 bytes to buffer */
            for (uint8_t i = 0; i < 4; i++)
            {
                flash_buffer[buffer_index++] = rx_data[2 + i];
            }
            
            /* Write to flash when we have 8 bytes (two 4-byte chunks) */
            if (buffer_index >= 8)
            {
                result = Bootloader_WriteFlash(bootloader_status.current_address, 
                                               flash_buffer, 8);
                if (result == ERR_NONE)
                {
                    bootloader_status.current_address += 8;
                    bootloader_status.bytes_written += 8;
                    buffer_index = 0;
                    Bootloader_SendACK();
                }
                else
                {
                    buffer_index = 0;  /* Reset buffer on write failure */
                    Bootloader_SendNACK(result);
                }
            }
            else
            {
                Bootloader_SendACK();  /* Data buffered, waiting for more */
            }
            
            bootloader_status.state = BL_STATE_IDLE;
            break;
            
        case CMD_WRITE_FLASH:
            /* Legacy write command - write address and data in one message */
            address = (rx_data[1] << 24) | (rx_data[2] << 16) |
                     (rx_data[3] << 8) | rx_data[4];
            length = rx_data[5];

            /* Validate address is in application area (not in permanent storage) */
            if (Bootloader_IsAddressInBank(address, length, inactive_bank))
            {
                bootloader_status.state = BL_STATE_WRITING;
                result = Bootloader_WriteFlash(address, &rx_data[6], length);
                if (result == ERR_NONE)
                {
                    Bootloader_SendACK();
                }
                else
                {
                    Bootloader_SendNACK(result);
                }
                bootloader_status.state = BL_STATE_IDLE;
            }
            else
            {
                Bootloader_SendNACK(ERR_INVALID_ADDRESS);
            }
            break;
            
        case CMD_READ_FLASH:
            /* Read flash memory */
            address = (rx_data[1] << 24) | (rx_data[2] << 16) |
                     (rx_data[3] << 8) | rx_data[4];
            length = rx_data[5];

            if (Bootloader_IsAddressInAnyBank(address, length) && length <= 7)
            {
                bootloader_status.state = BL_STATE_READING;
                tx_data[0] = RESP_DATA;
                result = Bootloader_ReadFlash(address, &tx_data[1], length);
                if (result == ERR_NONE)
                {
                    Bootloader_SendCANMessage(RESP_DATA, tx_data, length + 1);
                }
                else
                {
                    Bootloader_SendNACK(result);
                }
                bootloader_status.state = BL_STATE_IDLE;
            }
            else
            {
                Bootloader_SendNACK(ERR_INVALID_ADDRESS);
            }
            break;

        case CMD_GET_ACTIVE_BANK:
            tx_data[0] = RESP_DATA;
            tx_data[1] = active_bank;
            tx_data[2] = boot_metadata.bank_a_valid;
            tx_data[3] = boot_metadata.bank_b_valid;
            tx_data[4] = (BANK_SIZE >> 16) & 0xFF;
            tx_data[5] = (BANK_SIZE >> 8) & 0xFF;
            tx_data[6] = BANK_SIZE & 0xFF;
            tx_data[7] = 1; /* Metadata format version */
            Bootloader_SendCANMessage(RESP_DATA, tx_data, 8);
            break;

        case CMD_SET_IMAGE_INFO:
            pending_image_crc = ((uint32_t)rx_data[1] << 24) |
                                ((uint32_t)rx_data[2] << 16) |
                                ((uint32_t)rx_data[3] << 8) |
                                 (uint32_t)rx_data[4];
            pending_image_size = ((uint32_t)rx_data[5] << 16) |
                                 ((uint32_t)rx_data[6] << 8) |
                                  (uint32_t)rx_data[7];

            if (pending_image_size == 0 || pending_image_size > BANK_SIZE)
            {
                pending_image_info_valid = 0;
                pending_verified_bank = BOOT_BANK_INVALID;
                Bootloader_SendNACK(ERR_INVALID_DATA_LENGTH);
            }
            else
            {
                pending_image_info_valid = 1;
                pending_verified_bank = BOOT_BANK_INVALID;
                Bootloader_SendACK();
            }
            break;

        case CMD_VERIFY_BANK:
            if (!pending_image_info_valid)
            {
                Bootloader_SendNACK(ERR_INVALID_DATA_LENGTH);
                break;
            }

            bootloader_status.state = BL_STATE_VERIFYING;
            {
                uint32_t verify_start = Bootloader_GetBankStartAddress(inactive_bank);
                uint32_t computed_crc = Bootloader_ComputeCRC32(verify_start, pending_image_size);

                if (computed_crc == pending_image_crc)
                {
                    pending_verified_bank = inactive_bank;
                    Bootloader_SendACK();
                }
                else
                {
                    pending_verified_bank = BOOT_BANK_INVALID;
                    Bootloader_SendNACK(ERR_CRC_MISMATCH);
                }
            }
            bootloader_status.state = BL_STATE_IDLE;
            break;
            
        case CMD_JUMP_TO_APP:
            /* Jump to application - set flag to jump from main loop, not from interrupt */
            jump_target_bank = active_bank;

            if (pending_image_info_valid && pending_verified_bank == inactive_bank)
            {
                boot_metadata.active_bank = inactive_bank;
                if (inactive_bank == BOOT_BANK_A)
                {
                    boot_metadata.bank_a_valid = 1;
                    boot_metadata.bank_a_crc = pending_image_crc;
                    boot_metadata.bank_a_size = pending_image_size;
                    boot_metadata.bank_b_valid = 0;
                    boot_metadata.bank_b_crc = 0;
                    boot_metadata.bank_b_size = 0;
                    bank_a_crc_ok = 1;
                    bank_b_crc_ok = 0;
                }
                else
                {
                    boot_metadata.bank_b_valid = 1;
                    boot_metadata.bank_b_crc = pending_image_crc;
                    boot_metadata.bank_b_size = pending_image_size;
                    boot_metadata.bank_a_valid = 0;
                    boot_metadata.bank_a_crc = 0;
                    boot_metadata.bank_a_size = 0;
                    bank_b_crc_ok = 1;
                    bank_a_crc_ok = 0;
                }

                boot_metadata.reserved0 = BOOT_METADATA_UPDATE_IDLE;

                result = Bootloader_WriteMetadata(&boot_metadata);
                if (result != ERR_NONE)
                {
                    Bootloader_SendNACK(result);
                    break;
                }

                jump_target_bank = inactive_bank;
                pending_image_info_valid = 0;
                pending_verified_bank = BOOT_BANK_INVALID;
            }

            if (Bootloader_CheckValidApplication(jump_target_bank))
            {
                /* Update LED2 to show valid application */
                Bootloader_LED_UpdateStatus();
                
                Bootloader_SendACK();
                jump_to_app_flag = 1;  /* Set flag to trigger jump from main loop */
            }
            else
            {
                Bootloader_SendNACK(ERR_NO_VALID_APP);
            }
            break;
            
        default:
            /* Unknown command */
            Bootloader_SendNACK(ERR_INVALID_COMMAND);
            break;
    }
}

/**
    * @brief  Erase selected application bank
  * @retval Error code
  */
static uint8_t Bootloader_EraseBank(uint8_t bank)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    HAL_StatusTypeDef status;
        uint32_t bank_start = Bootloader_GetBankStartAddress(bank);
        uint32_t first_page = (bank_start - 0x08000000u) / FLASH_PAGE_SIZE;
        uint32_t num_pages = BANK_SIZE / FLASH_PAGE_SIZE;
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
    
    /* Clear all flash flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    
    /* Erase pages one at a time for better responsiveness */
    for (uint32_t page = 0; page < num_pages; page++)
    {
        /* Configure erase for single page */
        erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
        erase_init.Banks = FLASH_BANK_1;
        erase_init.Page = first_page + page;
        erase_init.NbPages = 1;
        
        /* Perform erase */
        status = HAL_FLASHEx_Erase(&erase_init, &page_error);
        
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            bootloader_status.last_error = ERR_FLASH_ERASE_FAILED;
            return ERR_FLASH_ERASE_FAILED;
        }
    }
    
    /* Lock flash */
    HAL_FLASH_Lock();

    boot_metadata.reserved0 = BOOT_METADATA_UPDATE_IN_PROGRESS;

    if (bank == BOOT_BANK_A)
    {
        boot_metadata.bank_a_valid = 0;
        boot_metadata.bank_a_crc = 0;
        boot_metadata.bank_a_size = 0;
        bank_a_crc_ok = 0;
    }
    else
    {
        boot_metadata.bank_b_valid = 0;
        boot_metadata.bank_b_crc = 0;
        boot_metadata.bank_b_size = 0;
        bank_b_crc_ok = 0;
    }

    if (Bootloader_WriteMetadata(&boot_metadata) != ERR_NONE)
    {
        bootloader_status.last_error = ERR_FLASH_WRITE_FAILED;
        return ERR_FLASH_WRITE_FAILED;
    }

    Bootloader_LED_UpdateStatus();
    
    bootloader_status.last_error = ERR_NONE;
    return ERR_NONE;
}

/**
  * @brief  Write data to flash
  * @param  address: Flash address to write
  * @param  data: Pointer to data buffer
  * @param  length: Number of bytes to write (must be multiple of 8)
  * @retval Error code
  */
static uint8_t Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t i;
    uint64_t data64;

    /* Writes are allowed only in the inactive bank */
    if (!Bootloader_IsAddressInBank(address, length, Bootloader_GetInactiveBank()))
    {
        bootloader_status.last_error = ERR_INVALID_ADDRESS;
        return ERR_INVALID_ADDRESS;
    }
    
    /* Length must be multiple of 8 for double word programming */
    if (length % 8 != 0)
    {
        bootloader_status.last_error = ERR_INVALID_DATA_LENGTH;
        return ERR_INVALID_DATA_LENGTH;
    }
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
    
    /* Clear all flash flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    
    /* Write data in 64-bit chunks */
    for (i = 0; i < length; i += 8)
    {
        /* Combine 8 bytes into 64-bit value */
        data64 = ((uint64_t)data[i + 0] << 0)  |
                 ((uint64_t)data[i + 1] << 8)  |
                 ((uint64_t)data[i + 2] << 16) |
                 ((uint64_t)data[i + 3] << 24) |
                 ((uint64_t)data[i + 4] << 32) |
                 ((uint64_t)data[i + 5] << 40) |
                 ((uint64_t)data[i + 6] << 48) |
                 ((uint64_t)data[i + 7] << 56);
        
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data64);
        
        if (status != HAL_OK)
        {
            break;
        }
    }
    
    /* Lock flash */
    HAL_FLASH_Lock();
    
    if (status != HAL_OK)
    {
        bootloader_status.last_error = ERR_FLASH_WRITE_FAILED;
        return ERR_FLASH_WRITE_FAILED;
    }
    
    bootloader_status.last_error = ERR_NONE;
    return ERR_NONE;
}

/**
  * @brief  Read data from flash
  * @param  address: Flash address to read
  * @param  data: Pointer to data buffer
  * @param  length: Number of bytes to read
  * @retval Error code
  */
static uint8_t Bootloader_ReadFlash(uint32_t address, uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint8_t *flash_ptr = (uint8_t *)address;

    if (!Bootloader_IsAddressInAnyBank(address, length))
    {
        return ERR_INVALID_ADDRESS;
    }
    
    /* Read data */
    for (i = 0; i < length; i++)
    {
        data[i] = flash_ptr[i];
    }
    
    return ERR_NONE;
}

/**
  * @brief  Check if valid application exists
  * @retval 1 if valid application, 0 otherwise
  */
uint8_t Bootloader_CheckValidApplication(uint8_t bank)
{
    uint32_t app_base = Bootloader_GetBankStartAddress(bank);
    uint32_t stack_pointer = *(__IO uint32_t *)app_base;
    uint32_t reset_handler = *(__IO uint32_t *)(app_base + 4);
    
    /* Check if stack pointer is in valid RAM range */
    if ((stack_pointer & 0xFFF00000) != 0x20000000)
    {
        return 0;
    }
    
    /* Check if reset handler is in valid flash range (odd address for Thumb) */
    if ((reset_handler & 0xFF000000) != 0x08000000 || (reset_handler & 0x01) == 0)
    {
        return 0;
    }
    
    return 1;
}

/**
  * @brief  De-initialize peripherals before jumping to application
  * @retval None
  */
static void Bootloader_DeInit(void)
{
    /* Prevent new interrupts while tearing down the bootloader context. */
    __disable_irq();

    /* Turn off and de-initialize LED */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_DeInit(LED_PORT, LED_PIN);
    
    /* Disable CAN */
    HAL_CAN_DeactivateNotification(hcan_bootloader, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Stop(hcan_bootloader);
    HAL_CAN_MspDeInit(hcan_bootloader);
    
    /* Disable SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* Disable and clear all NVIC interrupts to mimic a reset-like handoff. */
    for (uint32_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    /* Reset clock tree / HAL state before handing control to the application. */
    HAL_RCC_DeInit();
    HAL_DeInit();
}

/**
  * @brief  Wait for all CAN transmissions to complete
  * @retval None
  */
static void Bootloader_WaitForCANTransmission(void)
{
    uint32_t timeout = 0;
    
    /* Wait until all 3 TX mailboxes are free (all transmissions complete) */
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_bootloader) != 3 && timeout < 1000000)
    {
        timeout++;
    }
    
    /* Additional delay to ensure message physically leaves the CAN controller */
    HAL_Delay(10);
}

/**
  * @brief  Jump to application
  * @retval Error code (if returns, jump failed)
  */
static uint8_t Bootloader_JumpToBank(uint8_t bank)
{
    typedef void (*pFunction)(void);
    pFunction jump_to_application;
    uint32_t jump_address;
    uint32_t app_stack_pointer;
    uint8_t jump_info[8];
    
    /* Check if valid application exists */
    uint32_t app_base = Bootloader_GetBankStartAddress(bank);

    if (!Bootloader_IsBankMarkedValid(bank))
    {
        return ERR_NO_VALID_APP;
    }

    if (!Bootloader_IsBankCrcValid(bank))
    {
        Bootloader_InvalidateBankMetadata(bank);
        return ERR_CRC_MISMATCH;
    }

    if (!Bootloader_CheckValidApplication(bank))
    {
        Bootloader_InvalidateBankMetadata(bank);
        return ERR_NO_VALID_APP;
    }
    
    /* Read application vector table */
    app_stack_pointer = *(__IO uint32_t *)app_base;
    jump_address = *(__IO uint32_t *)(app_base + 4);
    
    /* Send detailed jump information before jumping */
    jump_info[0] = RESP_JUMP_INFO;
    jump_info[1] = (app_stack_pointer >> 24) & 0xFF;  /* Stack pointer MSB */
    jump_info[2] = (app_stack_pointer >> 16) & 0xFF;
    jump_info[3] = (app_stack_pointer >> 8) & 0xFF;
    jump_info[4] = app_stack_pointer & 0xFF;          /* Stack pointer LSB */
    jump_info[5] = (jump_address >> 16) & 0xFF;       /* Reset vector high word */
    jump_info[6] = (jump_address >> 8) & 0xFF;
    jump_info[7] = jump_address & 0xFF;               /* Reset vector LSB */
    
    Bootloader_SendCANMessage(RESP_JUMP_INFO, jump_info, 8);
    
    /* Wait for CAN transmission to complete */
    Bootloader_WaitForCANTransmission();
    
    bootloader_status.state = BL_STATE_JUMPING;
    
    /* De-initialize peripherals */
    Bootloader_DeInit();
    
    /* Get the application stack pointer */
    __set_CONTROL(0);
    __set_MSP(app_stack_pointer);
    
    /* Cast to function pointer */
    jump_to_application = (pFunction)jump_address;
    
    /* Relocate vector table */
    SCB->VTOR = app_base;

    __DSB();
    __ISB();

    /* Enter the application with interrupts unmasked; startup code can manage them. */
    __enable_irq();
    
    /* Jump to application */
    jump_to_application();
    
    /* Should never reach here */
    return ERR_NONE;
}

uint8_t Bootloader_JumpToApplication(void)
{
    return Bootloader_JumpToBank(Bootloader_GetActiveBank());
}

/**
  * @brief  Send ACK message
  * @retval None
  */
static void Bootloader_SendACK(void)
{
    tx_data[0] = RESP_ACK;
    tx_data[1] = bootloader_status.last_error;
    Bootloader_SendCANMessage(RESP_ACK, tx_data, 2);
}

/**
  * @brief  Send NACK message
  * @param  error_code: Error code to send
  * @retval None
  */
static void Bootloader_SendNACK(uint8_t error_code)
{
    bootloader_status.last_error = error_code;
    tx_data[0] = RESP_NACK;
    tx_data[1] = error_code;
    Bootloader_SendCANMessage(RESP_NACK, tx_data, 2);
}

/**
  * @brief  Send CAN message
  * @param  cmd: Command/response byte
  * @param  data: Pointer to data buffer
  * @param  length: Data length
  * @retval None
  */
void Bootloader_SendCANMessage(uint8_t cmd, uint8_t *data, uint8_t length)
{
        (void)cmd;

    uint8_t i;
    uint8_t tx_buffer[8] = {0};
    uint32_t timeout;
    
    /* Copy data to TX buffer */
    for (i = 0; i < length && i < 8; i++)
    {
        tx_buffer[i] = data[i];
    }
    
    /* Set DLC */
    tx_header.DLC = length;
    
    /* Wait for a free TX mailbox (busy bus can stall all 3 mailboxes) */
    timeout = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_bootloader) == 0 && timeout < 1000000)
    {
        timeout++;
    }
    
    /* Send message */
    HAL_CAN_AddTxMessage(hcan_bootloader, &tx_header, tx_buffer, &tx_mailbox);
}

/**
  * @brief  Initialize LED debug indicators
  * @retval None
  */
void Bootloader_LED_Init(void)
{
    /* Turn on LED1 to indicate bootloader is running */
    LED1_ON();
    
    /* LED2 and LED3 start off */
    LED2_OFF();
    LED3_OFF();
}

/**
  * @brief  Update LED2 based on valid application status
  * @retval None
  */
void Bootloader_LED_UpdateStatus(void)
{
    uint8_t active_bank = Bootloader_GetActiveBank();
    uint8_t valid_active = (active_bank == BOOT_BANK_A) ? boot_metadata.bank_a_valid : boot_metadata.bank_b_valid;

    if (valid_active)
    {
        LED2_ON();
    }
    else
    {
        LED2_OFF();
    }
}

/**
  * @brief  Not used - kept for compatibility
  * @retval None
  */
void Bootloader_LED_FlashingStart(void)
{
    /* No-op: LED3 is controlled directly in ProcessCANMessage */
}

/**
  * @brief  Not used - kept for compatibility
  * @retval None
  */
void Bootloader_LED_FlashingStop(void)
{
    /* No-op: LED3 stays on once CAN activity detected */
}

/**
  * @brief  Not used - kept for compatibility
  * @retval None
  */
void Bootloader_LED_FlashingUpdate(void)
{
    /* No-op: LED3 is controlled directly, no updates needed */
}

/**
  * @brief  Blink all LEDs rapidly 20 times before jumping to application
  * @retval None
  */
void Bootloader_LED_BlinkBeforeJump(void)
{
    for (uint8_t i = 0; i < 20; i++)
    {
        LED1_OFF();
        HAL_Delay(5);
        LED2_ON();
        HAL_Delay(10);
        LED3_OFF();
        HAL_Delay(15); 
        
        LED1_ON();
        HAL_Delay(15);
        LED2_OFF();
        HAL_Delay(10);
        LED3_ON();
        HAL_Delay(5); 
    }
    
    /* Turn all LEDs off before jumping */
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
}

/**
  * @brief  CAN RX FIFO 0 message pending callback
  * @param  hcan: pointer to CAN Handle
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef isr_rx_header;
    uint8_t isr_rx_data[8];

    if (hcan == hcan_bootloader)
    {
        /* Drain message from hardware FIFO immediately (prevents FIFO overflow) */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &isr_rx_header, isr_rx_data) == HAL_OK)
        {
            /* Check if message is for bootloader (using ExtId for 29-bit extended ID) */
            if (isr_rx_header.ExtId == CAN_HOST_ID && isr_rx_header.IDE == CAN_ID_EXT)
            {
                /* Queue for main loop processing only if previous message was consumed.
                 * This avoids slow flash operations blocking the ISR on a busy bus. */
                if (!rx_msg_pending)
                {
                    memcpy(rx_data, isr_rx_data, 8);
                    rx_msg_pending = 1;
                }
            }
        }
    }
}
