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
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
static uint8_t tx_data[8];
static CAN_TxHeaderTypeDef tx_header;
static uint32_t tx_mailbox;
static uint8_t flash_buffer[8];     /* Buffer for 8-byte flash writes (two 4-byte chunks) */
static uint16_t buffer_index = 0;   /* Current position in flash_buffer */
static volatile uint8_t jump_to_app_flag = 0;  /* Flag to trigger jump from main loop */
static volatile uint8_t can_command_received = 0;  /* Flag to disable auto-jump timeout */
static uint32_t last_heartbeat_time = 0;  /* Last heartbeat timestamp for resetting on commands */

/* Private function prototypes -----------------------------------------------*/
static void Bootloader_ConfigureCANFilter(void);
static uint8_t Bootloader_EraseApplicationFlash(void);
static uint8_t Bootloader_WriteFlash(uint32_t address, uint8_t *data, uint16_t length);
static uint8_t Bootloader_ReadFlash(uint32_t address, uint8_t *data, uint16_t length);
static void Bootloader_DeInit(void);
static void Bootloader_SendACK(void);
static void Bootloader_SendNACK(uint8_t error_code);
static void Bootloader_WaitForCANTransmission(void);

/**
  * @brief  Initialize the bootloader
  * @param  hcan: pointer to CAN handle
  * @retval None
  */
void Bootloader_Init(CAN_HandleTypeDef *hcan)
{
    hcan_bootloader = hcan;
    
    /* Initialize bootloader status */
    memset(&bootloader_status, 0, sizeof(BootloaderStatus_t));
    bootloader_status.state = BL_STATE_IDLE;
    bootloader_status.current_address = APPLICATION_ADDRESS;
    
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
    /* For extended IDs: bits [28:13] in FilterIdHigh, bits [12:0] + IDE in FilterIdLow */
    can_filter.FilterIdHigh = (CAN_HOST_ID >> 13) & 0xFFFF;
    can_filter.FilterIdLow = ((CAN_HOST_ID << 3) & 0xFFF8) | 0x0004;  /* IDE bit set */
    can_filter.FilterMaskIdHigh = 0xFFFF;  /* Match all upper bits */
    can_filter.FilterMaskIdLow = 0xFFFC;   /* Match lower bits + IDE */
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
    
    /* Send ready message */
    tx_data[0] = RESP_READY;
    tx_data[1] = 0x01;  /* Bootloader version */
    tx_data[2] = 0x00;  /* Bootloader subversion */
    Bootloader_SendCANMessage(RESP_READY, tx_data, 3);
    
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
            /* Send ready/heartbeat message */
            tx_data[0] = RESP_READY;
            tx_data[1] = 0x01;  /* Bootloader version */
            tx_data[2] = 0x00;  /* Bootloader subversion */
            Bootloader_SendCANMessage(RESP_READY, tx_data, 3);
            last_heartbeat = HAL_GetTick();
            last_heartbeat_time = last_heartbeat;  /* Update static variable */
        }
        
        /* Check for timeout to auto-jump to application */
        /* Only auto-jump if: timeout expired AND no CAN commands received AND valid app exists */
        if (!timeout_expired && !can_command_received &&
            (HAL_GetTick() - timeout_start >= BOOTLOADER_TIMEOUT_MS))
        {
            timeout_expired = 1;

            /* Only jump if we have a valid application flag set */
            if (Bootloader_CheckApplicationValidFlag())
            {
                /* Send timeout jump message */
                tx_data[0] = RESP_READY;
                tx_data[1] = 0xAA;  /* Special: Timeout auto-jump */
                tx_data[2] = 0x55;
                Bootloader_SendCANMessage(RESP_READY, tx_data, 3);

                /* Wait for message to send */
                Bootloader_WaitForCANTransmission();
                HAL_Delay(10);

                /* Blink LEDs 5 times rapidly before jumping */
                Bootloader_LED_BlinkBeforeJump();

                /* Jump to application */
                Bootloader_JumpToApplication();
            }
            /* If no valid app, stay in bootloader mode indefinitely */
        }
        
        /* Check if we should jump to application (from CAN command) */
        if (jump_to_app_flag)
        {
            /* Send a pre-jump indicator message (using READY with special code) */
            tx_data[0] = RESP_READY;
            tx_data[1] = 0xFF;  /* Special: About to jump */
            tx_data[2] = 0xFF;
            Bootloader_SendCANMessage(RESP_READY, tx_data, 3);
            
            /* Wait for this message to be sent */
            Bootloader_WaitForCANTransmission();
            
            /* Blink LEDs 5 times rapidly before jumping */
            Bootloader_LED_BlinkBeforeJump();
            
            /* Jump to application (this function should not return) */
            Bootloader_JumpToApplication();
            
            /* If we get here, jump failed - reset the flag */
            jump_to_app_flag = 0;
        }
        
        /* Process received CAN messages (handled in interrupt) */
        HAL_Delay(1);
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

    /* Mark that a CAN command has been received - disable auto-jump timeout */
    can_command_received = 1;
    
    /* Turn on LED3 to indicate CAN activity from host */
    LED3_ON();
    
    /* Reset heartbeat timer to prevent heartbeat from being sent immediately after command */
    last_heartbeat_time = HAL_GetTick();

    /* Get the command byte */
    command = rx_data[0];

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
            /* Erase application flash area */
            bootloader_status.state = BL_STATE_ERASING;
            
            /* Clear the application valid flag first */
            Bootloader_ClearApplicationValidFlag();
            
            /* Update LED2 since we cleared the valid flag */
            Bootloader_LED_UpdateStatus();
            
            result = Bootloader_EraseApplicationFlash();
            if (result == ERR_NONE)
            {
                bootloader_status.bytes_written = 0;
                bootloader_status.current_address = APPLICATION_ADDRESS;
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

            /* Validate address is in application area (not in permanent storage) */
            if (address >= APPLICATION_ADDRESS && address <= APPLICATION_END_ADDRESS)
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
            if (address >= APPLICATION_ADDRESS && address <= APPLICATION_END_ADDRESS)
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

            /* Allow reading from application area only (not permanent storage during flash ops) */
            if (address >= APPLICATION_ADDRESS && address <= APPLICATION_END_ADDRESS && length <= 7)
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
            
        case CMD_JUMP_TO_APP:
            /* Jump to application - set flag to jump from main loop, not from interrupt */
            if (Bootloader_CheckValidApplication())
            {
                /* Set the valid application flag in flash for future auto-boots */
                result = Bootloader_SetApplicationValidFlag();
                if (result != ERR_NONE)
                {
                    /* Log error but still try to jump - flag write failure shouldn't prevent immediate jump */
                    bootloader_status.last_error = result;
                }
                
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
  * @brief  Erase application flash area
  * @retval Error code
  */
static uint8_t Bootloader_EraseApplicationFlash(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    HAL_StatusTypeDef status;
    
    /* Calculate first and last pages for application */
    uint32_t first_page = (APPLICATION_ADDRESS - 0x08000000) / FLASH_PAGE_SIZE;
    uint32_t num_pages = APPLICATION_SIZE / FLASH_PAGE_SIZE;
    
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

    /* Validate parameters - protect permanent storage area */
    if (address < APPLICATION_ADDRESS || address > APPLICATION_END_ADDRESS)
    {
        bootloader_status.last_error = ERR_INVALID_ADDRESS;
        return ERR_INVALID_ADDRESS;
    }

    /* Ensure write doesn't overflow into permanent storage */
    if ((address + length - 1) > APPLICATION_END_ADDRESS)
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

    /* Validate address - protect permanent storage area */
    if (address < APPLICATION_ADDRESS || address > APPLICATION_END_ADDRESS)
    {
        return ERR_INVALID_ADDRESS;
    }

    /* Ensure read doesn't overflow into permanent storage */
    if ((address + length - 1) > APPLICATION_END_ADDRESS)
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
uint8_t Bootloader_CheckValidApplication(void)
{
    uint32_t stack_pointer = *(__IO uint32_t *)APPLICATION_ADDRESS;
    uint32_t reset_handler = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
    
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
    
    /* Disable all interrupts */
    __disable_irq();
    
    /* Disable all peripheral clocks */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_CAN1_CLK_DISABLE();
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
uint8_t Bootloader_JumpToApplication(void)
{
    typedef void (*pFunction)(void);
    pFunction jump_to_application;
    uint32_t jump_address;
    uint32_t app_stack_pointer;
    uint8_t jump_info[8];
    
    /* Check if valid application exists */
    if (!Bootloader_CheckValidApplication())
    {
        return ERR_NO_VALID_APP;
    }
    
    /* Read application vector table */
    app_stack_pointer = *(__IO uint32_t *)APPLICATION_ADDRESS;
    jump_address = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
    
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
    __set_MSP(app_stack_pointer);
    
    /* Cast to function pointer */
    jump_to_application = (pFunction)jump_address;
    
    /* Relocate vector table */
    SCB->VTOR = APPLICATION_ADDRESS;
    
    /* Jump to application */
    jump_to_application();
    
    /* Should never reach here */
    return ERR_NONE;
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
    uint8_t i;
    uint8_t tx_buffer[8] = {0};
    
    /* Copy data to TX buffer */
    for (i = 0; i < length && i < 8; i++)
    {
        tx_buffer[i] = data[i];
    }
    
    /* Set DLC */
    tx_header.DLC = length;
    
    /* Send message */
    HAL_CAN_AddTxMessage(hcan_bootloader, &tx_header, tx_buffer, &tx_mailbox);
}

/**
  * @brief  Set application valid flag in flash
  * @retval ERR_NONE if successful, error code otherwise
  */
uint8_t Bootloader_SetApplicationValidFlag(void)
{
    HAL_StatusTypeDef status;
    uint32_t magic_data[2];
    
    magic_data[0] = APP_VALID_MAGIC_NUMBER;
    magic_data[1] = APP_VALID_FLAG_COMPLEMENT;
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
    
    /* Erase the page containing the flag (Page 15 - last page of bootloader) */
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = (APP_VALID_FLAG_ADDRESS - 0x08000000) / FLASH_PAGE_SIZE;  /* Page 15: contains APP_VALID_FLAG_ADDRESS (0x08007FF8) */
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return ERR_FLASH_ERASE_FAILED;
    }
    
    /* Write magic number (first 4 bytes) */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_VALID_FLAG_ADDRESS, 
                                *((uint64_t*)magic_data));
    
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return ERR_FLASH_WRITE_FAILED;
    }
    
    /* Lock flash */
    HAL_FLASH_Lock();
    
    return ERR_NONE;
}

/**
  * @brief  Check if application valid flag is set
  * @retval 1 if valid, 0 if not valid
  */
uint8_t Bootloader_CheckApplicationValidFlag(void)
{
    uint32_t *flag_ptr = (uint32_t *)APP_VALID_FLAG_ADDRESS;
    
    /* Check if both magic number and complement are present */
    if (flag_ptr[0] == APP_VALID_MAGIC_NUMBER && 
        flag_ptr[1] == APP_VALID_FLAG_COMPLEMENT)
    {
        return 1;  /* Valid flag found */
    }
    
    return 0;  /* No valid flag */
}

/**
  * @brief  Clear application valid flag
  * @retval None
  */
void Bootloader_ClearApplicationValidFlag(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
    
    /* Erase the page containing the flag */
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = (APP_VALID_FLAG_ADDRESS - 0x08000000) / FLASH_PAGE_SIZE;  /* Page 15: contains APP_VALID_FLAG_ADDRESS */
    erase_init.NbPages = 1;
    
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    /* Lock flash */
    HAL_FLASH_Lock();
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
    /* LED2 indicates if valid application exists */
    if (Bootloader_CheckApplicationValidFlag())
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
    if (hcan == hcan_bootloader)
    {
        /* Get message from FIFO */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            /* Check if message is for bootloader (using ExtId for 29-bit extended ID) */
            if (rx_header.ExtId == CAN_HOST_ID && rx_header.IDE == CAN_ID_EXT)
            {
                /* Process the message */
                Bootloader_ProcessCANMessage();
            }
        }
    }
}
