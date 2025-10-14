# STM32-CAN-Bootloader

CAN Enabled virtual bootloader for STM32L432
============================================

A CAN-based bootloader for STM32L432 microcontrollers, designed for firmware updates over CAN bus (BMS and SDC).

## Memory Layout

- **Bootloader**: `0x08000000 - 0x08007FFF` (32KB)
- **Application**: `0x08008000 - 0x0803FFFF` (224KB)
- **Valid Flag**: `0x08007FF8` (Application validity marker)

## Usage

Use the `Flash_Application.py` script to flash firmware to the device over CAN bus using a PCAN-USB adapter.

Your application should be a *.bin file with the correct changes to its memory locations. (see below)

The Python script will flash the application, then read back the entire contents of the flash to verify flashing was successful, it will then tell the MCU to jump to the application.

## Application Development

When creating an application to run with this bootloader, you must modify the linker script and vector table:

### Linker Script Changes

Update your application's linker script (`.ld` file) to place code in the application region:

```ld
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 0x38000   /* 224KB app region */
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 0x00010000
}

/* Ensure vector table is at application start */
.isr_vector 0x08008000 : ALIGN(4)
{
  KEEP(*(.isr_vector))
} > FLASH
```

### Vector Table Relocation

In your application's startup code or `main()`, set the Vector Table Offset Register (VTOR):

```c
#include "stm32l4xx.h"

void SystemInit(void) {
    /* Relocate vector table to application base */
    SCB->VTOR = 0x08008000U;  /* APPLICATION_ADDRESS */
    /* ...other init... */
}
```

- Application FLASH origin **must** be `0x08008000` (32KB offset from bootloader)
- Vector table **must** be placed at the start of the application region
- VTOR **must** be set to `0x08008000` early in startup
- Do not use addresses below `0x08008000` (bootloader region)

## CAN Protocol

### CAN Message IDs (29-bit Extended IDs)

| Direction          | CAN ID        | Description               |
| ------------------ | ------------- | ------------------------- |
| Host → Bootloader | `0x18000701` | Commands from PC/Host     |
| Bootloader → Host | `0x18000700` | Responses from Bootloader |

### Command Codes (First byte)

| Command             | Code     | Description                   |
| ------------------- | -------- | ----------------------------- |
| `CMD_ERASE_FLASH` | `0x01` | Erase application flash       |
| `CMD_WRITE_DATA`  | `0x07` | Write 4 bytes (buffered to 8) |
| `CMD_READ_FLASH`  | `0x03` | Read data from flash          |
| `CMD_JUMP_TO_APP` | `0x04` | Jump to application           |
| `CMD_GET_STATUS`  | `0x05` | Get bootloader status         |
| `CMD_SET_ADDRESS` | `0x06` | Set write address             |

### Response Codes (First byte)

| Response           | Code     | Description          |
| ------------------ | -------- | -------------------- |
| `RESP_ACK`       | `0x10` | Command acknowledged |
| `RESP_NACK`      | `0x11` | Command rejected     |
| `RESP_READY`     | `0x14` | Bootloader ready     |
| `RESP_DATA`      | `0x15` | Data response        |
| `RESP_JUMP_INFO` | `0x16` | Jump information     |
