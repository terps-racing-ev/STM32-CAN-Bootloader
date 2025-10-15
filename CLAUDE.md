# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a CAN-based bootloader for STM32L432 microcontrollers that enables firmware updates over CAN bus. The bootloader occupies the first 32KB of flash memory and can flash applications to the remaining 224KB.

## Build System

### Building the Bootloader

```bash
make clean
make all
```

This generates:
- `build/CAN-Bootloader-TEST - Copy.elf`
- `build/CAN-Bootloader-TEST - Copy.hex`
- `build/CAN-Bootloader-TEST - Copy.bin`

The build uses ARM GCC toolchain (`arm-none-eabi-gcc`) configured in the Makefile. Target MCU is STM32L432KC (Cortex-M4 with FPU).

### Flashing Firmware via CAN

Use the Python flashing script:

```bash
python Flash_Application.py application.bin
```

Optional arguments:
- `--channel USB1` - Specify PCAN channel (default: USB1)
- `--no-verify` - Skip read-back verification
- `--no-jump` - Don't jump to application after flashing
- `--status-only` - Only query bootloader status
- `--list-devices` - List available PCAN devices

The script requires:
- PCAN-USB adapter
- `PCAN_Driver.py` module
- `python-can` library

## Memory Architecture

### Flash Layout

- **Bootloader Region**: `0x08000000 - 0x08007FFF` (32KB)
- **Application Region**: `0x08008000 - 0x0803BFFF` (208KB)
- **Permanent Storage**: `0x0803C000 - 0x0803FFFF` (16KB, reserved - never erased by bootloader)
- **Valid Flag Address**: `0x08007FF8` (stored at end of bootloader region)

The linker script [STM32L432XX_FLASH.ld](STM32L432XX_FLASH.ld) constrains the bootloader to 32KB. Applications must use a modified linker script that places code starting at `0x08008000` with a maximum length of `0x34000` (208KB).

**Important**: The last 16KB of flash is reserved for permanent data storage and is **protected by the bootloader**. Attempts to write or read from this area during firmware updates will be rejected with `ERR_INVALID_ADDRESS`.

### Application Requirements

Applications built to run with this bootloader MUST:

1. **Linker Script**: Set FLASH origin to `0x08008000` and length to `0x34000` (208KB)
2. **Vector Table**: Place `.isr_vector` at `0x08008000`
3. **VTOR Register**: Set `SCB->VTOR = 0x08008000` in `SystemInit()` or early in `main()`
4. **Permanent Storage Access**: Applications can access the permanent storage area at `0x0803C000` directly for read/write operations (not managed by bootloader)

### Valid Application Flag

The bootloader uses a dual-word magic number at `0x08007FF8`:
- Magic: `0xDEADBEEF`
- Complement: `0x21524110`

When both values are present, the bootloader will automatically jump to the application after a 10-second timeout. This flag is set when a successful firmware flash completes with the jump command.

## Code Architecture

### Bootloader State Machine

The bootloader operates in several states defined in [bootloader.h](Core/Inc/bootloader.h):

- `BL_STATE_IDLE` - Waiting for commands
- `BL_STATE_ERASING` - Erasing application flash
- `BL_STATE_WRITING` - Writing firmware data
- `BL_STATE_READING` - Reading flash for verification
- `BL_STATE_VERIFYING` - Not currently used
- `BL_STATE_JUMPING` - About to jump to application

### Main Flow

1. **Initialization** ([main.c:95](Core/Src/main.c#L95)): `Bootloader_Init()` configures CAN filters and enables RX interrupts
2. **Main Loop** ([bootloader.c:110](Core/Src/bootloader.c#L110)): `Bootloader_Main()` handles:
   - Sending READY message on startup
   - 10-second timeout to auto-jump if valid app exists
   - Processing jump flag set by CAN commands
3. **Message Processing** ([bootloader.c:176](Core/Src/bootloader.c#L176)): `Bootloader_ProcessCANMessage()` handles commands in RX interrupt callback

### CAN Protocol

**CAN IDs** (29-bit extended):
- Host → Bootloader: `0x18000701`
- Bootloader → Host: `0x18000700`

**Command Codes** (first byte):
- `0x01` - Erase flash
- `0x02` - Write flash (legacy, single command)
- `0x03` - Read flash
- `0x04` - Jump to application
- `0x05` - Get status
- `0x06` - Set write address
- `0x07` - Write data (preferred, buffered 4-byte chunks)

**Response Codes**:
- `0x10` - ACK
- `0x11` - NACK (with error code in byte 1)
- `0x14` - READY
- `0x15` - DATA
- `0x16` - JUMP_INFO

### Write Data Buffering

The preferred write method (`CMD_WRITE_DATA`, command `0x07`) uses 4-byte chunks:
- Host sends 4 bytes per CAN message
- Bootloader buffers two messages (8 bytes total)
- Flash programming occurs every 8 bytes (STM32L4 requires 64-bit writes)
- This is more efficient than the legacy single-command write

See [bootloader.c:239-284](Core/Src/bootloader.c#L239-L284) for implementation.

### Flash Operations

**Flash Write** ([bootloader.c:416](Core/Src/bootloader.c#L416)):
- Requires 8-byte aligned data (STM32L4 double-word programming)
- Validates address is in application region
- Unlocks flash, programs in 64-bit chunks, re-locks flash

**Flash Erase** ([bootloader.c:377](Core/Src/bootloader.c#L377)):
- Erases 104 pages of application region (208KB / 2KB per page)
- Does NOT erase the last 16KB (permanent storage area)
- Clears application valid flag before erasing
- Takes several seconds (uses `ERASE_TIMEOUT` of 15 seconds in Python script)

**Flash Read** ([bootloader.c:483](Core/Src/bootloader.c#L483)):
- Direct memory read from flash address
- Used by Python script to verify written data

### Jump to Application

When jumping ([bootloader.c:573](Core/Src/bootloader.c#L573)):
1. Validates application by checking stack pointer and reset handler
2. Sends `RESP_JUMP_INFO` with stack pointer and reset vector values
3. Waits for CAN transmission to complete
4. Deinitializes CAN, SysTick, and GPIO
5. Sets MSP to application's stack pointer
6. Updates VTOR to `0x08008000`
7. Branches to application's reset handler

The jump is triggered from the main loop (not from interrupt context) via the `jump_to_app_flag`.

## Key Source Files

- [Core/Src/bootloader.c](Core/Src/bootloader.c) - Main bootloader logic and CAN protocol implementation
- [Core/Inc/bootloader.h](Core/Inc/bootloader.h) - Bootloader constants and API
- [Core/Src/main.c](Core/Src/main.c) - Hardware initialization and bootloader entry point
- [Flash_Application.py](Flash_Application.py) - Python flashing tool for host PC
- [PCAN_Driver.py](PCAN_Driver.py) - PCAN-USB adapter driver interface
- [STM32L432XX_FLASH.ld](STM32L432XX_FLASH.ld) - Linker script restricting bootloader to 32KB

## CAN Configuration

- **Baud Rate**: 500 kbps
- **Timing**: 48 MHz clock, prescaler 6, BS1=13TQ, BS2=2TQ, SJW=1TQ
- **Filter**: Configured for 29-bit extended ID `0x18000701` in mask mode
- **Hardware**: Uses CAN1 peripheral on STM32L432

Filter configuration is in [bootloader.c:84-104](Core/Src/bootloader.c#L84-L104). Note the bit-shifting required to properly format extended IDs for the STM32 CAN filter registers.

## Recent Changes

- Added support for 29-bit extended CAN IDs (changed from 11-bit standard IDs)
- Implemented 4-byte chunk buffering for efficient flash writes
- Changed bootloader timeout from 5 seconds to 10 seconds
- Added application valid flag mechanism for auto-boot functionality
