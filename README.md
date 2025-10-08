# STM32-CAN-Bootloader
<<<<<<< HEAD
CAN Enabled virtual bootloader for STM32L432 used in BMS and SDC
=======

A CAN-based bootloader for STM32L432 microcontrollers, designed for firmware updates over CAN bus (BMS and SDC).

## Memory Layout

- **Bootloader**: `0x08000000 - 0x08007FFF` (32KB)
- **Application**: `0x08008000 - 0x0803FFFF` (224KB)
- **Valid Flag**: `0x08007FF8` (Application validity marker)

## CAN Protocol

### CAN Message IDs (Actual CAN IDs)

| Direction | CAN ID | Description |
|-----------|--------|-------------|
| Host → Bootloader | `0x701` | Commands from PC/Host |
| Bootloader → Host | `0x700` | Responses from Bootloader |

### Command Codes (First byte of CAN data payload)

| Command | Code | Description |
|---------|------|-------------|
| `CMD_ERASE_FLASH` | `0x01` | Erase application flash |
| `CMD_WRITE_DATA` | `0x07` | Write 4 bytes (buffered to 8) |
| `CMD_READ_FLASH` | `0x03` | Read data from flash |
| `CMD_JUMP_TO_APP` | `0x04` | Jump to application |
| `CMD_GET_STATUS` | `0x05` | Get bootloader status |
| `CMD_SET_ADDRESS` | `0x06` | Set write address |

### Response Codes (First byte of CAN data payload)

| Response | Code | Description |
|----------|------|-------------|
| `RESP_ACK` | `0x10` | Command acknowledged |
| `RESP_NACK` | `0x11` | Command rejected |
| `RESP_READY` | `0x14` | Bootloader ready |
| `RESP_DATA` | `0x15` | Data response |
| `RESP_JUMP_INFO` | `0x16` | Jump information |

## Usage

1. **Enter Bootloader**: Reset MCU without valid application or send commands within 5-second window
2. **Erase Flash**: Send `CMD_ERASE_FLASH` to prepare for new firmware
3. **Set Address**: Use `CMD_SET_ADDRESS` to specify write location
4. **Write Data**: Send firmware data using `CMD_WRITE_DATA` (4 bytes per message)
5. **Jump to App**: Send `CMD_JUMP_TO_APP` to boot new firmware

The bootloader automatically validates the application and jumps to it on startup if valid.
>>>>>>> origin/master
