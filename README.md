# STM32-CAN-Bootloader
CAN bootloader for the STM32L432 with dual-bank application support, boot metadata in flash, and host-driven CRC validation over a 29-bit extended CAN protocol.

## Overview

This bootloader is designed for field updates over CAN on STM32L432-based modules.

Current behavior:

- Reserves the first 30 KB of flash for bootloader code
- Reserves one 2 KB flash page for boot metadata
- Splits the application region into two 104 KB banks: Bank A and Bank B
- Boots the bank marked active in metadata
- Only allows writes and erases to the inactive bank
- Requires the host to provide image CRC32 and image size before a bank is promoted
- Reports bank validity, active bank, update state, and CRC health in the READY heartbeat
- Computes startup CRC status for both banks whenever stored CRC/size metadata exists, even if that bank is currently marked invalid

This repository contains the firmware bootloader. For application-side linker setup, see [DUAL_BANK_APPLICATION_GUIDE.md](DUAL_BANK_APPLICATION_GUIDE.md).

## Flash Layout

```text
STM32L432 Flash (256 KB)

0x08000000  +----------------------------------+
            | Bootloader code                  |
            | 30 KB                            |
0x08007800  +----------------------------------+
            | Boot metadata page               |
            | 2 KB                             |
0x08008000  +----------------------------------+
            | Application Bank A               |
            | 104 KB                           |
0x08022000  +----------------------------------+
            | Application Bank B               |
            | 104 KB                           |
0x0803C000  +----------------------------------+
            | Permanent storage / app-owned    |
            | 16 KB                            |
0x08040000  +----------------------------------+
```

### Regions

- Bootloader code: `0x08000000 - 0x080077FF`
- Boot metadata page: `0x08007800 - 0x08007FFF`
- Bank A: `0x08008000 - 0x08021FFF`
- Bank B: `0x08022000 - 0x0803BFFF`
- Permanent storage: `0x0803C000 - 0x0803FFFF`

### Notes

- The bootloader never flashes application code into the metadata page.
- The bootloader only writes to the inactive bank.
- The bootloader does not use the last 16 KB for firmware updates.
- Maximum image size per bank is `104 KB`.

## Boot Metadata

Boot metadata is stored in flash page 15 at `0x08007800`.

The metadata records:

- Active bank (`A` or `B`)
- Bank A valid flag
- Bank B valid flag
- Stored CRC32 for Bank A
- Stored CRC32 for Bank B
- Stored image size for Bank A
- Stored image size for Bank B
- Update-in-progress marker

Important behavior:

- Validity is host-authoritative. A bank is only marked valid after the host completes the verified update flow.
- During erase of the inactive bank, that bank is invalidated and the metadata is updated immediately.
- On startup, the bootloader computes CRC status for any bank that has stored CRC and size metadata, even if that bank is marked invalid. This lets the host inspect whether the flash contents still match the stored image CRC.

## Boot Flow

### Normal startup

1. Bootloader reads metadata.
2. Bootloader computes CRC health flags for both banks when stored metadata is present.
3. Bootloader begins sending READY heartbeats.
4. If no host command is received within the timeout, it attempts to jump to the active bank.
5. If the active bank is not bootable but the other bank has a valid vector table, it may fall back to that bank for recovery.

### Interrupted update recovery

If metadata indicates an update was in progress:

- The bootloader only boots the currently active bank.
- That bank must pass metadata validity, stored CRC check, and vector table checks.
- Otherwise the bootloader remains active and waits for recovery via CAN.

## Update Workflow

The intended dual-bank update sequence is:

1. Host enters bootloader mode.
2. Host reads READY heartbeat or sends `CMD_GET_ACTIVE_BANK`.
3. Host selects the inactive bank as the target.
4. Host sends `CMD_ERASE_FLASH`.
5. Host sends `CMD_SET_ADDRESS` and `CMD_WRITE_DATA` messages to program the inactive bank.
6. Host sends `CMD_SET_IMAGE_INFO` with expected CRC32 and image size.
7. Host sends `CMD_VERIFY_BANK`.
8. If verification succeeds, host sends `CMD_JUMP_TO_APP`.
9. Bootloader commits the new bank in metadata and jumps to it.

Key constraints:

- `CMD_ERASE_FLASH` erases the inactive bank only.
- `CMD_SET_ADDRESS` and `CMD_WRITE_DATA` are limited to the inactive bank only.
- `CMD_WRITE_DATA` accepts 4 bytes per CAN frame and buffers two frames before programming one 8-byte flash doubleword.
- The active bank is not switched until verification has succeeded and `CMD_JUMP_TO_APP` is processed.

## Application Requirements

Applications must be built separately for Bank A and Bank B.

See [DUAL_BANK_APPLICATION_GUIDE.md](DUAL_BANK_APPLICATION_GUIDE.md) for the complete setup.

### Required linker origins

- Bank A build: `FLASH ORIGIN = 0x08008000`
- Bank B build: `FLASH ORIGIN = 0x08022000`

### Required VTOR offsets

- Bank A build: `VECT_TAB_OFFSET = 0x8000`
- Bank B build: `VECT_TAB_OFFSET = 0x22000`

## CAN Protocol

The bootloader uses 29-bit extended CAN IDs.

### CAN IDs

| Direction | CAN ID | Description |
| --- | --- | --- |
| Host -> Bootloader | `0x18000701` | Bootloader commands |
| Bootloader -> Host | `0x18000700` | Responses and READY heartbeat |

### Command Codes

| Command | Code | Description |
| --- | --- | --- |
| `CMD_ERASE_FLASH` | `0x01` | Erase inactive application bank |
| `CMD_WRITE_FLASH` | `0x02` | Legacy write command |
| `CMD_READ_FLASH` | `0x03` | Read from Bank A or Bank B |
| `CMD_JUMP_TO_APP` | `0x04` | Jump to active bank, or commit verified inactive bank and jump |
| `CMD_GET_STATUS` | `0x05` | Return bootloader state/error/write count |
| `CMD_SET_ADDRESS` | `0x06` | Set write pointer in inactive bank |
| `CMD_WRITE_DATA` | `0x07` | Write 4 bytes, buffered into 8-byte flash writes |
| `CMD_GET_ACTIVE_BANK` | `0x08` | Return active bank and validity flags |
| `CMD_SET_IMAGE_INFO` | `0x09` | Send expected CRC32 and image size |
| `CMD_VERIFY_BANK` | `0x0A` | Verify inactive bank CRC |

### Response Codes

| Response | Code | Description |
| --- | --- | --- |
| `RESP_ACK` | `0x10` | Command accepted |
| `RESP_NACK` | `0x11` | Command rejected |
| `RESP_ERROR` | `0x12` | Error response |
| `RESP_BUSY` | `0x13` | Busy response |
| `RESP_READY` | `0x14` | READY heartbeat / diagnostic frame |
| `RESP_DATA` | `0x15` | Command-specific data payload |
| `RESP_JUMP_INFO` | `0x16` | Jump diagnostic payload |

### Error Codes

| Error | Code |
| --- | --- |
| `ERR_NONE` | `0x00` |
| `ERR_INVALID_COMMAND` | `0x01` |
| `ERR_INVALID_ADDRESS` | `0x02` |
| `ERR_FLASH_ERASE_FAILED` | `0x03` |
| `ERR_FLASH_WRITE_FAILED` | `0x04` |
| `ERR_INVALID_DATA_LENGTH` | `0x05` |
| `ERR_NO_VALID_APP` | `0x06` |
| `ERR_TIMEOUT` | `0x07` |
| `ERR_CRC_MISMATCH` | `0x08` |

## READY Heartbeat Format

`RESP_READY` is also the bootloader heartbeat frame.

Payload layout:

| Byte | Field | Meaning |
| --- | --- | --- |
| 0 | `RESP_READY` | Response code (`0x14`) |
| 1 | `code1` | Diagnostic marker |
| 2 | CRC health flags | Bit 0 = Bank A CRC OK, Bit 1 = Bank B CRC OK |
| 3 | State | `IDLE`, `ERASING`, `WRITING`, `READING`, `VERIFYING`, `JUMPING` |
| 4 | Last error | Last bootloader error code |
| 5 | Status flags | Active bank, validity, metadata ready, pending verify/jump, etc. |
| 6 | Bytes written hi | High byte of write count |
| 7 | Bytes written lo | Low byte of write count |

### `code1` values currently used

| Value | Meaning |
| --- | --- |
| `0x01` | Startup / normal heartbeat |
| `0xA5` | Interrupted update recovery path before jump |
| `0xAA` | Auto-jump timeout path before jump |
| `0xFF` | Explicit jump pending |

### Status flags in byte 5

| Bit | Meaning |
| --- | --- |
| 0 | Active bank is B (`0` = A, `1` = B) |
| 1 | Bank A marked valid |
| 2 | Bank B marked valid |
| 3 | Metadata ready |
| 4 | At least one host command seen since boot |
| 5 | Image info received in current session |
| 6 | A bank has been verified in current session |
| 7 | Jump pending |

## Host Tools

### Generate the DBC

```bash
python Scripts/generate_dbc_bootloader.py
```

This regenerates [STM32L432_Bootloader.dbc](STM32L432_Bootloader.dbc).

### Python flashing script

The repository includes [Scripts/Flash_Application.py](Scripts/Flash_Application.py) for CAN-based flashing.

Example:

```bash
python Scripts/Flash_Application.py application.bin --adapter pcan --channel USB1
```

### GUI flasher

The separate STM32-CAN-Flasher GUI repository also understands the dual-bank workflow and READY heartbeat fields.

## Build

Preferred in VS Code:

- `Build STM`
- `Build Clean STM`
- `Flash STM`

Terminal build:

```bash
& "$env:APPDATA\Code\User\globalStorage\bmd.stm32-for-vscode\@xpack-dev-tools\windows-build-tools\4.4.1-3.1\.content\bin\make.EXE" -j16 DEBUG=1 -f STM32Make.make
```

## Notes

- The bootloader CAN filter accepts only the host bootloader command ID, which prevents unrelated extended CAN traffic from filling the receive FIFO.
- Writes are performed as 8-byte flash doublewords, using two 4-byte CAN frames per write.
- CRC verification uses the reflected CRC32 polynomial `0xEDB88320`.
- A bank can report `CRC OK` in the READY heartbeat even while marked invalid if stored metadata still exists for that bank. This is intentional and allows better host-side diagnostics.
