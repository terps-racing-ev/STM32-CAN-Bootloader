# Dual-Bank Application Build Guide (_a / _b)

This guide explains how to build two application images for STM32L432 A/B partitioning in one compile action:

- `*_a.bin` linked for Bank A at `0x08008000`
- `*_b.bin` linked for Bank B at `0x08022000`

## 1) Required flash map

- Bootloader code: `0x08000000` to `0x080077FF` (30KB)
- Bootloader metadata page: `0x08007800` to `0x08007FFF` (2KB, reserved)
- App Bank A: `0x08008000` to `0x08021FFF` (104KB)
- App Bank B: `0x08022000` to `0x0803BFFF` (104KB)
- Permanent storage: `0x0803C000` to `0x0803FFFF` (application-owned)

Application constraints:

- Do not place application code below `0x08008000`.
- Do not erase or write bootloader metadata page `0x08007800-0x08007FFF`.
- Maximum application binary size per bank: `104KB`.

## 2) New application setup

For each new application project, create two linker scripts.

### `STM32L432XX_APP_BANK_A.ld`

Use your normal application linker script, but set:

- `FLASH ORIGIN = 0x08008000`
- `FLASH LENGTH = 104K`

### `STM32L432XX_APP_BANK_B.ld`

Copy Bank A script and set:

- `FLASH ORIGIN = 0x08022000`
- `FLASH LENGTH = 104K`

RAM sections stay unchanged.

## 3) Vector table offset per bank

Your application startup must use the matching VTOR offset:

- Bank A build: `VECT_TAB_OFFSET = 0x8000`
- Bank B build: `VECT_TAB_OFFSET = 0x22000`

Recommended approach: pass this as a compiler define in each build variant.

Example in `system_stm32l4xx.c`:

```c
#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET 0x00000000U
#endif

SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
```

## 4) Create dual-build wrapper makefile

In the application repo root, add `dual_build.mk`:

```make
# Dual bank wrapper for STM32Make.make
# Produces *_a.bin and *_b.bin in one build invocation.

BASE_TARGET ?= MyApplication
BUILD_DIR ?= build
INNER_MAKE ?= STM32Make.make

.PHONY: all bank_a bank_b clean

all: bank_a bank_b

bank_a:
	$(MAKE) -f $(INNER_MAKE) \
		TARGET=$(BASE_TARGET)_a \
		LDSCRIPT=STM32L432XX_APP_BANK_A.ld \
		C_DEFS="-DSTM32L432xx -DUSE_HAL_DRIVER -DVECT_TAB_OFFSET=0x8000" \
		CXX_DEFS="-DSTM32L432xx -DUSE_HAL_DRIVER -DVECT_TAB_OFFSET=0x8000"

bank_b:
	$(MAKE) -f $(INNER_MAKE) \
		TARGET=$(BASE_TARGET)_b \
		LDSCRIPT=STM32L432XX_APP_BANK_B.ld \
		C_DEFS="-DSTM32L432xx -DUSE_HAL_DRIVER -DVECT_TAB_OFFSET=0x22000" \
		CXX_DEFS="-DSTM32L432xx -DUSE_HAL_DRIVER -DVECT_TAB_OFFSET=0x22000"

clean:
	$(MAKE) -f $(INNER_MAKE) clean
```

Notes:

- This wrapper preserves your generated `STM32Make.make` and avoids editing it directly.
- If your generated makefile does not allow overriding `C_DEFS` and `LDSCRIPT` from command line, copy `STM32Make.make` to a project-owned makefile and replace `=` with `?=` for those variables.

## 5) VS Code compile button (Ctrl+Shift+B)

In the application repo, create or update `.vscode/tasks.json` to run the dual wrapper:

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Build App A+B",
      "type": "shell",
      "command": "make",
      "args": ["-f", "dual_build.mk"],
      "group": {
        "kind": "build",
        "isDefault": true
      },
      "problemMatcher": []
    }
  ]
}
```

Now pressing Ctrl+Shift+B builds both banks in one pass.

Expected outputs:

- `build/debug/<AppName>_a.bin`
- `build/debug/<AppName>_b.bin`

## 6) Flashing workflow with this bootloader

1. Bootloader reports active bank.
2. Host selects inactive bank.
3. Host flashes inactive bank image.
4. Host sends image CRC/size and requests verification.
5. Bootloader validates CRC and switches active bank.
6. Bootloader jumps to the new active bank.

Tip:

- If your CI runs release builds, generate both `_a` and `_b` artifacts for each version.
