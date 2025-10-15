#!/usr/bin/env python3
"""
STM32L432 CAN Bootloader Flash Script - 4-Byte Chunk Method
===========================================================
Optimized for 4-byte chunks that align perfectly with STM32 flash requirements.
Flash application firmware to STM32L432 via CAN bus and verify by reading back.

Usage:
    python Flash_Application.py application.bin [--adapter pcan] [--channel USB1]
    python Flash_Application.py application.bin [--adapter canable] [--channel 0]

Requirements:
    - PCAN_Driver.py or CANable_Driver.py (adapter driver modules)
    - python-can library
    - PCAN-USB adapter or CANable adapter

Author: GitHub Copilot
Date: October 15, 2025
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Optional, Tuple, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod

# Import both drivers
try:
    from PCAN_Driver import PCANDriver, PCANChannel, PCANBaudRate, CANMessage as PCANMessage
    PCAN_AVAILABLE = True
except ImportError:
    PCAN_AVAILABLE = False
    print("Warning: PCAN_Driver not available")

try:
    from CANable_Driver import CANableDriver, CANableBaudRate, CANMessage as CANableMessage
    CANABLE_AVAILABLE = True
except ImportError:
    CANABLE_AVAILABLE = False
    print("Warning: CANable_Driver not available")



# ============================================================================
# Unified CAN Message Class
# ============================================================================

@dataclass
class CANMessage:
    """Unified CAN message class compatible with both drivers"""
    id: int
    data: bytes
    timestamp: float = 0.0
    is_extended: bool = False
    is_remote: bool = False
    is_error: bool = False
    is_fd: bool = False
    dlc: int = 0

    def __post_init__(self):
        if self.dlc == 0:
            self.dlc = len(self.data)


# ============================================================================
# Abstract Driver Interface
# ============================================================================

class CANAdapter(ABC):
    """Abstract base class for CAN adapters"""

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the CAN adapter"""
        pass

    @abstractmethod
    def disconnect(self):
        """Disconnect from the CAN adapter"""
        pass

    @abstractmethod
    def send_message(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        """Send a CAN message"""
        pass

    @abstractmethod
    def read_message(self, timeout: float = 1.0) -> Optional[CANMessage]:
        """Read a CAN message"""
        pass

    @abstractmethod
    def clear_receive_queue(self) -> bool:
        """Clear the receive queue"""
        pass


class PCANAdapter(CANAdapter):
    """Adapter wrapper for PCAN driver"""

    def __init__(self, channel: str):
        if not PCAN_AVAILABLE:
            raise RuntimeError("PCAN driver not available")
        self.driver = PCANDriver()
        self.channel = PCANChannel[channel]

    def connect(self) -> bool:
        return self.driver.connect(self.channel, PCANBaudRate.BAUD_500K)

    def disconnect(self):
        self.driver.disconnect()

    def send_message(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        return self.driver.send_message(can_id, data, is_extended)

    def read_message(self, timeout: float = 1.0) -> Optional[CANMessage]:
        msg = self.driver.read_message(timeout)
        if msg is None:
            return None
        # Convert PCAN message to unified format
        return CANMessage(
            id=msg.id,
            data=msg.data,
            timestamp=msg.timestamp,
            is_extended=msg.is_extended,
            is_remote=msg.is_remote,
            is_error=msg.is_error,
            is_fd=msg.is_fd,
            dlc=msg.dlc
        )

    def clear_receive_queue(self) -> bool:
        return self.driver.clear_receive_queue()


class CANableAdapter(CANAdapter):
    """Adapter wrapper for CANable driver"""

    def __init__(self, channel: int):
        if not CANABLE_AVAILABLE:
            raise RuntimeError("CANable driver not available")
        self.driver = CANableDriver()
        self.channel = channel

    def connect(self) -> bool:
        return self.driver.connect(self.channel, CANableBaudRate.BAUD_500K)

    def disconnect(self):
        self.driver.disconnect()

    def send_message(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        return self.driver.send_message(can_id, data, is_extended)

    def read_message(self, timeout: float = 1.0) -> Optional[CANMessage]:
        msg = self.driver.read_message(timeout)
        if msg is None:
            return None
        # Convert CANable message to unified format
        return CANMessage(
            id=msg.id,
            data=msg.data,
            timestamp=msg.timestamp,
            is_extended=msg.is_extended,
            is_remote=msg.is_remote,
            is_error=msg.is_error,
            is_fd=msg.is_fd,
            dlc=msg.dlc
        )

    def clear_receive_queue(self) -> bool:
        return self.driver.clear_receive_queue()


# ============================================================================
# Bootloader Protocol Constants
# ============================================================================

# CAN IDs - 29-bit Extended IDs
CAN_HOST_ID = 0x18000701         # PC sends commands to this ID
CAN_BOOTLOADER_ID = 0x18000700   # Bootloader responds from this ID

# Commands
CMD_ERASE_FLASH = 0x01
CMD_WRITE_FLASH = 0x02
CMD_READ_FLASH = 0x03
CMD_JUMP_TO_APP = 0x04
CMD_GET_STATUS = 0x05
CMD_SET_ADDRESS = 0x06
CMD_WRITE_DATA = 0x07

# Responses
RESP_ACK = 0x10
RESP_NACK = 0x11
RESP_ERROR = 0x12
RESP_BUSY = 0x13
RESP_READY = 0x14
RESP_DATA = 0x15

# Error Codes
ERR_NONE = 0x00
ERR_INVALID_COMMAND = 0x01
ERR_INVALID_ADDRESS = 0x02
ERR_FLASH_ERASE_FAILED = 0x03
ERR_FLASH_WRITE_FAILED = 0x04
ERR_INVALID_DATA_LENGTH = 0x05
ERR_NO_VALID_APP = 0x06
ERR_TIMEOUT = 0x07

# Memory Configuration
APP_START_ADDRESS = 0x08008000
APP_MAX_SIZE = 208 * 1024  # 208 KB (last 16KB reserved for permanent storage)

# Timing
RESPONSE_TIMEOUT = 1.0       # Normal response timeout (seconds)
ERASE_TIMEOUT = 15.0         # Flash erase timeout (seconds)
WRITE_CHUNK_SIZE = 4         # Write 4 bytes per CAN message (bootloader buffers 2 chunks for 8-byte flash write)


# ============================================================================
# Error Code Descriptions
# ============================================================================

ERROR_DESCRIPTIONS = {
    ERR_NONE: "No error",
    ERR_INVALID_COMMAND: "Invalid command",
    ERR_INVALID_ADDRESS: "Invalid address",
    ERR_FLASH_ERASE_FAILED: "Flash erase failed",
    ERR_FLASH_WRITE_FAILED: "Flash write failed",
    ERR_INVALID_DATA_LENGTH: "Invalid data length",
    ERR_NO_VALID_APP: "No valid application",
    ERR_TIMEOUT: "Operation timeout"
}


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class BootloaderStatus:
    """Bootloader status information"""
    state: int
    error: int
    bytes_written: int
    
    def __str__(self):
        states = ['IDLE', 'ERASING', 'WRITING', 'READING', 'VERIFYING', 'JUMPING']
        state_name = states[self.state] if self.state < len(states) else 'UNKNOWN'
        error_desc = ERROR_DESCRIPTIONS.get(self.error, f'Unknown error {self.error}')
        return f"State: {state_name}, Error: {error_desc}, Bytes Written: {self.bytes_written}"


# ============================================================================
# CAN Bootloader Flash Class
# ============================================================================

class CANBootloaderFlash:
    """
    Main class for flashing firmware via CAN bootloader.
    """

    def __init__(self, adapter: CANAdapter):
        """
        Initialize the CAN flasher.

        Args:
            adapter: CAN adapter instance (PCANAdapter or CANableAdapter)
        """
        self.driver = adapter
        self.connected = False
        self.verbose = True
        
    def connect(self) -> bool:
        """
        Connect to CAN device and initialize CAN communication.

        Returns:
            True if connection successful
        """
        print(f"\n{'='*60}")
        print("Connecting to CAN device...")
        print(f"{'='*60}")

        # Connect to CAN adapter at 500 kbps (bootloader baud rate)
        if not self.driver.connect():
            print("✗ Failed to connect to CAN device")
            return False

        self.connected = True

        # Clear receive queue
        self.driver.clear_receive_queue()

        print("✓ Connected successfully")

        # Wait for bootloader READY message
        self.wait_for_bootloader_ready()

        print()
        return True
    
    def disconnect(self):
        """Disconnect from CAN device."""
        if self.connected:
            self.driver.disconnect()
            self.connected = False
    
    def send_command(self, command: int, data: list) -> bool:
        """
        Send a command to the bootloader.
        
        Args:
            command: Command byte
            data: List of data bytes (will be padded to 8 bytes)
        
        Returns:
            True if sent successfully
        """
        # Prepare message (pad to 8 bytes)
        msg_data = [command] + data
        while len(msg_data) < 8:
            msg_data.append(0x00)
        
        # Send to bootloader (use extended 29-bit ID)
        return self.driver.send_message(CAN_HOST_ID, bytes(msg_data[:8]), is_extended=True)
    
    def wait_response(self, timeout: float = RESPONSE_TIMEOUT) -> Optional[CANMessage]:
        """
        Wait for a response from bootloader.
        
        Args:
            timeout: Maximum time to wait in seconds
        
        Returns:
            CANMessage if received, None if timeout
        """
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            msg = self.driver.read_message(timeout=0.1)
            
            if msg and msg.id == CAN_BOOTLOADER_ID:
                return msg
        
        return None
    
    def wait_for_bootloader_ready(self, timeout: float = 3.0) -> bool:
        """
        Wait for bootloader READY message on startup.
        According to BUILD_AND_FLASH_INSTRUCTIONS.md:
        - Bootloader sends READY message on power-up
        - CAN ID: 0x700
        - Data: 0x14 0x01 0x00 ... (READY + version)
        
        Args:
            timeout: Maximum time to wait for READY message
            
        Returns:
            True if READY message received
        """
        if self.verbose:
            print("Waiting for bootloader READY message...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            msg = self.driver.read_message(timeout=0.1)
            
            if msg and msg.id == CAN_BOOTLOADER_ID:
                if len(msg.data) > 0 and msg.data[0] == RESP_READY:
                    version = msg.data[1] if len(msg.data) > 1 else 0
                    if self.verbose:
                        print(f"✓ Bootloader READY (version: {version})")
                    return True
        
        if self.verbose:
            print("⚠ No READY message received (bootloader may already be running)")
        return False
    
    def get_status(self) -> Optional[BootloaderStatus]:
        """
        Get bootloader status.
        
        Returns:
            BootloaderStatus object or None if failed
        """
        if self.verbose:
            print("Getting bootloader status...")
        
        # Send GET_STATUS command
        if not self.send_command(CMD_GET_STATUS, []):
            return None
        
        # Wait for response
        resp = self.wait_response()
        if not resp or len(resp.data) < 7:
            if self.verbose:
                print("✗ No response or invalid response")
            return None
        
        # Parse response
        if resp.data[0] == RESP_DATA:
            status = BootloaderStatus(
                state=resp.data[1],
                error=resp.data[2],
                bytes_written=(resp.data[3] << 24) | (resp.data[4] << 16) | 
                              (resp.data[5] << 8) | resp.data[6]
            )
            if self.verbose:
                print(f"✓ {status}")
            return status
        
        return None
    
    def erase_flash(self) -> bool:
        """
        Erase application flash area.
        
        Returns:
            True if erase successful
        """
        print("\n" + "="*60)
        print("Erasing flash memory...")
        print("="*60)
        print("This may take several seconds...")
        
        # Send ERASE command
        if not self.send_command(CMD_ERASE_FLASH, []):
            print("✗ Failed to send erase command")
            return False
        
        # Wait for ACK (with longer timeout)
        resp = self.wait_response(timeout=ERASE_TIMEOUT)
        
        if not resp:
            print("✗ Erase timeout (no response)")
            return False
        
        if resp.data[0] == RESP_ACK:
            print("✓ Flash erased successfully\n")
            return True
        elif resp.data[0] == RESP_NACK:
            error_code = resp.data[1] if len(resp.data) > 1 else 0
            error_desc = ERROR_DESCRIPTIONS.get(error_code, f"Error {error_code}")
            print(f"✗ Erase failed: {error_desc}")
            return False
        else:
            print(f"✗ Unexpected response: 0x{resp.data[0]:02X}")
            return False
    
    def set_address(self, address: int) -> bool:
        """
        Set write address pointer.
        
        Args:
            address: Flash address to set
        
        Returns:
            True if successful
        """
        if self.verbose:
            print(f"Setting address to 0x{address:08X}...")
        
        # Prepare address bytes (MSB first)
        addr_bytes = [
            (address >> 24) & 0xFF,
            (address >> 16) & 0xFF,
            (address >> 8) & 0xFF,
            address & 0xFF
        ]
        
        # Send SET_ADDRESS command
        if not self.send_command(CMD_SET_ADDRESS, addr_bytes):
            return False
        
        # Wait for ACK
        resp = self.wait_response()
        
        if not resp:
            if self.verbose:
                print("✗ No response")
            return False
        
        if resp.data[0] == RESP_ACK:
            if self.verbose:
                print("✓ Address set")
            return True
        elif resp.data[0] == RESP_NACK:
            error_code = resp.data[1] if len(resp.data) > 1 else 0
            error_desc = ERROR_DESCRIPTIONS.get(error_code, f"Error {error_code}")
            if self.verbose:
                print(f"✗ Failed: {error_desc}")
            return False
        
        return False
    
    def write_4bytes(self, data: bytes) -> bool:
        """
        Write exactly 4 bytes to flash.
        Bootloader buffers two 4-byte chunks before writing 8 bytes to flash.
        
        Args:
            data: Exactly 4 bytes to write
            
        Returns:
            True if write successful
        """
        if len(data) != 4:
            raise ValueError(f"Must write exactly 4 bytes, got {len(data)}")
        
        # Build command: [CMD] [0x04] [byte0] [byte1] [byte2] [byte3]
        cmd_data = [0x04] + list(data)
        
        # Send WRITE_DATA command
        if not self.send_command(CMD_WRITE_DATA, cmd_data):
            return False
        
        # Wait for ACK
        resp = self.wait_response()
        
        if not resp:
            return False
        
        if resp.data[0] == RESP_ACK:
            return True
        elif resp.data[0] == RESP_NACK:
            error_code = resp.data[1] if len(resp.data) > 1 else 0
            print(f"\n✗ Write failed: {ERROR_DESCRIPTIONS.get(error_code, 'Unknown error')}")
            return False
        
        return False
    
    def read_data(self, address: int, length: int) -> Optional[bytes]:
        """
        Read data from flash.
        
        Args:
            address: Flash address
            length: Number of bytes to read (max 7)
            
        Returns:
            Read data or None if failed
        """
        if length == 0 or length > 7:
            return None
        
        # Build command: [CMD] [addr3] [addr2] [addr1] [addr0] [length]
        addr_bytes = [
            (address >> 24) & 0xFF,
            (address >> 16) & 0xFF,
            (address >> 8) & 0xFF,
            address & 0xFF,
            length
        ]
        
        if not self.send_command(CMD_READ_FLASH, addr_bytes):
            return None
        
        msg = self.wait_response()
        if not msg or len(msg.data) == 0:
            return None
        
        if msg.data[0] == RESP_DATA:
            # Data starts at byte 1
            return bytes(msg.data[1:1+length])
        
        return None
    
    @staticmethod
    def pad_to_4byte_boundary(data: bytes) -> bytes:
        """
        Pad data to 4-byte boundary (and therefore 8-byte boundary).
        
        Args:
            data: Original firmware data
            
        Returns:
            Padded data (length is multiple of 4)
        """
        padding_needed = (4 - len(data) % 4) % 4
        if padding_needed > 0:
            data = data + b'\xFF' * padding_needed
        return data
    
    def write_firmware(self, firmware_data: bytes) -> bool:
        """
        Write complete firmware to flash using 4-byte chunks.
        
        Args:
            firmware_data: Complete firmware binary (will be padded to 4-byte boundary)
        
        Returns:
            True if write successful
        """
        print("\n" + "="*60)
        print("Writing firmware...")
        print("="*60)
        
        total_bytes = len(firmware_data)
        chunk_size = WRITE_CHUNK_SIZE  # 4 bytes per message
        
        print(f"Total size: {total_bytes} bytes ({total_bytes/1024:.2f} KB)")
        print(f"Chunk size: {chunk_size} bytes per CAN message")
        print("Bootloader buffers 2 chunks (8 bytes) before writing to flash")
        print()
        
        # Set initial address
        if not self.set_address(APP_START_ADDRESS):
            print("✗ Failed to set initial address")
            return False
        
        # Write data in 4-byte chunks
        start_time = time.time()
        last_progress = -1
        bytes_written = 0
        
        while bytes_written < total_bytes:
            # Get next 4-byte chunk
            chunk_end = min(bytes_written + chunk_size, total_bytes)
            chunk = firmware_data[bytes_written:chunk_end]
            
            # Ensure exactly 4 bytes (pad if needed for last chunk)
            if len(chunk) < 4:
                chunk = chunk + b'\xFF' * (4 - len(chunk))
            
            # Write 4-byte chunk
            if not self.write_4bytes(chunk):
                print(f"\n✗ Write failed at offset 0x{bytes_written:08X}")
                return False
            
            bytes_written += len(chunk) if chunk_end != total_bytes or len(chunk) == 4 else (chunk_end - bytes_written)
            
            # Update progress every 128 bytes (32 messages)
            progress = int((bytes_written * 100) / total_bytes)
            if bytes_written % 128 == 0 or bytes_written >= total_bytes:
                if progress != last_progress:
                    elapsed = time.time() - start_time
                    speed = bytes_written / elapsed / 1024 if elapsed > 0 else 0
                    eta = (total_bytes - bytes_written) / (bytes_written / elapsed) if elapsed > 0 and bytes_written > 0 else 0
                    print(f"Progress: {progress:3d}% [{bytes_written}/{total_bytes} bytes] "
                          f"Speed: {speed:.1f} KB/s ETA: {eta:.1f}s", end='\r')
                    last_progress = progress
        
        elapsed = time.time() - start_time
        avg_speed = total_bytes / elapsed / 1024 if elapsed > 0 else 0
        print(f"\n\n✓ Firmware written successfully!")
        print(f"  Total time: {elapsed:.1f}s")
        print(f"  Average speed: {avg_speed:.1f} KB/s\n")
        
        return True
    
    def verify_flash(self, expected_data: bytes) -> bool:
        """
        Verify flashed data by reading back and comparing.
        
        Args:
            expected_data: Expected binary data
            
        Returns:
            True if verification successful
        """
        print("\n" + "="*60)
        print("Verifying flash contents...")
        print("="*60)
        
        address = APP_START_ADDRESS
        bytes_verified = 0
        chunk_size = 4  # Read 4 bytes at a time for consistency with write
        
        start_time = time.time()
        last_progress = -1
        
        while bytes_verified < len(expected_data):
            # Read chunk
            remaining = len(expected_data) - bytes_verified
            read_size = min(chunk_size, remaining)
            
            read_data = self.read_data(address, read_size)
            
            if read_data is None:
                print(f"\n✗ Failed to read at address 0x{address:08X}")
                return False
            
            # Compare
            expected_chunk = expected_data[bytes_verified:bytes_verified + read_size]
            
            if read_data != expected_chunk:
                print(f"\n✗ Verification failed at address 0x{address:08X}")
                print(f"  Expected: {expected_chunk.hex()}")
                print(f"  Read:     {read_data.hex()}")
                return False
            
            bytes_verified += read_size
            address += read_size
            
            # Update progress every 128 bytes
            progress = int((bytes_verified * 100) / len(expected_data))
            if bytes_verified % 128 == 0 or bytes_verified >= len(expected_data):
                if progress != last_progress:
                    elapsed = time.time() - start_time
                    speed = bytes_verified / elapsed / 1024 if elapsed > 0 else 0
                    print(f"Verifying: {progress:3d}% [{bytes_verified}/{len(expected_data)} bytes] "
                          f"Speed: {speed:.1f} KB/s", end='\r')
                    last_progress = progress
        
        elapsed = time.time() - start_time
        print(f"\n\n✓ Verification successful ({bytes_verified} bytes)")
        print(f"  Total time: {elapsed:.1f}s\n")
        
        return True
    
    def jump_to_application(self) -> bool:
        """
        Command bootloader to jump to application.
        
        Returns:
            True if command sent successfully
        """
        print("\n" + "="*60)
        print("Jumping to application...")
        print("="*60)
        
        # Send JUMP command
        if not self.send_command(CMD_JUMP_TO_APP, []):
            print("✗ Failed to send jump command")
            return False
        
        # Wait for ACK (bootloader may not respond if it successfully jumps)
        resp = self.wait_response(timeout=0.5)
        
        if resp:
            if resp.data[0] == RESP_ACK:
                print("✓ Application started\n")
                return True
            elif resp.data[0] == RESP_NACK:
                error_code = resp.data[1] if len(resp.data) > 1 else 0
                error_desc = ERROR_DESCRIPTIONS.get(error_code, f"Error {error_code}")
                print(f"✗ Jump failed: {error_desc}")
                return False
        else:
            # No response might mean bootloader successfully jumped
            print("✓ Command sent (bootloader may have jumped)\n")
            return True
        
        return False
    
    def flash_firmware(self, firmware_path: Path, verify: bool = True, 
                      jump: bool = True) -> bool:
        """
        Complete firmware flashing process.
        
        Args:
            firmware_path: Path to .bin file
            verify: Verify by reading back after writing (default: True)
            jump: Jump to application after flashing (default: True)
        
        Returns:
            True if flashing successful
        """
        # Read firmware file
        print(f"\n{'='*60}")
        print(f"Loading firmware: {firmware_path.name}")
        print(f"{'='*60}")
        
        try:
            firmware_data = firmware_path.read_bytes()
            original_size = len(firmware_data)

            # Truncate to APP_MAX_SIZE if larger (last 16KB reserved for permanent storage)
            if original_size > APP_MAX_SIZE:
                print(f"⚠ Firmware size ({original_size} bytes) exceeds {APP_MAX_SIZE} bytes")
                print(f"  Truncating to {APP_MAX_SIZE} bytes (last 16KB reserved for permanent storage)")
                firmware_data = firmware_data[:APP_MAX_SIZE]
                original_size = len(firmware_data)

            # Pad to 4-byte boundary (ensures 8-byte alignment)
            firmware_data = self.pad_to_4byte_boundary(firmware_data)

            print(f"✓ Loaded {original_size} bytes ({original_size/1024:.2f} KB)")
            if len(firmware_data) != original_size:
                print(f"  Padded to {len(firmware_data)} bytes (4-byte aligned)\n")
            else:
                print()
        except Exception as e:
            print(f"✗ Failed to read firmware file: {e}")
            return False
        
        # Get initial status
        status = self.get_status()
        if not status:
            print("⚠ Warning: Could not get bootloader status")
            print("  Continuing anyway...\n")
        
        # Erase flash
        if not self.erase_flash():
            return False
        
        # Write firmware
        if not self.write_firmware(firmware_data):
            return False
        
        # Verify by reading back
        if verify:
            if not self.verify_flash(firmware_data):
                print("⚠ Warning: Flash verification failed")
                return False
        
        # Jump to application
        if jump:
            if not self.jump_to_application():
                print("⚠ Warning: Jump command may have failed")
        
        return True


# ============================================================================
# Main Function
# ============================================================================

def main():
    """Main entry point for the script."""

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Flash firmware to STM32L432 via CAN bootloader',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python Flash_Application.py application.bin
  python Flash_Application.py application.bin --adapter pcan --channel USB1
  python Flash_Application.py application.bin --adapter canable --channel 0
  python Flash_Application.py application.bin --no-jump
  python Flash_Application.py application.bin --status-only
        '''
    )

    parser.add_argument('firmware', type=str, nargs='?',
                       help='Path to firmware .bin file')
    parser.add_argument('--adapter', type=str, default='pcan',
                       choices=['pcan', 'canable'],
                       help='CAN adapter type (default: pcan)')
    parser.add_argument('--channel', type=str, default=None,
                       help='Channel: For PCAN use USB1-USB16, for CANable use device index 0-N (default: USB1 for PCAN, 0 for CANable)')
    parser.add_argument('--verify', action='store_true', default=True,
                       help='Verify by reading back after flashing (default: enabled)')
    parser.add_argument('--no-verify', action='store_false', dest='verify',
                       help='Skip read-back verification')
    parser.add_argument('--jump', action='store_true', default=True,
                       help='Jump to application after flashing (default: enabled)')
    parser.add_argument('--no-jump', action='store_false', dest='jump',
                       help='Stay in bootloader after flashing')
    parser.add_argument('--status-only', action='store_true',
                       help='Only get bootloader status and exit')
    parser.add_argument('--list-devices', action='store_true',
                       help='List available CAN devices and exit')

    args = parser.parse_args()

    # Set default channel if not specified
    if args.channel is None:
        args.channel = 'USB1' if args.adapter == 'pcan' else '0'
    
    # Print banner
    print("\n" + "="*60)
    print("STM32L432 CAN Bootloader Flash Tool")
    print("="*60)
    print(f"Version: 2.0")
    print(f"Date: October 15, 2025")
    print("="*60 + "\n")

    # List devices if requested
    if args.list_devices:
        if args.adapter == 'pcan':
            if not PCAN_AVAILABLE:
                print("✗ PCAN driver not available. Install PCAN_Driver.py")
                return 1
            driver = PCANDriver()
            print("Scanning for PCAN devices...\n")
            devices = driver.get_available_devices()

            if not devices:
                print("✗ No PCAN devices found")
                return 1

            print(f"Found {len(devices)} device(s):\n")
            for dev in devices:
                status = "OCCUPIED" if dev['occupied'] else "AVAILABLE"
                print(f"  {dev['channel']:10s} : {status}")

        elif args.adapter == 'canable':
            if not CANABLE_AVAILABLE:
                print("✗ CANable driver not available. Install CANable_Driver.py")
                return 1
            driver = CANableDriver()
            print("Scanning for CANable devices...\n")
            devices = driver.get_available_devices()

            if not devices:
                print("✗ No CANable devices found")
                return 1

            print(f"Found {len(devices)} device(s):\n")
            for dev in devices:
                print(f"  [{dev['index']}] {dev['description']}")
                print(f"      VID: 0x{dev['vid']:04X}, PID: 0x{dev['pid']:04X}")
                print(f"      Serial: {dev['serial_number']}")

        print()
        return 0
    
    # Check firmware file
    if not args.status_only and not args.firmware:
        parser.print_help()
        return 1

    if args.firmware:
        firmware_path = Path(args.firmware)
        if not firmware_path.exists():
            print(f"✗ Error: Firmware file not found: {firmware_path}")
            return 1

    # Create adapter instance based on selection
    try:
        if args.adapter == 'pcan':
            if not PCAN_AVAILABLE:
                print("✗ PCAN driver not available. Install PCAN_Driver.py and python-can")
                return 1
            adapter = PCANAdapter(args.channel)
            adapter_name = f"PCAN {args.channel}"
        elif args.adapter == 'canable':
            if not CANABLE_AVAILABLE:
                print("✗ CANable driver not available. Install CANable_Driver.py and python-can")
                return 1
            channel_index = int(args.channel)
            adapter = CANableAdapter(channel_index)
            adapter_name = f"CANable device {channel_index}"
        else:
            print(f"✗ Unknown adapter type: {args.adapter}")
            return 1
    except Exception as e:
        print(f"✗ Failed to create adapter: {e}")
        return 1

    # Create flasher instance
    flasher = CANBootloaderFlash(adapter)
    
    try:
        # Connect to CAN adapter
        if not flasher.connect():
            return 1

        # Status only mode
        if args.status_only:
            status = flasher.get_status()
            return 0 if status else 1

        # Flash firmware
        print(f"Firmware file: {firmware_path}")
        print(f"CAN adapter:   {adapter_name}")
        print(f"Read-back verify: {'Yes' if args.verify else 'No'}")
        print(f"Jump to app:   {'Yes' if args.jump else 'No'}")
        
        success = flasher.flash_firmware(
            firmware_path,
            verify=args.verify,
            jump=args.jump
        )
        
        if success:
            print("\n" + "="*60)
            print("✓ FLASHING COMPLETED SUCCESSFULLY!")
            print("="*60 + "\n")
            return 0
        else:
            print("\n" + "="*60)
            print("✗ FLASHING FAILED")
            print("="*60 + "\n")
            return 1
    
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        return 1
    
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Always disconnect
        flasher.disconnect()


if __name__ == '__main__':
    sys.exit(main())
