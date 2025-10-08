"""
PCAN Driver Module
==================
A Python object-oriented driver for PCAN-USB adapters.
This module provides a high-level interface to interact with PEAK PCAN devices.

Author: GitHub Copilot
Date: October 8, 2025
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Callable
import time
import threading


try:
    from can import Bus, Message
    from can.interfaces.pcan import PcanBus
    from can.interfaces.pcan.basic import *
except ImportError:
    print("Error: python-can library not found. Install with: pip install python-can")
    raise


class PCANBaudRate(Enum):
    """Standard CAN baud rates"""
    BAUD_1M = PCAN_BAUD_1M
    BAUD_800K = PCAN_BAUD_800K
    BAUD_500K = PCAN_BAUD_500K
    BAUD_250K = PCAN_BAUD_250K
    BAUD_125K = PCAN_BAUD_125K
    BAUD_100K = PCAN_BAUD_100K
    BAUD_95K = PCAN_BAUD_95K
    BAUD_83K = PCAN_BAUD_83K
    BAUD_50K = PCAN_BAUD_50K
    BAUD_47K = PCAN_BAUD_47K
    BAUD_33K = PCAN_BAUD_33K
    BAUD_20K = PCAN_BAUD_20K
    BAUD_10K = PCAN_BAUD_10K
    BAUD_5K = PCAN_BAUD_5K


class PCANChannel(Enum):
    """Available PCAN channels"""
    USB1 = PCAN_USBBUS1
    USB2 = PCAN_USBBUS2
    USB3 = PCAN_USBBUS3
    USB4 = PCAN_USBBUS4
    USB5 = PCAN_USBBUS5
    USB6 = PCAN_USBBUS6
    USB7 = PCAN_USBBUS7
    USB8 = PCAN_USBBUS8
    USB9 = PCAN_USBBUS9
    USB10 = PCAN_USBBUS10
    USB11 = PCAN_USBBUS11
    USB12 = PCAN_USBBUS12
    USB13 = PCAN_USBBUS13
    USB14 = PCAN_USBBUS14
    USB15 = PCAN_USBBUS15
    USB16 = PCAN_USBBUS16


class PCANMessageType(Enum):
    """CAN message types"""
    STANDARD = PCAN_MESSAGE_STANDARD
    RTR = PCAN_MESSAGE_RTR
    EXTENDED = PCAN_MESSAGE_EXTENDED
    FD = PCAN_MESSAGE_FD
    BRS = PCAN_MESSAGE_BRS
    ESI = PCAN_MESSAGE_ESI


@dataclass
class CANMessage:
    """
    Represents a CAN message with all relevant information.
    """
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
    
    def __str__(self):
        msg_type = "EXT" if self.is_extended else "STD"
        data_str = ' '.join([f'{b:02X}' for b in self.data])
        return f"ID: 0x{self.id:X} [{msg_type}] DLC: {self.dlc} Data: [{data_str}]"


class PCANDriver:
    """
    Object-oriented driver for PCAN-USB adapters.
    
    This class provides a high-level interface to:
    - Initialize and connect to PCAN devices
    - Send and receive CAN messages
    - Configure CAN parameters
    - Monitor bus status
    - Handle errors
    
    Example:
        >>> driver = PCANDriver()
        >>> driver.connect(PCANChannel.USB1, PCANBaudRate.BAUD_500K)
        >>> driver.send_message(0x123, b'\\x01\\x02\\x03\\x04')
        >>> msg = driver.read_message()
        >>> driver.disconnect()
    """
    
    def __init__(self):
        """Initialize the PCAN driver."""
        self._bus: Optional[Bus] = None
        self._channel: Optional[PCANChannel] = None
        self._baudrate: Optional[PCANBaudRate] = None
        self._is_connected: bool = False
        self._receive_thread: Optional[threading.Thread] = None
        self._receive_callback: Optional[Callable[[CANMessage], None]] = None
        self._stop_receive: bool = False
        self._pcan_basic = PCANBasic()
        
    def get_available_devices(self) -> List[dict]:
        """
        Scan for available PCAN devices.
        
        Returns:
            List of dictionaries containing device information.
        """
        available_devices = []
        
        for channel in PCANChannel:
            try:
                # Try to get channel condition
                result = self._pcan_basic.GetValue(
                    channel.value, 
                    PCAN_CHANNEL_CONDITION
                )
                
                if result[0] == PCAN_ERROR_OK:
                    condition = result[1]
                    if condition & PCAN_CHANNEL_AVAILABLE:
                        # Get device information
                        device_info = {
                            'channel': channel.name,
                            'channel_value': channel.value,
                            'available': True,
                            'occupied': bool(condition & PCAN_CHANNEL_OCCUPIED)
                        }
                        
                        # Try to get device number
                        result = self._pcan_basic.GetValue(
                            channel.value,
                            PCAN_DEVICE_NUMBER
                        )
                        if result[0] == PCAN_ERROR_OK:
                            device_info['device_number'] = result[1]
                        
                        available_devices.append(device_info)
            except Exception as e:
                # Channel not available or error occurred
                pass
        
        return available_devices
    
    def connect(self, channel: PCANChannel, baudrate: PCANBaudRate, 
                fd_mode: bool = False) -> bool:
        """
        Connect to a PCAN device.
        
        Args:
            channel: PCAN channel to connect to (e.g., PCANChannel.USB1)
            baudrate: CAN bus baudrate (e.g., PCANBaudRate.BAUD_500K)
            fd_mode: Enable CAN FD mode (default: False)
        
        Returns:
            True if connection successful, False otherwise.
        """
        if self._is_connected:
            print("Already connected to a PCAN device. Disconnect first.")
            return False
        
        try:
            # Map channel enum to string for python-can
            channel_str = channel.name.replace('USB', 'PCAN_USBBUS')
            
            # Map baudrate enum to integer bitrate
            baudrate_map = {
                PCANBaudRate.BAUD_1M: 1000000,
                PCANBaudRate.BAUD_800K: 800000,
                PCANBaudRate.BAUD_500K: 500000,
                PCANBaudRate.BAUD_250K: 250000,
                PCANBaudRate.BAUD_125K: 125000,
                PCANBaudRate.BAUD_100K: 100000,
                PCANBaudRate.BAUD_95K: 95000,
                PCANBaudRate.BAUD_83K: 83000,
                PCANBaudRate.BAUD_50K: 50000,
                PCANBaudRate.BAUD_47K: 47000,
                PCANBaudRate.BAUD_33K: 33000,
                PCANBaudRate.BAUD_20K: 20000,
                PCANBaudRate.BAUD_10K: 10000,
                PCANBaudRate.BAUD_5K: 5000,
            }
            
            bitrate = baudrate_map.get(baudrate, 500000)
            
            # Create bus instance
            self._bus = Bus(
                interface='pcan',
                channel=channel_str,
                bitrate=bitrate,
                fd=fd_mode
            )
            
            self._channel = channel
            self._baudrate = baudrate
            self._is_connected = True
            
            print(f"✓ Connected to {channel.name} at {bitrate} bps")
            return True
            
        except Exception as e:
            print(f"✗ Failed to connect: {str(e)}")
            return False
    
    def disconnect(self) -> bool:
        """
        Disconnect from the PCAN device.
        
        Returns:
            True if disconnection successful, False otherwise.
        """
        if not self._is_connected:
            print("Not connected to any PCAN device.")
            return False
        
        try:
            # Stop receive thread if running
            self.stop_receive_thread()
            
            # Shutdown bus
            if self._bus:
                self._bus.shutdown()
                self._bus = None
            
            self._is_connected = False
            self._channel = None
            self._baudrate = None
            
            print("✓ Disconnected from PCAN device")
            return True
            
        except Exception as e:
            print(f"✗ Failed to disconnect: {str(e)}")
            return False
    
    def send_message(self, can_id: int, data: bytes, 
                    is_extended: bool = False, 
                    is_remote: bool = False) -> bool:
        """
        Send a CAN message.
        
        Args:
            can_id: CAN identifier (11-bit for standard, 29-bit for extended)
            data: Message data (up to 8 bytes for CAN 2.0, 64 for CAN FD)
            is_extended: Use extended 29-bit identifier (default: False)
            is_remote: Send as remote frame (default: False)
        
        Returns:
            True if message sent successfully, False otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return False
        
        try:
            msg = Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=is_extended,
                is_remote_frame=is_remote
            )
            
            self._bus.send(msg)
            return True
            
        except Exception as e:
            print(f"✗ Failed to send message: {str(e)}")
            return False
    
    def read_message(self, timeout: float = 1.0) -> Optional[CANMessage]:
        """
        Read a CAN message from the bus.
        
        Args:
            timeout: Timeout in seconds (default: 1.0)
        
        Returns:
            CANMessage object if message received, None otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return None
        
        try:
            msg = self._bus.recv(timeout=timeout)
            
            if msg is None:
                return None
            
            return CANMessage(
                id=msg.arbitration_id,
                data=bytes(msg.data),
                timestamp=msg.timestamp,
                is_extended=msg.is_extended_id,
                is_remote=msg.is_remote_frame,
                is_error=msg.is_error_frame,
                is_fd=msg.is_fd,
                dlc=msg.dlc
            )
            
        except Exception as e:
            print(f"✗ Failed to read message: {str(e)}")
            return None
    
    def start_receive_thread(self, callback: Callable[[CANMessage], None]) -> bool:
        """
        Start a background thread to continuously receive messages.
        
        Args:
            callback: Function to call when a message is received.
                     Should accept a CANMessage parameter.
        
        Returns:
            True if thread started successfully, False otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return False
        
        if self._receive_thread and self._receive_thread.is_alive():
            print("✗ Receive thread already running")
            return False
        
        self._receive_callback = callback
        self._stop_receive = False
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True
        )
        self._receive_thread.start()
        
        print("✓ Receive thread started")
        return True
    
    def stop_receive_thread(self) -> bool:
        """
        Stop the background receive thread.
        
        Returns:
            True if thread stopped successfully, False otherwise.
        """
        if not self._receive_thread or not self._receive_thread.is_alive():
            return False
        
        self._stop_receive = True
        self._receive_thread.join(timeout=2.0)
        self._receive_thread = None
        self._receive_callback = None
        
        print("✓ Receive thread stopped")
        return True
    
    def _receive_loop(self):
        """Internal method for receiving messages in a loop."""
        while not self._stop_receive and self._is_connected:
            try:
                msg = self.read_message(timeout=0.1)
                if msg and self._receive_callback:
                    self._receive_callback(msg)
            except Exception as e:
                if not self._stop_receive:
                    print(f"Error in receive loop: {str(e)}")
    
    def reset_device(self) -> bool:
        """
        Reset the PCAN device.
        
        Returns:
            True if reset successful, False otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return False
        
        try:
            result = self._pcan_basic.Reset(self._channel.value)
            if result == PCAN_ERROR_OK:
                print("✓ Device reset successfully")
                return True
            else:
                print(f"✗ Failed to reset device: Error {result}")
                return False
        except Exception as e:
            print(f"✗ Failed to reset device: {str(e)}")
            return False
    
    def get_bus_status(self) -> dict:
        """
        Get the current status of the CAN bus.
        
        Returns:
            Dictionary containing bus status information.
        """
        if not self._is_connected:
            return {'connected': False, 'error': 'Not connected'}
        
        try:
            result = self._pcan_basic.GetStatus(self._channel.value)
            
            status = {
                'connected': True,
                'channel': self._channel.name,
                'baudrate': self._baudrate.name,
                'status_code': result
            }
            
            if result == PCAN_ERROR_OK:
                status['status'] = 'OK'
            elif result == PCAN_ERROR_BUSLIGHT:
                status['status'] = 'Bus Light Error'
            elif result == PCAN_ERROR_BUSHEAVY:
                status['status'] = 'Bus Heavy Error'
            elif result == PCAN_ERROR_BUSOFF:
                status['status'] = 'Bus Off'
            else:
                status['status'] = f'Error: {result}'
            
            return status
            
        except Exception as e:
            return {'connected': True, 'error': str(e)}
    
    def set_filter(self, from_id: int, to_id: int, is_extended: bool = False) -> bool:
        """
        Set an acceptance filter for received messages.
        
        Args:
            from_id: Start of ID range
            to_id: End of ID range
            is_extended: Filter for extended IDs (default: False)
        
        Returns:
            True if filter set successfully, False otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return False
        
        try:
            filter_mode = PCAN_MODE_EXTENDED if is_extended else PCAN_MODE_STANDARD
            
            result = self._pcan_basic.FilterMessages(
                self._channel.value,
                from_id,
                to_id,
                filter_mode
            )
            
            if result == PCAN_ERROR_OK:
                print(f"✓ Filter set: 0x{from_id:X} - 0x{to_id:X}")
                return True
            else:
                print(f"✗ Failed to set filter: Error {result}")
                return False
                
        except Exception as e:
            print(f"✗ Failed to set filter: {str(e)}")
            return False
    
    def clear_receive_queue(self) -> bool:
        """
        Clear the receive queue.
        
        Returns:
            True if queue cleared successfully, False otherwise.
        """
        if not self._is_connected:
            print("✗ Not connected to PCAN device")
            return False
        
        try:
            # Read all pending messages
            count = 0
            while self.read_message(timeout=0.01):
                count += 1
            
            print(f"✓ Cleared {count} messages from queue")
            return True
            
        except Exception as e:
            print(f"✗ Failed to clear queue: {str(e)}")
            return False
    
    @property
    def is_connected(self) -> bool:
        """Check if connected to a PCAN device."""
        return self._is_connected
    
    @property
    def channel(self) -> Optional[PCANChannel]:
        """Get the current channel."""
        return self._channel
    
    @property
    def baudrate(self) -> Optional[PCANBaudRate]:
        """Get the current baudrate."""
        return self._baudrate
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup."""
        self.disconnect()
    
    def __del__(self):
        """Destructor - ensures cleanup."""
        if self._is_connected:
            self.disconnect()


# Example usage and testing
if __name__ == "__main__":
    print("=" * 60)
    print("PCAN Driver Test")
    print("=" * 60)
    
    # Create driver instance
    driver = PCANDriver()
    
    # Scan for available devices
    print("\n1. Scanning for PCAN devices...")
    devices = driver.get_available_devices()
    
    if not devices:
        print("✗ No PCAN devices found!")
        print("  Make sure your PCAN-USB adapter is connected.")
        exit(1)
    
    print(f"✓ Found {len(devices)} device(s):")
    for dev in devices:
        status = "OCCUPIED" if dev['occupied'] else "AVAILABLE"
        print(f"  - {dev['channel']}: {status}")
    
    # Connect to first available device
    print("\n2. Connecting to device...")
    first_device = devices[0]
    channel = PCANChannel[first_device['channel']]
    
    if driver.connect(channel, PCANBaudRate.BAUD_500K):
        print(f"✓ Successfully connected!")
        
        # Get bus status
        print("\n3. Checking bus status...")
        status = driver.get_bus_status()
        print(f"  Status: {status.get('status', 'Unknown')}")
        
        # Example: Send a message
        print("\n4. Sending test message...")
        test_data = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])
        if driver.send_message(0x123, test_data):
            print(f"✓ Message sent: ID=0x123, Data={test_data.hex()}")
        
        # Example: Read messages
        print("\n5. Listening for messages (5 seconds)...")
        print("  (Send some CAN messages to see them here)")
        
        def message_handler(msg: CANMessage):
            print(f"  Received: {msg}")
        
        driver.start_receive_thread(message_handler)
        time.sleep(5)
        driver.stop_receive_thread()
        
        # Disconnect
        print("\n6. Disconnecting...")
        driver.disconnect()
    else:
        print("✗ Failed to connect!")
    
    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)
