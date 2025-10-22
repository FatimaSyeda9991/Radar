#!/usr/bin/env python3
import smbus2
import time
import struct
import RPi.GPIO as GPIO

# ============================
# XM125(A121) PCR Radar Module
# Python Script for Jetson Orin Nano
# ============================

# Constants
I2C_ADDR = 0x52  # Default I2C address
I2C_BUS = 1      # Jetson I2C bus 1 (pins 3, 5)

# GPIO Pins for Jetson Orin Nano - USING BOARD NUMBERING
# Based on your pinout:
# Pin 3 = I2C1_SDA (I2C Bus 7)
# Pin 5 = I2C1_SCL (I2C Bus 7)
# Pin 7 = GPIO09 (GPIO492) - Using for WAKE_UP
# Pin 11 = UART1_RTS (GPIO460) - Using for INT
WAKE_UP_PIN = 7   # Physical pin 7 (GPIO09/GPIO492)
INT_PIN = 11      # Physical pin 11 (UART1_RTS/GPIO460)

# Measurement Interval
MEASUREMENT_INTERVAL = 0.1  # seconds
POLL_INTERVAL = 0.01        # seconds

# Initialize I2C
bus = smbus2.SMBus(I2C_BUS)

# Initialize GPIO - USING BOARD NUMBERING
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set initial state
GPIO.output(WAKE_UP_PIN, GPIO.LOW)
print(f"Initialized WAKE_UP_PIN (Pin {WAKE_UP_PIN}) to LOW")
print(f"INT_PIN on Pin {INT_PIN}")

def read_unsigned_register(reg_addr):
    """
    Reads a 32-bit unsigned register from the XM125 module.
    """
    try:
        # Convert register address to bytes
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        # Write register address
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        
        # Read 4 bytes
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        value = struct.unpack('>I', bytes(data))[0]
        return value
    except Exception as e:
        print(f"Read unsigned register error: {e}")
        return None

def read_signed_register(reg_addr):
    """
    Reads a 32-bit signed register from the XM125 module.
    """
    try:
        # Convert register address to bytes
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        # Write register address
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        
        # Read 4 bytes
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        value = struct.unpack('>i', bytes(data))[0]
        return value
    except Exception as e:
        print(f"Read signed register error: {e}")
        return None

def write_register(reg_addr, value):
    """
    Writes a 32-bit value to a register in the XM125 module.
    """
    try:
        # Convert register address and value to bytes
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        value_bytes = struct.pack('>I', value)
        
        # Write data (register address + value)
        data = [addr_low] + list(value_bytes)
        bus.write_i2c_block_data(I2C_ADDR, addr_high, data)
        return True
    except Exception as e:
        print(f"Write register error: {e}")
        return False

def wait_for_int(timeout=10):
    """
    Waits for the INT pin to go HIGH indicating the module is ready.
    """
    start_time = time.time()
    print("Waiting for INT pin to go HIGH...")
    
    while time.time() - start_time < timeout:
        int_state = GPIO.input(INT_PIN)
        print(f"INT pin state: {int_state} (1=HIGH, 0=LOW)")
        if int_state == GPIO.HIGH:
            print("INT pin is HIGH - module is ready")
            return True
        time.sleep(POLL_INTERVAL)
    
    print("Timeout waiting for INT pin.")
    return False

def initialize_module():
    """
    Initializes the XM125 module by waking it up.
    """
    print(f"Setting WAKE_UP_PIN (Pin {WAKE_UP_PIN}) to HIGH")
    
    # Set WAKE UP high
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    print("WAKE UP set to HIGH.")

    # Wait for INT pin to go HIGH indicating ready state
    if not wait_for_int():
        print("Module did not become ready.")
        return False
    
    print("Module is ready for I2C communication.")
    return True

def check_i2c_connection():
    """
    Check if we can communicate with the XM125 module over I2C.
    """
    try:
        # Try to read the version register
        version = read_unsigned_register(0x0000)
        if version is not None:
            print(f"Module Version: 0x{version:08X}")
            return True
        else:
            print("Failed to read module version")
            return False
    except Exception as e:
        print(f"I2C connection check failed: {e}")
        return False

def test_gpio():
    """
    Test GPIO functionality
    """
    print("Testing GPIO pins...")
    
    # Test WAKE_UP_PIN output
    print(f"Testing WAKE_UP_PIN (Pin {WAKE_UP_PIN})...")
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    time.sleep(1)
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    print("WAKE_UP_PIN toggled successfully")
    
    # Test INT_PIN input
    print(f"Testing INT_PIN (Pin {INT_PIN})...")
    int_state = GPIO.input(INT_PIN)
    print(f"INT_PIN state: {int_state}")
    
    return True

def cleanup():
    """Clean up GPIO"""
    print("Cleaning up GPIO...")
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    GPIO.cleanup()
    bus.close()
    print("Cleanup completed")

def main():
    """
    Main function to initialize and run radar measurements.
    """
    try:
        print("Starting XM125 Radar Module Initialization")
        print("=" * 50)
        
        # First, test GPIO functionality
        print("1. Testing GPIO pins...")
        test_gpio()
        
        # Scan I2C bus
        print("\n2. Scanning I2C devices...")
        devices_found = []
        for address in range(0x08, 0x78):
            try:
                bus.read_byte(address)
                devices_found.append(address)
                print(f"   Device found at: 0x{address:02X}")
            except:
                pass
        
        if not devices_found:
            print("   No I2C devices found! Check connections.")
            return
        
        if I2C_ADDR not in devices_found:
            print(f"   XM125 module not found at 0x{I2C_ADDR:02X}")
            print(f"   Available devices: {[hex(addr) for addr in devices_found]}")
            return
        
        print(f"   ✓ XM125 module found at 0x{I2C_ADDR:02X}")
        
        # Initialize module
        print("\n3. Initializing module...")
        if not initialize_module():
            print("   ✗ Failed to initialize module")
            return
        
        # Check I2C communication
        print("\n4. Checking I2C communication...")
        if not check_i2c_connection():
            print("   ✗ Failed to communicate with module over I2C")
            return
        
        print("\n5. ✓ Module initialized successfully!")
        print("   Ready for measurements...")
        
        # Simple measurement loop
        print("\n6. Starting measurement loop...")
        counter = 0
        while counter < 10:  # Run for 10 measurements
            print(f"\nMeasurement {counter + 1}:")
            
            # Add your actual measurement logic here
            # For now, just simulate
            print("   Taking measurement...")
            
            # Check INT pin state
            int_state = GPIO.input(INT_PIN)
            print(f"   INT pin state: {int_state}")
            
            counter += 1
            time.sleep(MEASUREMENT_INTERVAL)
            
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == "__main__":
    main()
