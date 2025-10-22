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
I2C_BUS = 7      # I2C bus 7 (from your successful detection)

# GPIO Pins for Jetson Orin Nano - USING BOARD NUMBERING
WAKE_UP_PIN = 7   # Physical pin 7 (GPIO09/GPIO492)
INT_PIN = 11      # Physical pin 11 (UART1_RTS/GPIO460)

# Register Addresses
REGISTER_MAP = {
    0x0000: 'Version',
    0x0001: 'Protocol Status',
    0x0002: 'Measure Counter', 
    0x0003: 'Detector Status',
    0x0010: 'Distance Result',
    0x0011: 'Peak0 Distance',
    0x001B: 'Peak0 Strength',
}

# Command Values
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2

# Initialize I2C
bus = smbus2.SMBus(I2C_BUS)

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_unsigned_register(reg_addr):
    """Reads a 32-bit unsigned register from the XM125 module."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        value = struct.unpack('>I', bytes(data))[0]
        return value
    except Exception as e:
        print(f"Read register 0x{reg_addr:04X} error: {e}")
        return None

def read_signed_register(reg_addr):
    """Reads a 32-bit signed register from the XM125 module."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        value = struct.unpack('>i', bytes(data))[0]
        return value
    except Exception as e:
        print(f"Read signed register 0x{reg_addr:04X} error: {e}")
        return None

def write_register(reg_addr, value):
    """Writes a 32-bit value to a register in the XM125 module."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        
        value_bytes = struct.pack('>I', value)
        data = [addr_low] + list(value_bytes)
        bus.write_i2c_block_data(I2C_ADDR, addr_high, data)
        return True
    except Exception as e:
        print(f"Write register 0x{reg_addr:04X} error: {e}")
        return False

def wait_for_int(timeout=5):
    """Waits for the INT pin to go HIGH indicating the module is ready."""
    start_time = time.time()
    print("Waiting for INT pin to go HIGH...")
    
    while time.time() - start_time < timeout:
        int_state = GPIO.input(INT_PIN)
        if int_state == GPIO.HIGH:
            print("✓ INT pin is HIGH - module is ready")
            return True
        time.sleep(0.1)
    
    print("✗ Timeout waiting for INT pin. Continuing anyway...")
    return False

def initialize_module():
    """Initializes the XM125 module by waking it up."""
    print("Initializing module...")
    
    # Ensure WAKE_UP is LOW first
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    time.sleep(0.1)
    
    # Set WAKE UP high
    print("Setting WAKE_UP_PIN to HIGH...")
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    time.sleep(0.5)  # Give more time for module to wake up
    
    # Wait for INT pin
    if wait_for_int():
        return True
    else:
        # Try to continue anyway - sometimes INT might not work
        print("Trying to continue without INT confirmation...")
        time.sleep(1)
        return True

def read_module_info():
    """Read basic module information to verify communication."""
    print("\nReading module information...")
    
    # Read version
    version = read_unsigned_register(0x0000)
    if version is not None:
        print(f"Module Version: 0x{version:08X}")
    else:
        print("Failed to read version")
        return False
    
    # Read protocol status
    protocol_status = read_unsigned_register(0x0001)
    if protocol_status is not None:
        print(f"Protocol Status: 0x{protocol_status:08X}")
    
    # Read detector status
    detector_status = read_unsigned_register(0x0003)
    if detector_status is not None:
        print(f"Detector Status: 0x{detector_status:08X}")
        busy = (detector_status >> 31) & 0x1
        print(f"  Busy: {busy}")
    
    return True

def simple_measurement():
    """Perform a simple distance measurement."""
    print("\nPerforming measurement...")
    
    # Send MEASURE_DISTANCE command
    if write_register(0x0100, COMMAND_MEASURE_DISTANCE):
        print("MEASURE_DISTANCE command sent")
    else:
        print("Failed to send MEASURE_DISTANCE command")
        return None, None
    
    # Wait for measurement to complete (check busy bit)
    timeout = 3
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_unsigned_register(0x0003)
        if status is not None:
            busy = (status >> 31) & 0x1
            if not busy:
                print("Measurement completed")
                break
        time.sleep(0.1)
    else:
        print("Measurement timeout")
        return None, None
    
    # Read distance result
    distance_result = read_unsigned_register(0x0010)
    if distance_result is None:
        print("Failed to read distance result")
        return None, None
    
    print(f"Distance Result: 0x{distance_result:08X}")
    
    # Parse distance result
    num_distances = distance_result & 0x0000000F
    measure_distance_error = (distance_result >> 10) & 0x1
    
    if measure_distance_error:
        print("Measurement error detected")
        return None, None
    
    print(f"Number of peaks detected: {num_distances}")
    
    # Read peaks
    distances_m = []
    strengths_db = []
    
    for i in range(min(num_distances, 5)):  # Read up to 5 peaks
        peak_dist_addr = 0x0011 + i
        peak_str_addr = 0x001B + i

        distance_raw = read_unsigned_register(peak_dist_addr)
        strength_raw = read_signed_register(peak_str_addr)

        if distance_raw is not None and strength_raw is not None:
            distance_m = distance_raw / 1000.0  # Convert mm to m
            strength_db_val = strength_raw / 1000.0  # Convert to dB

            distances_m.append(distance_m)
            strengths_db.append(strength_db_val)
            print(f"Peak{i}: Distance = {distance_m:.3f} m | Strength = {strength_db_val:.3f} dB")
    
    return distances_m, strengths_db

def continuous_measurements():
    """Perform continuous measurements."""
    print("\nStarting continuous measurements...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            distances, strengths = simple_measurement()
            
            if distances and strengths:
                print("--- Measurement Result ---")
                for idx, (dist, stren) in enumerate(zip(distances, strengths)):
                    print(f"Peak{idx}: {dist:.3f} m | {stren:.3f} dB")
                print("--------------------------\n")
            else:
                print("No targets detected or measurement failed\n")
            
            time.sleep(1)  # Wait 1 second between measurements
            
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user")

def cleanup():
    """Clean up GPIO and I2C."""
    print("Cleaning up...")
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    GPIO.cleanup()
    bus.close()

def main():
    """Main function."""
    try:
        print("XM125 Radar Module - Data Acquisition")
        print("=" * 40)
        
        # Initialize module
        if not initialize_module():
            print("Failed to initialize module")
            return
        
        # Read module info
        if not read_module_info():
            print("Failed to read module information")
            return
        
        # Start continuous measurements
        continuous_measurements()
        
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == "__main__":
    main()
