#!/usr/bin/env python3
import smbus2
import time
import struct
import RPi.GPIO as GPIO

# ============================
# XM125(A121) PCR Radar Module
# Python Script for Jetson Orin Nano
# Converted from MicroPython
# ============================

# Constants
I2C_ADDR = 0x52  # Default I2C address
I2C_BUS = 1      # Jetson I2C bus 1

# Register Addresses (same as your code)
REGISTER_MAP = {
    0x0000: 'Version',
    0x0001: 'Protocol Status',
    0x0002: 'Measure Counter',
    0x0003: 'Detector Status',
    0x0010: 'Distance Result',
    0x0011: 'Peak0 Distance',
    # ... (keep all your register definitions)
}

# Command Values (same as your code)
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2
# ... (keep all your commands)

# Enumerations (same as your code)
class ThresholdMethod:
    FIXED_AMPLITUDE = 1
    RECORDED = 2
    CFAR = 3
    FIXED_STRENGTH = 4

class PeakSortingMethod:
    CLOSEST = 1
    STRONGEST = 2

class ReflectorShape:
    GENERIC = 1
    PLANAR = 2

class Profile:
    PROFILE1 = 1
    PROFILE2 = 2
    PROFILE3 = 3
    PROFILE4 = 4
    PROFILE5 = 5

# Detector Configuration (same as your code)
DetectorConfig = {
    'start_m': 0.1,
    'end_m': 3.0,
    'max_step_length': 0,
    'max_profile': Profile.PROFILE1,
    'close_range_leakage_cancellation': False,
    'signal_quality': 30.0,
    'threshold_method': ThresholdMethod.CFAR,
    'peaksorting_method': PeakSortingMethod.CLOSEST,
    'reflector_shape': ReflectorShape.GENERIC,
    'num_frames_in_recorded_threshold': 100,
    'fixed_threshold_value': 100.0,
    'fixed_strength_threshold_value': 0.0,
    'threshold_sensitivity': 500,
    'measure_on_wakeup': False,
}

# GPIO Pins for Jetson Orin Nano
WAKE_UP_PIN = 12  # GPIO12 (BOARD numbering)
INT_PIN = 13      # GPIO13 (BOARD numbering)

# Measurement Interval
MEASUREMENT_INTERVAL = 0.1  # seconds
POLL_INTERVAL = 0.01        # seconds

# Initialize I2C
bus = smbus2.SMBus(I2C_BUS)

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(WAKE_UP_PIN, GPIO.LOW)  # Initially low

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
        if GPIO.input(INT_PIN) == GPIO.HIGH:
            return True
        time.sleep(POLL_INTERVAL)
    
    print("Timeout waiting for INT pin.")
    return False

def initialize_module():
    """
    Initializes the XM125 module by waking it up.
    """
    # Set WAKE UP high
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    print("WAKE UP set to HIGH.")

    # Wait for INT pin to go HIGH indicating ready state
    if not wait_for_int():
        print("Module did not become ready.")
        return False
    
    print("Module is ready for I2C communication.")
    return True

def write_default_configuration():
    """
    Writes the default configuration registers to the XM125 module.
    """
    config_registers = {
        0x0040: int(DetectorConfig['start_m'] * 1000),  # Start (mm)
        0x0041: int(DetectorConfig['end_m'] * 1000),    # End (mm)
        0x0042: DetectorConfig['max_step_length'],
        0x0043: int(DetectorConfig['close_range_leakage_cancellation']),
        0x0044: int(DetectorConfig['signal_quality'] * 1000),
        0x0045: DetectorConfig['max_profile'],
        0x0046: DetectorConfig['threshold_method'],
        0x0047: DetectorConfig['peaksorting_method'],
        0x0048: DetectorConfig['num_frames_in_recorded_threshold'],
        0x0049: int(DetectorConfig['fixed_threshold_value'] * 1000),
        0x004A: DetectorConfig['threshold_sensitivity'],
        0x004B: DetectorConfig['reflector_shape'],
        0x004C: int(DetectorConfig['fixed_strength_threshold_value'] * 1000),
        0x0080: int(DetectorConfig['measure_on_wakeup']),
    }

    print("Writing default configuration...")
    for reg, val in config_registers.items():
        if not write_register(reg, val):
            print(f"Failed to write to register 0x{reg:04X}.")
            return False
        print(f"Register 0x{reg:04X} set to {val}.")
    
    return True

def apply_config_and_calibrate():
    """
    Applies configuration settings and calibrates the module.
    """
    if not write_default_configuration():
        print("Failed to write default configuration.")
        return False

    # Send APPLY_CONFIG_AND_CALIBRATE command
    if not write_register(0x0100, COMMAND_APPLY_CONFIG_AND_CALIBRATE):
        print("Failed to send APPLY_CONFIG_AND_CALIBRATE command.")
        return False
    
    print("APPLY_CONFIG_AND_CALIBRATE command sent.")

    # Wait for BUSY bit to clear
    print("Waiting for module to finish configuration and calibration...")
    while True:
        status = read_unsigned_register(0x0003)  # Detector Status
        if status is None:
            print("Failed to read Detector Status.")
            return False
        if not (status & 0x80000000):  # BUSY mask
            break
        time.sleep(POLL_INTERVAL)
    
    print("Configuration and calibration completed.")

    # Check for Detector Errors
    error_flags = (status >> 16) & 0xFF
    if error_flags != 0:
        print(f"Detector Status Error Flags: 0x{error_flags:02X}")
        return False
    
    return True

def measure_distance():
    """
    Triggers a distance measurement and reads the result.
    """
    # Send MEASURE_DISTANCE command
    if not write_register(0x0100, COMMAND_MEASURE_DISTANCE):
        print("Failed to send MEASURE_DISTANCE command.")
        return (None, None)
    
    print("MEASURE_DISTANCE command sent.")

    # Wait for BUSY bit to clear
    print("Waiting for measurement to complete...")
    while True:
        status = read_unsigned_register(0x0003)
        if status is None:
            print("Failed to read Detector Status.")
            return (None, None)
        if not (status & 0x80000000):
            break
        time.sleep(POLL_INTERVAL)
    
    print("Measurement completed.")

    # Read Distance Result
    distance_result = read_unsigned_register(0x0010)
    if distance_result is None:
        print("Failed to read Distance Result.")
        return (None, None)

    # Parse Distance Result (same logic as your code)
    num_distances = distance_result & 0x0000000F
    measure_distance_error = (distance_result >> 10) & 0x1

    if measure_distance_error:
        print("Measurement failed (MEASURE_DISTANCE_ERROR).")
        return (None, None)

    if num_distances == 0:
        print("No peaks detected.")
        return ([], [])

    # Read peaks
    distances_m = []
    strengths_db = []
    
    for i in range(min(num_distances, 10)):
        peak_dist_addr = 0x0011 + i
        peak_str_addr = 0x001B + i

        distance_raw = read_unsigned_register(peak_dist_addr)
        strength_raw = read_signed_register(peak_str_addr)

        if distance_raw is not None and strength_raw is not None:
            distance_m = distance_raw / 1000.0
            strength_db_val = strength_raw / 1000.0

            distances_m.append(distance_m)
            strengths_db.append(strength_db_val)
            print(f"Peak{i} - Distance: {distance_m:.3f} m | Strength: {strength_db_val:.3f} dB")
    
    return (distances_m, strengths_db)

def cleanup():
    """Clean up GPIO"""
    GPIO.cleanup()
    bus.close()

def main():
    """
    Main function to initialize and run radar measurements.
    """
    try:
        # Scan I2C bus
        print("Scanning I2C devices...")
        for address in range(0x08, 0x78):
            try:
                bus.read_byte(address)
                print(f"Device found at: 0x{address:02X}")
            except:
                pass

        if not initialize_module():
            return

        if not apply_config_and_calibrate():
            return

        print("Entering measurement loop. Press Ctrl+C to exit.")
        while True:
            distances, strengths = measure_distance()
            if distances and strengths:
                for idx, (dist, stren) in enumerate(zip(distances, strengths)):
                    print(f"Peak{idx}: Distance = {dist:.3f} m | Strength = {stren:.3f} dB")
                print("")
            time.sleep(MEASUREMENT_INTERVAL)

    except KeyboardInterrupt:
        print("Measurement stopped by user.")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
