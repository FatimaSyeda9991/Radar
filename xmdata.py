#!/usr/bin/env python3
import smbus2
import time
import struct
import RPi.GPIO as GPIO

# ============================
# XM125(A121) PCR Radar Module
# Short-Range Floor Detection Configuration
# ============================

# Constants
I2C_ADDR = 0x52
I2C_BUS = 7

# GPIO Pins
WAKE_UP_PIN = 7
INT_PIN = 11

# Register Addresses
REGISTERS = {
    'DETECTOR_START': 0x0040,
    'DETECTOR_END': 0x0041,
    'MAX_STEP_LENGTH': 0x0042,
    'CLOSE_RANGE_LEAKAGE_CANCELLATION': 0x0043,
    'SIGNAL_QUALITY': 0x0044,
    'MAX_PROFILE': 0x0045,
    'THRESHOLD_METHOD': 0x0046,
    'PEAK_SORTING': 0x0047,
    'REFLECTOR_SHAPE': 0x004B,
    'MEASURE_ON_WAKEUP': 0x0080,
    'COMMAND': 0x0100,
}

# Command Values
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2

# Profile Types
PROFILE_1 = 1  # Short range, high resolution
PROFILE_2 = 2  # Medium range
PROFILE_3 = 3  # Long range
PROFILE_4 = 4  # Low power
PROFILE_5 = 5  # High speed

# Threshold Methods
THRESHOLD_CFAR = 3  # Constant False Alarm Rate - best for dynamic environments

# Peak Sorting
PEAK_SORTING_STRONGEST = 2  # Show strongest reflections first

# Reflector Shapes
REFLECTOR_GENERIC = 1
REFLECTOR_PLANAR = 2  # Better for flat surfaces like floors

# Initialize I2C and GPIO
bus = smbus2.SMBus(I2C_BUS)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_unsigned_register(reg_addr):
    """Reads a 32-bit unsigned register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        return struct.unpack('>I', bytes(data))[0]
    except Exception as e:
        print(f"Read error 0x{reg_addr:04X}: {e}")
        return None

def read_signed_register(reg_addr):
    """Reads a 32-bit signed register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        bus.write_byte_data(I2C_ADDR, addr_high, addr_low)
        time.sleep(0.01)
        data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 4)
        return struct.unpack('>i', bytes(data))[0]
    except Exception as e:
        print(f"Read signed error 0x{reg_addr:04X}: {e}")
        return None

def write_register(reg_addr, value):
    """Writes a 32-bit value to a register."""
    try:
        addr_high = (reg_addr >> 8) & 0xFF
        addr_low = reg_addr & 0xFF
        value_bytes = struct.pack('>I', value)
        data = [addr_low] + list(value_bytes)
        bus.write_i2c_block_data(I2C_ADDR, addr_high, data)
        return True
    except Exception as e:
        print(f"Write error 0x{reg_addr:04X}: {e}")
        return False

def wait_for_int(timeout=5):
    """Waits for INT pin to go HIGH."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        if GPIO.input(INT_PIN) == GPIO.HIGH:
            return True
        time.sleep(0.1)
    print("INT timeout - continuing anyway")
    return False

def initialize_module():
    """Initializes the XM125 module."""
    print("Initializing module...")
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    time.sleep(0.5)
    wait_for_int()
    return True

def configure_short_range_floor_detection():
    """
    Configures the radar for short-range floor detection.
    This is optimized for detecting objects under a rover.
    """
    print("Configuring for short-range floor detection...")
    
    # Configuration for floor detection (values in mm where applicable)
    config = {
        REGISTERS['DETECTOR_START']: 50,           # 5cm min distance
        REGISTERS['DETECTOR_END']: 1000,           # 1m max distance
        REGISTERS['MAX_STEP_LENGTH']: 0,           # Auto step length
        REGISTERS['CLOSE_RANGE_LEAKAGE_CANCELLATION']: 1,  # Enable for close range
        REGISTERS['SIGNAL_QUALITY']: 15000,        # Medium sensitivity (15.0 * 1000)
        REGISTERS['MAX_PROFILE']: PROFILE_1,       # Short range profile
        REGISTERS['THRESHOLD_METHOD']: THRESHOLD_CFAR,
        REGISTERS['PEAK_SORTING']: PEAK_SORTING_STRONGEST,
        REGISTERS['REFLECTOR_SHAPE']: REFLECTOR_PLANAR,  # Better for flat surfaces
        REGISTERS['MEASURE_ON_WAKEUP']: 0,         # Don't measure on wakeup
    }
    
    # Write configuration
    for reg, value in config.items():
        if not write_register(reg, value):
            print(f"Failed to write config register 0x{reg:04X}")
            return False
        print(f"Config 0x{reg:04X} = {value}")
        time.sleep(0.01)
    
    return True

def apply_configuration():
    """Applies configuration and calibrates the module."""
    print("Applying configuration and calibrating...")
    
    if not write_register(REGISTERS['COMMAND'], COMMAND_APPLY_CONFIG_AND_CALIBRATE):
        print("Failed to send calibration command")
        return False
    
    # Wait for calibration to complete
    timeout = 10
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_unsigned_register(0x0003)  # Detector status
        if status is not None:
            busy = (status >> 31) & 0x1
            if not busy:
                print("Calibration completed")
                
                # Check for errors
                error_flags = (status >> 16) & 0xFF
                if error_flags != 0:
                    print(f"Calibration errors: 0x{error_flags:02X}")
                    return False
                return True
        time.sleep(0.1)
    
    print("Calibration timeout")
    return False

def get_detection_data():
    """
    Gets comprehensive detection data including:
    - Distance to objects
    - Signal strength
    - Object size/characteristics
    - Environmental data
    """
    print("Getting detection data...")
    
    # Send measurement command
    if not write_register(REGISTERS['COMMAND'], COMMAND_MEASURE_DISTANCE):
        print("Failed to send measurement command")
        return None
    
    # Wait for measurement
    timeout = 3
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_unsigned_register(0x0003)
        if status is not None:
            busy = (status >> 31) & 0x1
            if not busy:
                break
        time.sleep(0.1)
    else:
        print("Measurement timeout")
        return None
    
    # Read comprehensive data
    detection_data = {}
    
    # Basic distance result
    distance_result = read_unsigned_register(0x0010)
    if distance_result is not None:
        detection_data['num_targets'] = distance_result & 0x0000000F
        detection_data['measurement_error'] = (distance_result >> 10) & 0x1
    
    # Read individual peaks/targets
    targets = []
    max_targets = min(detection_data.get('num_targets', 0), 10)
    
    for i in range(max_targets):
        target = {}
        
        # Distance
        dist_raw = read_unsigned_register(0x0011 + i)
        if dist_raw is not None:
            target['distance_m'] = dist_raw / 1000.0
            target['distance_cm'] = dist_raw / 10.0
        
        # Signal strength
        strength_raw = read_signed_register(0x001B + i)
        if strength_raw is not None:
            target['strength_db'] = strength_raw / 1000.0
        
        # Additional target characteristics
        # You can read more registers here for additional data
        targets.append(target)
    
    detection_data['targets'] = targets
    
    # Read environmental data
    ambient_data = read_signed_register(0x0025)  # Ambient level
    if ambient_data is not None:
        detection_data['ambient_level'] = ambient_data / 1000.0
    
    # Read measurement counter
    measure_counter = read_unsigned_register(0x0002)
    if measure_counter is not None:
        detection_data['measurement_count'] = measure_counter
    
    return detection_data

def interpret_detection_data(data):
    """Interprets the detection data for rover floor analysis."""
    if not data or 'targets' not in data:
        return "No detection data"
    
    interpretation = []
    
    if data.get('measurement_error'):
        interpretation.append("MEASUREMENT ERROR DETECTED")
    
    num_targets = data.get('num_targets', 0)
    interpretation.append(f"Detected {num_targets} object(s) under rover")
    
    for i, target in enumerate(data.get('targets', [])):
        dist_cm = target.get('distance_cm', 0)
        strength = target.get('strength_db', 0)
        
        # Interpret based on distance and strength
        if dist_cm < 10:
            obj_type = "VERY CLOSE OBJECT/OBSTRUCTION"
            urgency = "HIGH"
        elif dist_cm < 30:
            obj_type = "Close object"
            urgency = "MEDIUM"
        elif dist_cm < 50:
            obj_type = "Medium range object"
            urgency = "LOW"
        else:
            obj_type = "Distant object/floor"
            urgency = "INFO"
        
        # Interpret signal strength
        if strength > 100:
            material = "Strong reflector (metal, dense material)"
        elif strength > 50:
            material = "Medium reflector (plastic, wood)"
        else:
            material = "Weak reflector (fabric, organic)"
        
        interpretation.append(
            f"Target {i+1}: {dist_cm:.1f}cm - {obj_type} - {material} - {urgency}"
        )
    
    if num_targets == 0:
        interpretation.append("Clear path under rover")
    
    return interpretation

def main():
    """Main function for rover floor detection."""
    try:
        print("Rover Floor Detection System")
        print("=" * 40)
        
        # Initialize
        if not initialize_module():
            return
        
        # Configure for floor detection
        if not configure_short_range_floor_detection():
            return
        
        # Apply configuration
        if not apply_configuration():
            return
        
        print("\nStarting floor detection...")
        print("Press Ctrl+C to stop\n")
        
        # Continuous detection
        while True:
            # Get detection data
            data = get_detection_data()
            
            if data:
                # Interpret and display results
                interpretation = interpret_detection_data(data)
                
                print("\n" + "=" * 50)
                print(f"Measurement #{data.get('measurement_count', 0)}")
                print("=" * 50)
                
                for line in interpretation:
                    print(line)
                
                # Raw data for debugging
                print("\nRaw data:")
                print(f"  Targets: {data.get('num_targets', 0)}")
                for i, target in enumerate(data.get('targets', [])):
                    print(f"  Target {i}: {target.get('distance_cm', 0):.1f}cm, "
                          f"{target.get('strength_db', 0):.1f}dB")
                if 'ambient_level' in data:
                    print(f"  Ambient: {data['ambient_level']:.1f}dB")
            
            print("\n" + "-" * 50)
            time.sleep(0.5)  # Faster updates for rover navigation
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        GPIO.output(WAKE_UP_PIN, GPIO.LOW)
        GPIO.cleanup()
        bus.close()

if __name__ == "__main__":
    main()
