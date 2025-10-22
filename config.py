#!/usr/bin/env python3
import smbus2
import time
import struct
import RPi.GPIO as GPIO

# ============================
# XM125(A121) PCR Radar Module
# 100cm Floor Detection Configuration
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
    'NUM_FRAMES_RECORDED_THRESHOLD': 0x0048,
    'FIXED_THRESHOLD_VALUE': 0x0049,
    'THRESHOLD_SENSITIVITY': 0x004A,
    'REFLECTOR_SHAPE': 0x004B,
    'FIXED_STRENGTH_THRESHOLD': 0x004C,
    'MEASURE_ON_WAKEUP': 0x0080,
    'COMMAND': 0x0100,
}

# Command Values
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2

# Profile Types
PROFILE_1 = 1
PROFILE_2 = 2
PROFILE_3 = 3
PROFILE_4 = 4
PROFILE_5 = 5

# Threshold Methods
THRESHOLD_FIXED_AMPLITUDE = 1
THRESHOLD_RECORDED = 2
THRESHOLD_CFAR = 3
THRESHOLD_FIXED_STRENGTH = 4

# Peak Sorting
PEAK_SORTING_CLOSEST = 1
PEAK_SORTING_STRONGEST = 2

# Reflector Shapes
REFLECTOR_GENERIC = 1
REFLECTOR_PLANAR = 2

# Calibration Error Bits
CALIBRATION_ERRORS = {
    0x01: "CPU error",
    0x02: "Data path error", 
    0x04: "Interference error",
    0x08: "Temperature error",
    0x10: "Profile configuration error",
    0x20: "Distance range error",
    0x40: "Hardware error",
    0x80: "Software error"
}

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
            print("✓ INT pin HIGH")
            return True
        time.sleep(0.1)
    print("⚠ INT timeout - continuing anyway")
    return False

def initialize_module():
    """Initializes the XM125 module."""
    print("Initializing module...")
    GPIO.output(WAKE_UP_PIN, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(WAKE_UP_PIN, GPIO.HIGH)
    time.sleep(1.0)
    wait_for_int()
    return True

def decode_calibration_errors(error_byte):
    """Decodes calibration error bits."""
    errors = []
    for bit, description in CALIBRATION_ERRORS.items():
        if error_byte & bit:
            errors.append(description)
    return errors if errors else ["No errors"]

def configure_for_100cm_range():
    """
    Configures the radar for 100cm range with optimized parameters.
    """
    print("Configuring for 100cm floor detection range...")
    
    # Optimized configuration for 100cm range
    config = {
        # Distance range (in mm) - 5cm to 100cm as requested
        REGISTERS['DETECTOR_START']: 50,       # 5cm min
        REGISTERS['DETECTOR_END']: 1000,       # 100cm max
        
        # Signal processing - optimized for 100cm
        REGISTERS['MAX_STEP_LENGTH']: 0,       # Auto step length
        REGISTERS['CLOSE_RANGE_LEAKAGE_CANCELLATION']: 1,  # Enable for close range
        REGISTERS['SIGNAL_QUALITY']: 12000,    # Balanced sensitivity (12.0 * 1000)
        
        # Profile - Profile 2 is good for medium range
        REGISTERS['MAX_PROFILE']: PROFILE_2,   # Medium range profile
        
        # Threshold settings - CFAR with moderate sensitivity
        REGISTERS['THRESHOLD_METHOD']: THRESHOLD_CFAR,
        REGISTERS['PEAK_SORTING']: PEAK_SORTING_STRONGEST,
        REGISTERS['THRESHOLD_SENSITIVITY']: 500,  # CFAR sensitivity
        
        # Reflector type - Generic for versatility
        REGISTERS['REFLECTOR_SHAPE']: REFLECTOR_GENERIC,
        
        # Recorded threshold frames
        REGISTERS['NUM_FRAMES_RECORDED_THRESHOLD']: 20,
        
        # Fixed threshold as fallback (not used with CFAR but set anyway)
        REGISTERS['FIXED_THRESHOLD_VALUE']: 4000,
        
        # Other settings
        REGISTERS['MEASURE_ON_WAKEUP']: 0,
    }
    
    # Write configuration
    success_count = 0
    for reg, value in config.items():
        if write_register(reg, value):
            success_count += 1
            print(f"✓ Config 0x{reg:04X} = {value}")
        else:
            print(f"✗ Failed: 0x{reg:04X} = {value}")
        time.sleep(0.02)
    
    return success_count == len(config)

def try_100cm_alternative_configs():
    """Try different configurations for 100cm range."""
    configs_to_try = [
        {
            'name': 'Profile 1 - High Resolution',
            'profile': PROFILE_1,
            'sensitivity': 10000,
            'threshold_method': THRESHOLD_FIXED_AMPLITUDE,
        },
        {
            'name': 'Profile 2 - Balanced', 
            'profile': PROFILE_2,
            'sensitivity': 12000,
            'threshold_method': THRESHOLD_CFAR,
        },
        {
            'name': 'Profile 3 - Long Range',
            'profile': PROFILE_3,
            'sensitivity': 15000,
            'threshold_method': THRESHOLD_CFAR,
        },
        {
            'name': 'Conservative Fixed Threshold',
            'profile': PROFILE_2,
            'sensitivity': 8000,
            'threshold_method': THRESHOLD_FIXED_AMPLITUDE,
        }
    ]
    
    for config in configs_to_try:
        print(f"\nTrying {config['name']} configuration...")
        
        alt_config = {
            REGISTERS['DETECTOR_START']: 50,      # 5cm
            REGISTERS['DETECTOR_END']: 1000,      # 100cm
            REGISTERS['MAX_PROFILE']: config['profile'],
            REGISTERS['THRESHOLD_METHOD']: config['threshold_method'],
            REGISTERS['SIGNAL_QUALITY']: config['sensitivity'],
            REGISTERS['CLOSE_RANGE_LEAKAGE_CANCELLATION']: 1,
            REGISTERS['PEAK_SORTING']: PEAK_SORTING_STRONGEST,
            REGISTERS['MAX_STEP_LENGTH']: 0,
        }
        
        # Add threshold-specific settings
        if config['threshold_method'] == THRESHOLD_FIXED_AMPLITUDE:
            alt_config[REGISTERS['FIXED_THRESHOLD_VALUE']] = 5000
        else:  # CFAR
            alt_config[REGISTERS['THRESHOLD_SENSITIVITY']] = 500
        
        # Write alternative config
        success = True
        for reg, value in alt_config.items():
            if not write_register(reg, value):
                success = False
                break
            time.sleep(0.02)
        
        if success and apply_configuration():
            print(f"✓ {config['name']} successful!")
            return True
        else:
            print(f"✗ {config['name']} failed")
    
    return False

def apply_configuration():
    """Applies configuration and calibrates the module."""
    print("Applying configuration and calibrating...")
    
    if not write_register(REGISTERS['COMMAND'], COMMAND_APPLY_CONFIG_AND_CALIBRATE):
        print("Failed to send calibration command")
        return False
    
    # Wait for calibration to complete
    timeout = 15
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
                    errors = decode_calibration_errors(error_flags)
                    print(f"Calibration errors (0x{error_flags:02X}): {errors}")
                    return False
                else:
                    print("✓ Calibration successful - no errors")
                    return True
        time.sleep(0.2)
    
    print("Calibration timeout")
    return False

def comprehensive_measurement():
    """Perform comprehensive measurement with multiple targets."""
    print("\nPerforming measurement...")
    
    if not write_register(REGISTERS['COMMAND'], COMMAND_MEASURE_DISTANCE):
        print("Failed to send measurement command")
        return None
    
    # Wait for measurement
    timeout = 5
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
    
    # Read distance result
    distance_result = read_unsigned_register(0x0010)
    if distance_result is None:
        print("Failed to read distance result")
        return None
    
    num_targets = distance_result & 0x0000000F
    measurement_error = (distance_result >> 10) & 0x1
    
    if measurement_error:
        print("Measurement error detected")
        return None
    
    print(f"Targets detected: {num_targets}")
    
    # Read all detected targets
    targets = []
    for i in range(min(num_targets, 5)):  # Read up to 5 targets
        dist_raw = read_unsigned_register(0x0011 + i)  # Peak distance
        strength_raw = read_signed_register(0x001B + i)  # Peak strength
        
        if dist_raw is not None and strength_raw is not None:
            distance_cm = dist_raw / 10.0
            strength_db = strength_raw / 1000.0
            targets.append({
                'distance_cm': distance_cm,
                'strength_db': strength_db
            })
    
    return {
        'num_targets': num_targets,
        'targets': targets
    }

def interpret_floor_data(data):
    """Interpret the data for rover floor analysis up to 100cm."""
    if not data or 'targets' not in data:
        return ["No targets detected within 100cm range"]
    
    interpretations = []
    num_targets = data.get('num_targets', 0)
    
    interpretations.append(f"Detected {num_targets} object(s) within 100cm range:")
    
    for i, target in enumerate(data.get('targets', [])):
        dist_cm = target.get('distance_cm', 0)
        strength = target.get('strength_db', 0)
        
        # Interpretation for floor detection
        if dist_cm <= 10:
            obj_type = "🚨 IMMEDIATE OBSTRUCTION"
            action = "STOP/AVOID"
        elif dist_cm <= 30:
            obj_type = "⚠ Close obstacle"
            action = "SLOW DOWN"
        elif dist_cm <= 50:
            obj_type = "Medium range object"
            action = "PROCEED WITH CAUTION"
        elif dist_cm <= 80:
            obj_type = "Distant object"
            action = "NORMAL OPERATION"
        else:  # 80-100cm
            obj_type = "Far object/floor boundary"
            action = "CLEAR PATH"
        
        # Material interpretation based on signal strength
        if strength > 100:
            material = "Strong reflector (metal, dense material)"
        elif strength > 50:
            material = "Medium reflector (plastic, wood)"
        else:
            material = "Weak reflector (fabric, carpet, organic)"
        
        interpretations.append(
            f"  Target {i+1}: {dist_cm:.1f}cm - {obj_type}"
            f" - {material} - {action}"
        )
    
    return interpretations

def main():
    """Main function for 100cm floor detection."""
    try:
        print("Rover Floor Detection System - 100cm Range")
        print("=" * 50)
        
        # Initialize
        if not initialize_module():
            print("Failed to initialize module")
            return
        
        # Try primary 100cm configuration
        if not configure_for_100cm_range():
            print("Failed to write initial configuration")
            return
        
        print("\nAttempting calibration with 100cm configuration...")
        if not apply_configuration():
            print("\nPrimary configuration failed, trying alternatives...")
            if not try_100cm_alternative_configs():
                print("All 100cm configuration attempts failed")
                return
        
        print("\n" + "=" * 50)
        print("✓ SYSTEM READY - 100cm FLOOR DETECTION ACTIVE")
        print("=" * 50)
        print("Starting continuous measurements...")
        print("Press Ctrl+C to stop\n")
        
        # Continuous measurements
        measurement_count = 0
        while True:
            measurement_count += 1
            print(f"\n--- Measurement #{measurement_count} ---")
            
            data = comprehensive_measurement()
            
            if data:
                interpretations = interpret_floor_data(data)
                for line in interpretations:
                    print(line)
                
                # Show raw data for debugging
                if data['num_targets'] > 0:
                    print("\nRaw data:")
                    for i, target in enumerate(data['targets']):
                        print(f"  Target {i}: {target['distance_cm']:.1f}cm, "
                              f"{target['strength_db']:.1f}dB")
            else:
                print("Measurement failed or no targets detected")
            
            time.sleep(0.5)  # Fast updates for rover navigation
            
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("Detection system stopped by user")
        print("=" * 50)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        GPIO.output(WAKE_UP_PIN, GPIO.LOW)
        GPIO.cleanup()
        bus.close()

if __name__ == "__main__":
    main()
