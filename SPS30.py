"""
SPS30 Particulate Matter Sensor Library for MicroPython (Enhanced CRC Validation)
==================================================================================

SPS30 library with CRC validation

Copyright 2025 Jesse Anderson

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
# IEEE 754 nonsense due to fears sparked from: https://forum.arduino.cc/t/sensirion-sps30-particle-sensor/568629

from machine import I2C, Pin
from time import sleep, ticks_ms
import gc #compat issues with gc in older micropython, i dont need random issues

collect = gc.collect
mem_free = getattr(gc, "mem_free", lambda: 0)

#from collections import deque #Regulatory guidance on getting rolling average... # ESP32 doesn't like deque
class SPS30:
    """Enhanced SPS30 Library with Rigorous CRC Validation"""
    
    # I2C Constants
    I2C_ADDRESS = 0x69
    
    # =============================================================================
    # SPS30 I2C COMMAND CONSTANTS - COMPREHENSIVE EXPLANATION
    # =============================================================================
    
    # IMPORTANT: Commands are stored as lists [MSB, LSB] for dynamic building
    # 
    # WHY LISTS INSTEAD OF 16-BIT INTEGERS?
    # - SPS30 I2C protocol requires commands + optional arguments + CRCs
    # - Lists allow easy appending: CMD_START_MEASUREMENT + [arg1, arg2, crc]
    # - Final command becomes: [0x00, 0x10, 0x03, 0x00, 0x76] for start measurement
    # - Alternative would require complex bit manipulation for each command type
    #
    # I2C PROTOCOL STRUCTURE:
    # Basic command:     [CMD_MSB, CMD_LSB]
    # Command with arg:  [CMD_MSB, CMD_LSB, ARG1, ARG2, CRC8(ARG1,ARG2)]
    # Command with 32bit:[CMD_MSB, CMD_LSB, B1, B2, CRC8(B1,B2), B3, B4, CRC8(B3,B4)]
    
    # MEASUREMENT CONTROL COMMANDS
    # ============================
    
    CMD_START_MEASUREMENT = [0x00, 0x10]
    # Purpose: Start continuous particulate matter measurement
    # Usage: Requires argument for output format:
    #        - 0x0300 = IEEE754 32-bit float format (recommended)
    #        - 0x0500 = 16-bit integer format (less precise)
    # Final command: [0x00, 0x10, 0x03, 0x00, 0x76] (with CRC)
    # Response: None (command only)
    # Notes: Fan starts spinning, draws ~60mA, takes ~8 seconds to stabilize
    
    CMD_STOP_MEASUREMENT = [0x01, 0x04]
    # Purpose: Stop continuous measurement and turn off fan
    # Usage: Simple command, no arguments needed
    # Final command: [0x01, 0x04]
    # Response: None (command only)
    # Notes: Fan stops, power drops to <8mA, measurement data becomes invalid
    
    CMD_READ_DATA_READY_FLAG = [0x02, 0x02]
    # Purpose: Check if new measurement data is available for reading
    # Usage: Simple command, no arguments needed
    # Final command: [0x02, 0x02]
    # Response: 3 bytes [0x00, ready_flag, CRC] where ready_flag=1 means data ready
    # Notes: Should be called before CMD_READ_MEASURED_VALUES to avoid stale data
    
    CMD_READ_MEASURED_VALUES = [0x03, 0x00]
    # Purpose: Read all 10 measurement values (PM + particle counts + size)
    # Usage: Simple command, no arguments needed
    # Final command: [0x03, 0x00]
    # Response: 60 bytes total:
    #   - 4 PM mass concentrations (PM1.0, PM2.5, PM4.0, PM10) = 24 bytes
    #   - 5 particle number concentrations (0.5-10μm ranges) = 30 bytes  
    #   - 1 typical particle size = 6 bytes
    #   Each float: [B1, B2, CRC, B3, B4, CRC] = 6 bytes per measurement
    # Notes: Data only valid if measurement is running (after start command)
    
    # POWER MANAGEMENT COMMANDS
    # =========================
    
    CMD_SLEEP = [0x10, 0x01]
    # Purpose: Put sensor into sleep mode (ultra-low power)
    # Usage: Simple command, no arguments needed
    # Final command: [0x10, 0x01]
    # Response: None (command only)
    # Notes: Current drops to <0.5mA, I2C remains active, fan stops
    #        Wake with CMD_WAKEUP, takes ~50ms to become ready
    
    CMD_WAKEUP = [0x11, 0x03]
    # Purpose: Wake sensor from sleep mode
    # Usage: Simple command, no arguments needed
    # Final command: [0x11, 0x03]
    # Response: None (command only)
    # Notes: Returns to idle state (~8mA), ready for measurement commands
    #        Wait 50ms after this command before sending other commands
    
    # MAINTENANCE COMMANDS
    # ====================
    
    CMD_START_FAN_CLEANING = [0x56, 0x07]
    # Purpose: Manually trigger fan cleaning cycle (removes dust)
    # Usage: Simple command, no arguments needed
    # Final command: [0x56, 0x07]
    # Response: None (command only)
    # Notes: Only works during measurement mode, takes ~10 seconds
    #        Fan runs at high speed to blow out accumulated particles
    #        Measurement data invalid during cleaning cycle
    
    CMD_AUTO_CLEANING_INTERVAL = [0x80, 0x04]
    # Purpose: Read OR write automatic cleaning interval setting
    # Usage: READ:  Simple command [0x80, 0x04]
    #        WRITE: Command + 32-bit seconds value + CRCs
    #               [0x80, 0x04, B1, B2, CRC(B1,B2), B3, B4, CRC(B3,B4)]
    # Response: READ: 6 bytes [B1, B2, CRC, B3, B4, CRC] = 32-bit seconds
    #           WRITE: None (command only)
    # Notes: Default = 604800 seconds (7 days), 0 = disabled
    #        Automatic cleaning preserves sensor accuracy over time
    
    # DEVICE INFORMATION COMMANDS
    # ===========================
    
    CMD_PRODUCT_TYPE = [0xD0, 0x02]
    # Purpose: Read product type string identifier
    # Usage: Simple command, no arguments needed
    # Final command: [0xD0, 0x02]
    # Response: 12 bytes in groups of [char1, char2, CRC] = 4 groups
    # Notes: Returns "00080000" for SPS30, used for device identification
    #        Each character pair has its own CRC for data integrity
    
    CMD_SERIAL_NUMBER = [0xD0, 0x33]
    # Purpose: Read unique device serial number
    # Usage: Simple command, no arguments needed
    # Final command: [0xD0, 0x33]
    # Response: 48 bytes in groups of [char1, char2, CRC] = 16 groups
    # Notes: Unique ASCII serial like "1234567890ABCDEF"
    #        Used for device tracking and warranty identification
    
    CMD_FIRMWARE_VERSION = [0xD1, 0x00]
    # Purpose: Read firmware version for compatibility checking
    # Usage: Simple command, no arguments needed
    # Final command: [0xD1, 0x00]
    # Response: 3 bytes [major, minor, CRC]
    # Notes: Version like "2.2" = [0x02, 0x02, CRC]
    #        Important for determining available features and bug fixes
    
    # DIAGNOSTIC COMMANDS
    # ===================
    
    CMD_READ_STATUS_REGISTER = [0xD2, 0x06]
    # Purpose: Read device status flags for error detection
    # Usage: Simple command, no arguments needed
    # Final command: [0xD2, 0x06]
    # Response: 6 bytes [B1, B2, CRC, B3, B4, CRC] = 32-bit status word
    # Notes: Status bits indicate:
    #        - Bit 21: Fan speed warning (too high/low)
    #        - Bit 26: Laser current out of range
    #        - Bit 27: Fan failure (0 RPM)
    #        Critical for detecting hardware malfunctions
    
    CMD_CLEAR_STATUS_REGISTER = [0xD2, 0x10]
    # Purpose: Clear error flags in status register
    # Usage: Simple command, no arguments needed
    # Final command: [0xD2, 0x10]
    # Response: None (command only)
    # Notes: Resets error conditions, useful after fixing hardware issues
    #        Should be used as part of error recovery procedures
    
    CMD_RESET = [0xD3, 0x04]
    # Purpose: Perform soft reset of the sensor (like power cycle)
    # Usage: Simple command, no arguments needed
    # Final command: [0xD3, 0x04]
    # Response: None (command only)
    # Notes: Resets all settings to defaults, stops measurement
    #        Takes ~100ms to complete, equivalent to power-on reset
    #        Use as last resort for error recovery
    
    # Response lengths in bytes
    NBYTES_READ_DATA_READY_FLAG = 3
    NBYTES_MEASURED_VALUES_FLOAT = 60  # IEEE754 float format
    NBYTES_MEASURED_VALUES_INTEGER = 30  # unsigned 16 bit integer format
    NBYTES_AUTO_CLEANING_INTERVAL = 6
    NBYTES_PRODUCT_TYPE = 12
    NBYTES_SERIAL_NUMBER = 48
    NBYTES_FIRMWARE_VERSION = 3
    NBYTES_READ_STATUS_REGISTER = 6
    
    # Data structure sizes
    PACKET_SIZE = 3  # [data1, data2, checksum]
    SIZE_FLOAT = 6   # IEEE754 float with CRCs: [B1, B2, CRC, B3, B4, CRC]
    SIZE_INTEGER = 3 # unsigned 16 bit integer format
    
    def __init__(self, scl_pin=22, sda_pin=21, freq=100000, bus_number=0, debug=True,power_on_wait=True):
        """
        Initialize Enhanced SPS30 sensor with rigorous CRC validation
        
        Args:
            scl_pin (int): GPIO pin for I2C clock (default: 22)
            sda_pin (int): GPIO pin for I2C data (default: 21)
            freq (int): I2C frequency in Hz (default: 100000)
            bus_number (int): I2C bus number (default: 0)
            debug (bool): Enable detailed CRC error reporting (default: True)
        """
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.freq = freq
        self.bus_number = bus_number
        self.address = self.I2C_ADDRESS
        self.debug = debug
        self._is_measuring = False
        
        self._valid = {
            "mass_density": False,
            "particle_count": False,
            "particle_size": False
        }
        
        # CRC error statistics
        self._total_crc_errors = 0
        self._last_measurement_crc_errors = 0
        
        # Initialize I2C with error handling
        try:
            self.i2c = I2C(bus_number, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        except Exception as e:
            raise Exception(f"I2C initialization failed: {e}")
        
        # Verify sensor connection
        if not self.is_connected():
            raise Exception(f"SPS30 not found at address 0x{self.address:02X}")
        
        print(f"SPS30 connected on I2C{bus_number} (SDA: GPIO{sda_pin}, SCL: GPIO{scl_pin})")
        print(f"Debug mode: {'ENABLED' if debug else 'DISABLED'}")
        if power_on_wait:
            self._dbg("Initial 20s burn-in …")
            sleep(20)
        # Force garbage collection after initialization
        collect()

    def _dbg(self, *msg):
        """Print only when self.debug is True."""
        if self.debug:
            print("[SPS30]", *msg)

    def _bus_reset(self):
        """Free a stuck I²C bus by toggling SCL 9× and re‑init I²C."""
        self._dbg("BUS RESET")
        scl = Pin(self.scl_pin, Pin.OUT, value=1)
        sda = Pin(self.sda_pin, Pin.OUT, value=1)
        for _ in range(9):
            scl.off(); sleep(0.00001)
            scl.on();  sleep(0.00001)
        # restore open‑drain and re‑open I²C
        scl.init(Pin.OPEN_DRAIN, value=1)
        sda.init(Pin.OPEN_DRAIN, value=1)
        self.i2c = I2C(self.bus_number,
                    scl=Pin(self.scl_pin),
                    sda=Pin(self.sda_pin),
                    freq=self.freq)
    
    def is_connected(self):
        """Check if SPS30 is connected to I2C bus"""
        self._dbg("is_connected()")
        try:
            devices = self.i2c.scan()
            return self.address in devices
        except:
            return False
    
    def crc_calc(self, data):
        """Return Sensirion CRC‑8 (poly 0x31, init 0xFF) for a 2‑byte iterable."""
        if len(data) != 2:
            self._dbg("CRC warn: need 2 bytes, got", len(data))
            return 0xFF

        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc


    
    def _write_cmd(self, cmd, *, guard_ms=20):
        """
        I²C write with optional 20 ms guard AND full trace:
            * cmd bytes
            * CRCs (auto‑detected)
        """
        if self.debug:
            # highlight every third byte if it *looks* like a CRC
            printable = []
            for idx, b in enumerate(cmd):
                highlight = "*" if idx >= 2 and (idx - 2) % 3 == 2 else " "
                printable.append(f"{highlight}{b:02X}")
            self._dbg("->", " ".join(printable), f"(guard {guard_ms} ms)")
        try:
            self.i2c.writeto(self.address, bytes(cmd))
        except OSError as e:
            self._dbg("I2C write error", e)
            self._bus_reset()
            raise
        if guard_ms:
            sleep(guard_ms / 1000)

    
    def _read_response(self, length):
        """Read exactly `length` bytes and dump hex + ASCII when debug is on."""
        try:
            data = self.i2c.readfrom(self.address, length)
        except OSError as e:
            self._dbg("I2C read error", e)
            self._bus_reset()
            raise
        if self.debug:
            hex_view = " ".join(f"{b:02X}" for b in data)
            ascii_view = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
            self._dbg("←", hex_view, "|", ascii_view)
            if len(data) != length:
                self._dbg(f"* length mismatch {len(data)} (expected {length})")
        return data

    
    @staticmethod
    def ieee754_number_conversion(u32: int) -> float:
        """Convert big‑endian 32‑bit word → float (fast path uses struct)."""
        u32 &= 0xFFFFFFFF
        try:
            import struct
            return struct.unpack(">f", u32.to_bytes(4, "big"))[0]
        except (ImportError, AttributeError):
            # fallback bit‑twiddler
            s = u32 >> 31
            e = (u32 >> 23) & 0xFF
            f =  u32 & 0x7FFFFF
            if e == 0 and f == 0:    return -0.0 if s else 0.0
            if e == 0xFF:            return float("nan") if f else float("inf")*(-1)**s
            bias = 127
            exp, mant = (1-bias, f/(1<<23)) if e == 0 else (e-bias, 1+f/(1<<23))
            return (-1)**s * mant * (2**exp)



    
    def firmware_version(self):
        """Get firmware version with enhanced CRC validation"""
        self._dbg("firmware_version()")
        try:
            self._write_cmd(self.CMD_FIRMWARE_VERSION)
            data = self._read_response(self.NBYTES_FIRMWARE_VERSION)
            
            # Enhanced CRC validation with detailed error reporting
            calculated_crc = self.crc_calc(data[:2])
            received_crc = data[2]
            
            if calculated_crc != received_crc:
                self._total_crc_errors += 1
                print(f"❌ FIRMWARE VERSION CRC MISMATCH!")
                print(f"   Data: [{data[0]:02X}, {data[1]:02X}]")
                print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                print(f"   Received CRC: 0x{received_crc:02X}")
                return "CRC mismatched"
            
            return f"{data[0]}.{data[1]}"
            
        except Exception as e:
            print(f"❌ Failed to get firmware version: {e}")
            return "Unknown"
    
    def product_type(self):
        """Get product type with enhanced CRC validation"""
        self._dbg("product_type()")
        try:
            self._write_cmd(self.CMD_PRODUCT_TYPE)
            data = self._read_response(self.NBYTES_PRODUCT_TYPE)
            result = ""
            
            # Validate each 3-byte packet
            for i in range(0, self.NBYTES_PRODUCT_TYPE, 3):
                data_bytes = data[i:i+2]
                calculated_crc = self.crc_calc(data_bytes)
                received_crc = data[i+2]
                
                if calculated_crc != received_crc:
                    self._total_crc_errors += 1
                    print(f"❌ PRODUCT TYPE CRC MISMATCH at position {i}!")
                    print(f"   Data: [{data_bytes[0]:02X}, {data_bytes[1]:02X}]")
                    print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                    print(f"   Received CRC: 0x{received_crc:02X}")
                    return "CRC mismatched"
                
                # Build result string (filter null bytes)
                for byte in data_bytes:
                    if byte != 0:
                        result += chr(byte)
            
            return result.strip()
            
        except Exception as e:
            print(f"❌ Failed to get product type: {e}")
            return "SPS30"
    
    def serial_number(self):
        """Get serial number with enhanced CRC validation"""
        self._dbg("serial_number()")
        try:
            self._write_cmd(self.CMD_SERIAL_NUMBER)
            data = self._read_response(self.NBYTES_SERIAL_NUMBER)
            result = ""
            
            # Validate each 3-byte packet
            for i in range(0, self.NBYTES_SERIAL_NUMBER, self.PACKET_SIZE):
                data_bytes = data[i:i+2]
                calculated_crc = self.crc_calc(data_bytes)
                received_crc = data[i+2]
                
                if calculated_crc != received_crc:
                    self._total_crc_errors += 1
                    print(f"❌ SERIAL NUMBER CRC MISMATCH at position {i}!")
                    print(f"   Data: [{data_bytes[0]:02X}, {data_bytes[1]:02X}]")
                    print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                    print(f"   Received CRC: 0x{received_crc:02X}")
                    return "CRC mismatched"
                
                # Build result string (filter null bytes)
                for byte in data_bytes:
                    if byte != 0:
                        result += chr(byte)
            
            return result.strip()
            
        except Exception as e:
            print(f"❌ Failed to get serial number: {e}")
            return "Unknown"
    
    def read_status_register(self):
        """Return decoded device‑status flags + raw 32‑bit value.

        Bits (datasheet §4.4):
            21  SPEED   – fan speed out of range
            5  LASER   – laser current out of range
            4  FAN     – fan mechanically blocked
        """
        self._dbg("read_status_register()")
        try:
            self._write_cmd(self.CMD_READ_STATUS_REGISTER)
            data = self._read_response(self.NBYTES_READ_STATUS_REGISTER)

            # CRC check & word assembly (big‑endian)
            word = 0
            for i in range(0, 6, 3):
                if self.crc_calc(data[i:i+2]) != data[i+2]:
                    self._total_crc_errors += 1
                    return {"error": "CRC mismatched"}
                word = (word << 16) | (data[i] << 8) | data[i+1]

            speed_warn = bool(word & (1 << 21))   # bit 21
            laser_err  = bool(word & (1 << 5))    # bit 5
            fan_err    = bool(word & (1 << 4))    # bit 4

            return {
                "speed_status": "warning" if speed_warn else "ok",
                "laser_status": "out of range" if laser_err else "ok",
                "fan_status": "0 rpm" if fan_err else "ok",
                "raw_status": word
            }
        except Exception as e:
            return {"error": str(e)}


    
    def clear_status_register(self):
        """Clear status register"""
        self._dbg("clear_status_register()")
        try:
            self._write_cmd(self.CMD_CLEAR_STATUS_REGISTER)
            return True
        except Exception as e:
            print(f"❌ Failed to clear status register: {e}")
            return False
    
    def read_data_ready_flag(self):
        """Check if data is ready with enhanced CRC validation"""
        self._dbg("read_data_ready_flag()")
        try:
            self._write_cmd(self.CMD_READ_DATA_READY_FLAG)
            data = self._read_response(self.NBYTES_READ_DATA_READY_FLAG)
            
            # Enhanced CRC validation with detailed error reporting
            calculated_crc = self.crc_calc(data[:2])
            received_crc = data[2]
            
            if calculated_crc != received_crc:
                self._total_crc_errors += 1
                print(f"❌ READ DATA READY FLAG CRC MISMATCH!")
                print(f"   Data: [{data[0]:02X}, {data[1]:02X}]")
                print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                print(f"   Received CRC: 0x{received_crc:02X}")
                return False
            
            return True if data[1] == 1 else False
            
        except Exception as e:
            if self.debug:
                print(f"❌ Data ready check failed: {e}")
            return False
    
    def read_auto_cleaning_interval(self):
        """
        Return the auto‑cleaning interval in **seconds** (0 = disabled).
        Returns **None** when a CRC mismatch or I²C length error is detected.
        """
        self._dbg("read_auto_cleaning_interval()")
        # 1 — command (no delay because we read immediately afterwards)
        self._write_cmd(self.CMD_AUTO_CLEANING_INTERVAL, guard_ms=0)

        # 2 — response
        data = self._read_response(self.NBYTES_AUTO_CLEANING_INTERVAL)

        if len(data) != self.NBYTES_AUTO_CLEANING_INTERVAL:
            if self.debug:
                print("❌ AUTO‑CLEAN interval: length mismatch",
                    len(data), "bytes (expected 6)")
            return None

        # 3 — validate two CRC‑protected packets
        lo_word = hi_word = None
        for offset in (0, 3):                       # packet 0 and packet 1
            chunk       = data[offset:offset+2]     # 2 data bytes
            received_crc = data[offset+2]
            calc_crc     = self.crc_calc(chunk)

            if calc_crc != received_crc:
                self._total_crc_errors += 1
                if self.debug:
                    print(f"❌ AUTO‑CLEAN CRC error @ offset {offset}:",
                        f"calc=0x{calc_crc:02X} recv=0x{received_crc:02X}")
                return None

            if offset == 0:
                lo_word = (chunk[0] << 8) | chunk[1]    # low 16 bits
            else:
                hi_word = (chunk[0] << 8) | chunk[1]    # high 16 bits

        # 4 — assemble 32‑bit little‑endian value
        seconds = (hi_word << 16) | lo_word
        return seconds

    
    def write_auto_cleaning_interval_days(self, days):
        """Set auto‑clean interval (0 … 4 294 967 295 s) → sensor expects **little‑endian**."""
        self._dbg(f"write_auto_cleaning_interval_days(days={days})")
        try:
            seconds = days * 86400
            lo = seconds & 0xFFFF           # lower 16 bits  (sent first)
            hi = seconds >> 16              # upper 16 bits  (sent second)

            cmd = self.CMD_AUTO_CLEANING_INTERVAL.copy()
            # low half‑word + CRC
            cmd.extend([(lo >> 8) & 0xFF, lo & 0xFF])
            cmd.append(self.crc_calc(cmd[2:4]))
            # high half‑word + CRC
            cmd.extend([(hi >> 8) & 0xFF, hi & 0xFF])
            cmd.append(self.crc_calc(cmd[5:7]))

            if self.debug:
                print("🔍 auto‑clean =", days, "days  →", seconds, "s")
                print("   cmd =", [hex(b) for b in cmd])

            self._write_cmd(cmd,guard_ms=0)
            # sleep(0.05)
            return self.read_auto_cleaning_interval()      # verify
        except Exception as e:
            print(f"Failed to set auto‑clean interval: {e}")
            return False

    
    def sleep(self):
        self._dbg("sleep()")
        """Put sensor to sleep"""
        try:
            self._write_cmd(self.CMD_SLEEP)
            return True
        except Exception as e:
            print(f"❌ Sleep failed: {e}")
            return False
    
    def wakeup(self):
        self._dbg("wakeup()")
        """Wake up sensor"""
        try:
            self._write_cmd(self.CMD_WAKEUP,guard_ms=0)
            # sleep(0.05)  # Wait for wake-up***
            return True
        except Exception as e:
            print(f"❌ Wakeup failed: {e}")
            return False
    
    def start_fan_cleaning(self):
        """Start manual fan cleaning"""
        self._dbg("start_fan_cleaning()")
        try:
            self._write_cmd(self.CMD_START_FAN_CLEANING)
            return True
        except Exception as e:
            print(f"❌ Fan cleaning failed: {e}")
            return False
    
    def reset(self):
        """Reset sensor"""
        try:
            self._write_cmd(self.CMD_RESET,guard_ms=0)
            # sleep(0.1)
            self._is_measuring = False
            # Reset error counters
            self._total_crc_errors = 0
            self._last_measurement_crc_errors = 0
            return True
        except Exception as e:
            print(f"❌ Reset failed: {e}")
            return False
    
    def start_measurement(self):
        """Start measurement with enhanced validation"""
        self._dbg("start_measurement()")
        try:
            # Build start command with IEEE754 float format
            data_format = {
                "IEEE754_float": 0x03,
                "unsigned_16_bit_integer": 0x05
            }
            
            cmd = self.CMD_START_MEASUREMENT.copy()
            cmd.extend([data_format["IEEE754_float"], 0x00])
            cmd.append(self.crc_calc(cmd[2:4]))
            
            if self.debug:
                expected_crc = self.crc_calc([0x03, 0x00])
                print(f"🔍 Start measurement command: {[hex(b) for b in cmd]}")
                print(f"   Data format: IEEE754 float (0x03, 0x00)")
                print(f"   Expected CRC: 0x{expected_crc:02X}")
            
            self._write_cmd(cmd,guard_ms=0)
            # sleep(0.05)
            self._is_measuring = True
            
            # Reset validity flags
            self._valid = {
                "mass_density": False,
                "particle_count": False,
                "particle_size": False
            }
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to start measurement: {e}")
            return False
    
    def stop_measurement(self):
        """Stop measurement"""
        self._dbg("stop_measurement()")
        try:
            self._write_cmd(self.CMD_STOP_MEASUREMENT,guard_ms=0)
            # sleep(0.05)
            self._is_measuring = False
            return True
        except Exception as e:
            print(f"❌ Failed to stop measurement: {e}")
            return False
    
    def _mass_density_measurement(self, data):
        """Parse mass density with rigorous CRC validation"""
        self._dbg("_mass_density_measurement() len", len(data))
        category = ["pm1.0", "pm2.5", "pm4.0", "pm10"]
        
        density = {
            "pm1.0": 0.0,
            "pm2.5": 0.0,
            "pm4.0": 0.0,
            "pm10": 0.0
        }
        
        
        for block, pm in enumerate(category):
            pm_data = []
            
            # Process each 6-byte float (2 packets of 3 bytes each)
            for i in range(0, self.SIZE_FLOAT, self.PACKET_SIZE):
                offset = (block * self.SIZE_FLOAT) + i
                data_bytes = data[offset:offset+2]
                calculated_crc = self.crc_calc(data_bytes)
                received_crc = data[offset+2]
                
                if calculated_crc != received_crc:
                    self._total_crc_errors += 1
                    print(f"❌ MASS DENSITY MEASUREMENT CRC MISMATCH!")
                    print(f"   PM Type: {pm}")
                    print(f"   Block: {block}, Packet: {i//3}")
                    print(f"   Offset: {offset}")
                    print(f"   Data: [{data_bytes[0]:02X}, {data_bytes[1]:02X}]")
                    print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                    print(f"   Received CRC: 0x{received_crc:02X}")
                    
                    #any CRC error invalidates entire section
                    self._valid["mass_density"] = False
                    return {}
                
                pm_data.extend(data_bytes)
            
            # Convert to float using custom IEEE754 conversion
            raw_value = (pm_data[0] << 24) | (pm_data[1] << 16) | (pm_data[2] << 8) | pm_data[3]
            density[pm] = self.ieee754_number_conversion(raw_value)
        
        # If we get here, all CRCs were valid
        self._valid["mass_density"] = True
        return density
    
    def _particle_count_measurement(self, data):
        """Parse particle count with rigorous CRC validation"""
        self._dbg("_particle_count_measurement() len", len(data))
        category = ["pm0.5", "pm1.0", "pm2.5", "pm4.0", "pm10"]
        
        count = {
            "pm0.5": 0.0,
            "pm1.0": 0.0,
            "pm2.5": 0.0,
            "pm4.0": 0.0,
            "pm10": 0.0
        }
        
        
        for block, pm in enumerate(category):
            pm_data = []
            
            # Process each 6-byte float (2 packets of 3 bytes each)
            for i in range(0, self.SIZE_FLOAT, self.PACKET_SIZE):
                offset = (block * self.SIZE_FLOAT) + i
                data_bytes = data[offset:offset+2]
                calculated_crc = self.crc_calc(data_bytes)
                received_crc = data[offset+2]
                
                if calculated_crc != received_crc:
                    self._total_crc_errors += 1
                    print(f"❌ PARTICLE COUNT MEASUREMENT CRC MISMATCH!")
                    print(f"   PM Type: {pm}")
                    print(f"   Block: {block}, Packet: {i//3}")
                    print(f"   Offset: {offset}")
                    print(f"   Data: [{data_bytes[0]:02X}, {data_bytes[1]:02X}]")
                    print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                    print(f"   Received CRC: 0x{received_crc:02X}")
                    
                    #any CRC error invalidates entire section
                    self._valid["particle_count"] = False
                    return {}
                
                pm_data.extend(data_bytes)
            
            # Convert to float using custom IEEE754 conversion
            raw_value = (pm_data[0] << 24) | (pm_data[1] << 16) | (pm_data[2] << 8) | pm_data[3]
            count[pm] = self.ieee754_number_conversion(raw_value)
        
        # If we get here, all CRCs were valid
        self._valid["particle_count"] = True
        return count
    
    def _particle_size_measurement(self, data):
        """Parse particle size with rigorous CRC validation"""
        self._dbg("_particle_size_measurement() len", len(data))
        size_data = []
        
        # Process each 3-byte packet in the 6-byte float
        for i in range(0, self.SIZE_FLOAT, self.PACKET_SIZE):
            data_bytes = data[i:i+2]
            calculated_crc = self.crc_calc(data_bytes)
            received_crc = data[i+2]
            
            if calculated_crc != received_crc:
                self._total_crc_errors += 1
                print(f"❌ PARTICLE SIZE MEASUREMENT CRC MISMATCH!")
                print(f"   Packet: {i//3}")
                print(f"   Offset: {i}")
                print(f"   Data: [{data_bytes[0]:02X}, {data_bytes[1]:02X}]")
                print(f"   Calculated CRC: 0x{calculated_crc:02X}")
                print(f"   Received CRC: 0x{received_crc:02X}")
                
                # any CRC error invalidates measurement
                self._valid["particle_size"] = False
                return 0.0
            
            size_data.extend(data_bytes)
        
        # If we get here, all CRCs were valid
        self._valid["particle_size"] = True
        
        # Convert to float using custom IEEE754 conversion
        raw_value = (size_data[0] << 24) | (size_data[1] << 16) | (size_data[2] << 8) | size_data[3]
        return self.ieee754_number_conversion(raw_value)
    
    def get_measurement(self, rolling_n=1):
        """
        Acquire a measurement frame. 

        Args:
            rolling_n (int): 1 = return the most‑recent clean frame.
                            N>1 = return the arithmetic mean of the last N
                            *valid* frames (frames with CRC errors are discarded).

        Returns:
            dict | {}  – averaged frame or empty dict until enough clean frames
                        are accumulated.
        """
        self._dbg(f"get_measurement(rolling_n={rolling_n})")
        if rolling_n < 1:
            rolling_n = 1
        # ────────────────────────── pre‑checks ──────────────────────────────
        if not self._is_measuring:
            print("⚠️  Measurement not started – call start_measurement() first")
            return {}

        # Rolling window (MicroPython-safe): use a LIST, not deque
        current_hist_len = getattr(self, "_hist_maxlen", None)
        if rolling_n > 1 and (not hasattr(self, "_hist_frames") or current_hist_len != rolling_n):
            self._hist_frames = []          # list of recent frames
            self._hist_maxlen = rolling_n

        if rolling_n == 1 and hasattr(self, "_hist_frames"):
            # Drop rolling history when switching back to single-frame mode
            try:
                self._hist_frames = []
            except Exception:
                pass
            self._hist_maxlen = None
        # ────────────────────────── read sensor ─────────────────────────────
        self._valid = {k: False for k in self._valid}
        try:
            if not self.read_data_ready_flag():
                return {}

            self._write_cmd(self.CMD_READ_MEASURED_VALUES)
            raw = self._read_response(self.NBYTES_MEASURED_VALUES_FLOAT)

            start_err = self._total_crc_errors
            mass = self._mass_density_measurement(raw[:24])
            cnt  = self._particle_count_measurement(raw[24:54])
            size = self._particle_size_measurement(raw[54:])
            self._last_measurement_crc_errors = self._total_crc_errors - start_err

            if not all(self._valid.values()):
                self._dbg("frame rejected (CRC error)")
                return {}

            frame = {
                "mass_density":   mass,
                "particle_count": cnt,
                "particle_size":  size
            }

            # ───────────────────── rolling‑average logic ────────────────────────
            if rolling_n > 1:
                self._hist_frames.append(frame)
                if len(self._hist_frames) > rolling_n:
                    # remove oldest (front)
                    self._hist_frames.pop(0)

                if self.debug:
                    idx = len(self._hist_frames)
                    self._dbg(f"[rolling] stored frame {idx}/{rolling_n}",
                              "PM2.5 =", frame["mass_density"]["pm2.5"],
                              "PC2.5 =", frame["particle_count"]["pm2.5"],
                              "SIZE =", frame["particle_size"])

                if len(self._hist_frames) < rolling_n:
                    if self.debug:
                         self._dbg(f"[rolling] waiting for {rolling_n} clean frames have {len(self._hist_frames)}")
                    return {}  # not enough samples yet

                # Build averages without indexing/iterating a deque
                # (lists are fully subscriptable/iterable on MicroPython)
                n = len(self._hist_frames)
                first = self._hist_frames[0]

                # Average mass_density and particle_count blocks
                avg_mass = {}
                for k in first["mass_density"]:
                    s = 0.0
                    for f in self._hist_frames:
                        s += f["mass_density"][k]
                    avg_mass[k] = s / n

                avg_count = {}
                for k in first["particle_count"]:
                    s = 0.0
                    for f in self._hist_frames:
                        s += f["particle_count"][k]
                    avg_count[k] = s / n

                # Average particle_size (single float)
                s_size = 0.0
                for f in self._hist_frames:
                    s_size += f["particle_size"]
                avg_size = s_size / n

                frame = {
                    "mass_density":   avg_mass,
                    "particle_count": avg_count,
                    "particle_size":  avg_size
                }

                if self.debug:
                    self._dbg("[rolling] AVERAGED frame ready",
                              "PM2.5 =", frame["mass_density"]["pm2.5"],
                              "PC2.5 =", frame["particle_count"]["pm2.5"],
                              "SIZE =", frame["particle_size"])

            # ───────────────────────── payload build ────────────────────────────
            sensor_data = frame.copy()                 # shallow copy is fine
            sensor_data["mass_density_unit"]   = "ug/m3"
            sensor_data["particle_count_unit"] = "#/cm3"
            sensor_data["particle_size_unit"]  = "um"

            result = {
                "sensor_data": sensor_data,
                "measurement_info": {
                    "timestamp": ticks_ms(),
                    "crc_errors_this_measurement": self._last_measurement_crc_errors,
                    "total_crc_errors": self._total_crc_errors,
                    "all_sections_valid": all(self._valid.values()),
                    "validity": self._valid.copy(),
                    "rolling_n": rolling_n
                }
            }

            if self.debug and self._last_measurement_crc_errors == 0:
                self._dbg("MEASUREMENT SUCCESSFUL – CRC OK")

            collect()              # free memory after heavy work
            return result

            
        except Exception as e:
            print(f"Measurement failed: {e}")
            return {}
    
    def get_device_info(self):
        """Get comprehensive device information with CRC statistics"""
        self._dbg("get_device_info()")
        _sec  = self.read_auto_cleaning_interval()
        _days = 0 if _sec is None else _sec // 86_400
        return {
            "device_info": {
                "product_type": self.product_type(),
                "serial_number": self.serial_number(),
                "firmware_version": self.firmware_version(),
                "i2c_address": f"0x{self.address:02X}",
                "connection": {
                    "scl_pin": self.scl_pin,
                    "sda_pin": self.sda_pin,
                    "frequency": self.freq,
                    "bus_number": self.bus_number
                }
            },
            "settings": {
                "auto_cleaning_interval_seconds": _sec,
                "auto_cleaning_interval_days":    _days,
                "is_measuring": self._is_measuring,
                "debug_mode": self.debug
            },
            "crc_statistics": {
                "total_crc_errors": self._total_crc_errors,
                "last_measurement_crc_errors": self._last_measurement_crc_errors,
                "current_validity": self._valid.copy()
            },
            "status": self.read_status_register()
        }
    
    def take_measurement_series(self, count=5, interval=2, stabilization_time=10):
        """
        Take multiple measurements with enhanced CRC monitoring
        
        Args:
            count (int): Number of measurements to take
            interval (float): Seconds between measurements
            stabilization_time (float): Initial stabilization time
            
        Returns:
            list: List of valid measurement dictionaries
        """
        self._dbg(f"take_measurement_series(count={count}, interval={interval}, stab={stabilization_time})")
        results = []
        crc_error_count = 0
        invalid_measurements = 0
        
        try:
            # Start measurement
            if not self.start_measurement():
                return results
            
            print(f"⏳ Waiting {stabilization_time} seconds for stabilization...")
            sleep(stabilization_time)
            
            print(f"📊 Taking {count} measurements with enhanced CRC validation...")
            
            for i in range(count):
                # Wait for data ready
                timeout = 0
                while not self.read_data_ready_flag() and timeout < 50:  # 5 second timeout
                    sleep(0.1)
                    timeout += 1
                
                if timeout >= 50:
                    print(f"⚠️ Timeout waiting for measurement {i+1}")
                    continue
                
                # Get measurement
                measurement_start_errors = self._total_crc_errors
                measurement = self.get_measurement()
                measurement_errors = self._total_crc_errors - measurement_start_errors
                
                if measurement:  # Valid measurement
                    results.append(measurement)
                    
                    # Display key values
                    mass = measurement["sensor_data"]["mass_density"]
                    pm25 = mass["pm2.5"]
                    pm10 = mass["pm10"]
                    particles = measurement["sensor_data"]["particle_count"]["pm2.5"]
                    
                    print(f"✅ Reading {i+1}: PM2.5={pm25} µg/m³, PM10={pm10} µg/m³, Particles={particles} /cm³ (CRC: OK)")
                else:  # Invalid measurement
                    invalid_measurements += 1
                    crc_error_count += measurement_errors
                    print(f"❌ Reading {i+1}: INVALID - CRC errors detected ({measurement_errors} errors)")
                
                # Wait before next measurement (except last one)
                if i < count - 1:
                    sleep(interval)
                    
                # Force garbage collection between measurements
                collect()
            
        except Exception as e:
            print(f"❌ Measurement series failed: {e}")
        
        finally:
            self.stop_measurement()
            
            # Print summary
            print(f"\n📊 MEASUREMENT SERIES SUMMARY:")
            print(f"   Valid measurements: {len(results)}/{count}")
            print(f"   Invalid measurements: {invalid_measurements}")
            print(f"   Total CRC errors in series: {crc_error_count}")
            print(f"   Success rate: {(len(results)/count)*100:.1f}%")
            
            collect()  # Final cleanup
        
        return results

# Enhanced test function with rigorous CRC monitoring
def enhanced_crc_test(scl_pin=22, sda_pin=27):
    """Enhanced test with rigorous CRC validation monitoring"""
    print("🚀 ENHANCED SPS30 CRC VALIDATION TEST")
    print("=" * 70)
    
    try:
        # Check initial memory
        print(f"📊 Free memory at start: {mem_free()} bytes")
        
        sensor = SPS30(scl_pin=scl_pin, sda_pin=sda_pin, debug=True)
        
        # Show device info with CRC validation
        print("\n📋 DEVICE INFORMATION (with CRC validation):")
        info = sensor.get_device_info()
        print(f"Product: {info['device_info']['product_type']}")
        print(f"Serial: {info['device_info']['serial_number']}")
        print(f"Firmware: {info['device_info']['firmware_version']}")
        print(f"Auto-clean: {info['settings']['auto_cleaning_interval_days']} days")
        print(f"CRC Errors so far: {info['crc_statistics']['total_crc_errors']}")
        
        print(f"📊 Free memory after init: {mem_free()} bytes")
        
        # Test device control functions
        print("\n🔧 TESTING DEVICE CONTROLS:")
        print(f"Clear status register: {sensor.clear_status_register()}")
        
        # Take measurement series with enhanced monitoring
        print("\n📊 MEASUREMENT SERIES WITH CRC MONITORING:")
        measurements = sensor.take_measurement_series(count=5, interval=2, stabilization_time=8)
        
        print(f"\n✅ Collected {len(measurements)} valid measurements")
        print(f"📊 Free memory after measurements: {mem_free()} bytes")
        
        # Show final CRC statistics
        final_info = sensor.get_device_info()
        final_crc_stats = final_info['crc_statistics']
        
        print(f"\n📈 FINAL CRC STATISTICS:")
        print(f"   Total CRC errors: {final_crc_stats['total_crc_errors']}")
        print(f"   Last measurement errors: {final_crc_stats['last_measurement_crc_errors']}")
        print(f"   Current validity: {final_crc_stats['current_validity']}")
        
        # Show summary if we have measurements
        if measurements:
            pm25_values = [m["sensor_data"]["mass_density"]["pm2.5"] for m in measurements]
            avg_pm25 = sum(pm25_values) / len(pm25_values)
            print(f"\n📈 Average PM2.5: {avg_pm25:.2f} µg/m³")
            
            # Air quality assessment
            if avg_pm25 <= 12:
                quality = "GOOD 🟢"
            elif avg_pm25 <= 35:
                quality = "MODERATE 🟡"
            else:
                quality = "UNHEALTHY 🔴"
            
            print(f"🌬️ Air Quality: {quality}")
        
        print("\n🎉 Enhanced CRC validation test completed!")
        print(f"📊 Final free memory: {mem_free()} bytes")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
    finally:
        collect()

# Example usage
if __name__ == "__main__":
    enhanced_crc_test()


