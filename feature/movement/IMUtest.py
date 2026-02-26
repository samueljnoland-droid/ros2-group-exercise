# This code is to read the IMU values
# The IMU im using is a 10 axis sensor with on board low power ICM20948
# I will be using the gyroscope values
#!/usr/bin/env python3
"""
ICM-20948 Gyroscope reader (Raspberry Pi 4, I2C, Python)
Reads gyro XYZ and prints values in deg/s.

Wiring (typical):
- VCC to 3.3V (check your board!)
- GND to GND
- SDA to SDA (GPIO2, pin 3)
- SCL to SCL (GPIO3, pin 5)
"""

import time
from smbus2 import SMBus

# ----------------------------
# ICM-20948 Register Addresses
# ----------------------------
REG_BANK_SEL   = 0x7F

# Bank 0
WHO_AM_I       = 0x00
PWR_MGMT_1     = 0x06
PWR_MGMT_2     = 0x07
GYRO_XOUT_H    = 0x33  # gyro data starts here (Bank 0): 0x33..0x38

# Bank 2
GYRO_SMPLRT_DIV = 0x00
GYRO_CONFIG_1    = 0x01  # FS_SEL bits, DLPF config, etc.

# WHO_AM_I expected for ICM-20948
WHO_AM_I_VAL = 0xEA


class ICM20948:
    def __init__(self, bus_num=1, address=None):
        self.bus = SMBus(bus_num)

        # If address not provided, try common ones
        if address is None:
            address = self._detect_address()
        self.addr = address

        # Wake and configure
        self._init_device()

    def close(self):
        self.bus.close()

    def _detect_address(self):
        for a in (0x68, 0x69):
            try:
                self._select_bank(a, 0)
                val = self.bus.read_byte_data(a, WHO_AM_I)
                if val == WHO_AM_I_VAL:
                    return a
            except OSError:
                pass
        raise RuntimeError("ICM-20948 not found at 0x68 or 0x69. Check wiring/I2C enablement.")

    def _select_bank(self, addr, bank: int):
        # bank is 0..3; placed in bits [5:4] of REG_BANK_SEL
        self.bus.write_byte_data(addr, REG_BANK_SEL, (bank & 0x03) << 4)

    def _write(self, bank, reg, val):
        self._select_bank(self.addr, bank)
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, bank, reg, length=1):
        self._select_bank(self.addr, bank)
        if length == 1:
            return self.bus.read_byte_data(self.addr, reg)
        data = self.bus.read_i2c_block_data(self.addr, reg, length)
        return data

    @staticmethod
    def _twos_complement_16(high, low):
        value = (high << 8) | low
        if value & 0x8000:
            value -= 0x10000
        return value

    def _init_device(self):
        # Check WHO_AM_I
        self._select_bank(self.addr, 0)
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if who != WHO_AM_I_VAL:
            raise RuntimeError(f"Unexpected WHO_AM_I: 0x{who:02X} (expected 0x{WHO_AM_I_VAL:02X})")

        # Wake up: set clock source (auto best available), clear sleep
        # PWR_MGMT_1:
        # bit 7 = DEVICE_RESET, bit 6 = SLEEP
        # set to 0x01 selects auto clock (often recommended)
        self._write(0, PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable gyro (and accel if you want) in PWR_MGMT_2
        # In ICM20948, bits are "disable" bits:
        # 0 = enabled, 1 = disabled
        # We'll enable gyro XYZ and disable accel XYZ (optional).
        # 0b0011_1000 disables accel XYZ, leaves gyro enabled.
        self._write(0, PWR_MGMT_2, 0x38)
        time.sleep(0.05)

        # Configure gyro in Bank 2:
        # - Sample rate divider
        # - Full scale range (FS_SEL) + DLPF (optional)
        # GYRO_SMPLRT_DIV: gyro ODR = 1100 Hz / (1 + div) when DLPF enabled settings vary
        self._write(2, GYRO_SMPLRT_DIV, 9)  # ~110 Hz-ish depending on config

        # GYRO_CONFIG_1:
        # FS_SEL bits [2:1]
        # 00: ±250 dps
        # 01: ±500 dps
        # 10: ±1000 dps
        # 11: ±2000 dps
        #
        # We'll set ±250 dps, and leave DLPF settings default.
        # Value example: 0x00 => FS_SEL=0, default filters
        self._write(2, GYRO_CONFIG_1, 0x00)

        # Store scale for conversion to deg/s
        self.gyro_scale_dps_per_lsb = 250.0 / 32768.0  # for ±250 dps

    def set_gyro_range(self, dps):
        """
        Set gyro full scale range: 250, 500, 1000, 2000 (deg/s)
        """
        fs_map = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}
        if dps not in fs_map:
            raise ValueError("dps must be one of: 250, 500, 1000, 2000")

        fs_sel = fs_map[dps]
        # Read current config, update FS bits [2:1]
        current = self._read(2, GYRO_CONFIG_1, 1)
        new_val = (current & ~0x06) | (fs_sel << 1)
        self._write(2, GYRO_CONFIG_1, new_val)

        self.gyro_scale_dps_per_lsb = float(dps) / 32768.0

    def read_gyro_raw(self):
        """
        Returns raw gyro counts (gx, gy, gz) as signed 16-bit ints.
        """
        data = self._read(0, GYRO_XOUT_H, 6)
        gx = self._twos_complement_16(data[0], data[1])
        gy = self._twos_complement_16(data[2], data[3])
        gz = self._twos_complement_16(data[4], data[5])
        return gx, gy, gz

    def read_gyro_dps(self):
        """
        Returns gyro in deg/s (gx, gy, gz) as floats.
        """
        gx, gy, gz = self.read_gyro_raw()
        s = self.gyro_scale_dps_per_lsb
        return gx * s, gy * s, gz * s


def main():
    imu = ICM20948(bus_num=1)  # auto-detect 0x68/0x69
    # If you KNOW the address, you can do: ICM20948(bus_num=1, address=0x68)

    # Optional: set range to ±500 dps, ±1000 dps, ±2000 dps
    # imu.set_gyro_range(500)

    print(f"ICM-20948 found at I2C address 0x{imu.addr:02X}")
    print("Reading gyro... Press Ctrl+C to stop.\n")

    try:
        while True:
            gx, gy, gz = imu.read_gyro_dps()
            print(f"Gyro (deg/s): X={gx:8.3f}  Y={gy:8.3f}  Z={gz:8.3f}")
            time.sleep(0.05)  # 20 Hz print
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()


if __name__ == "__main__":
    main()
