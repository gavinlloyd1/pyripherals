from ..core import Register
from ..utils import int_to_list
from .I2CController import I2CController
import time


class TMF8828(I2CController):
    """Class for the Time-of-flight I2C device

    Subclass of the I2CController class. Attributes and methods below are
    differences in this class from I2CController only.

    Attributes
    ----------
    ADDRESS : int
        7-bit device address with R/W bit space 
    registers : dict
        Name-Register pairs for the internal registers of the chip.
    """

    ADDRESS = 0b0100_0001 << 1
    registers = Register.get_chip_registers('TMF8828')
    apps = {'measure': 0x03, 'bootloader': 0x80}
    MULTI_BYTE_ORDER = 'LSB_1st'

    def write(self, data, register_name):
        """Write data to any register on the chip.
            first reads to enable writing of (just) bit-fields within the register
        """

        dev_addr = self.ADDRESS
        register = self.registers[register_name]
        data <<= register.bit_index_low

        # Mask only the bits for the specified register.
        mask = 0
        for bit in range(register.bit_index_high, register.bit_index_low - 1, -1):
            mask |= 0x1 << bit

        # Compare with current data to mask unchanged values
        read_out = self.read(register_name)
        if read_out == None:
            print('Read for masking FAILED')
            return False
        new_data = (data & mask) | (read_out & ~mask)

        # Turn data into a list of bytes for i2c_write method
        list_data = int_to_list(new_data)
        number_of_bytes = len(list_data)
        list_data_str = ''.join(" ".join(f'0x{x:02x}' for x in list_data))
        print(
            f'i2c write long: 0x{dev_addr:02x}, reg addr 0x{register.address:02x}, data {list_data_str}')
        self.i2c_write_long(
            dev_addr, [register.address], number_of_bytes, list_data)

    def read(self, register_name, number_of_bytes=None):
        """Return data from any register on the chip."""

        dev_addr = self.ADDRESS
        register = self.registers[register_name]
        # This is the total number of bytes to read
        byte_number = (register.bit_index_high // 8) + 1
        # e.g. 16-bit register = 2 bytes, i2c_read_long starts at the MSB so we read 2 bytes to get Byte 0
        if number_of_bytes is None:
            number_of_bytes = byte_number
        # print(f'Read slave address: 0x{dev_addr:02x}, reg addr 0x{register.address:02x}')
        read_back_list = self.i2c_read_long(
            dev_addr, [register.address], number_of_bytes)

        # Turn the list into an integer
        read_back_data = 0
        if read_back_list == None:
            return None

        if self.MULTI_BYTE_ORDER == 'MSB_1st':
            for byte in read_back_list:
                # print('Readback byte of 0x{:02X}'.format(byte))
                read_back_data <<= 8
                read_back_data |= byte
        elif self.MULTI_BYTE_ORDER == 'LSB_1st':
            for byte in read_back_list[::-1]:  # flip list order
                # print('Readback byte of 0x{:02X}'.format(byte))
                read_back_data <<= 8
                read_back_data |= byte

        # Get only the bits for the specified register from what was read back.
        desired_bits = 0
        for bit in range(register.bit_index_high, register.bit_index_low - 1, -1):
            desired_bits += 0x1 << bit
        desired_data = (read_back_data
                        & desired_bits) >> register.bit_index_low

        return desired_data

    def get_id(self):
        """Return the device ID and rev ID as one integer."""

        device_id = self.read('ID')  # may need to change the referred registers
        version_id = self.read('REVID')
        return (device_id << self.registers['ID'].bit_index_low) | (version_id << self.registers['ID'].bit_index_low)

    def cpu_ready(self):  # should carry over fine
        return self.read('CPU_READY', 1)

    def read_app(self):  # should work fine with changed appid values

        appid = self.read('APPID')
        appname = list(self.apps.keys())[list(self.apps.values()).index(appid)]
        print(f'In app {appname}')
        return appid

    def rom_fw_version(self):  # now diff registers, likely minor, patch, build_type
        patch = self.read('PATCH', number_of_bytes=1)
        build_type = self.read('BUILD_TYPE', number_of_bytes=1)

        print(f'patch: {patch}, build_type: {build_type}')

        return patch, build_type

    def ram_write_status(self):  
        # TODO: Host_Driver_Comm document suggests this reads back 3 bytes
        return self.read('CMD_STAT', number_of_bytes=3)

    def download_init(self):

        dev_addr = self.ADDRESS  
        reg_addr = self.registers['CMD_STAT'].address
        # TODO: why 0x29 - specified by host driver comms
        data = [0x14, 0x01, 0x29, 0xC1]
        self.i2c_write_long(dev_addr, [reg_addr],
                            len(data), data)

    def ramremap_reset(self):

        dev_addr = self.ADDRESS  
        reg_addr = self.registers['CMD_STAT'].address
        data = [0x11, 0x00, 0xEE]
        self.i2c_write_long(dev_addr, [reg_addr],
                            len(data), data)

    def read_by_addr(self, reg_addr, num_bytes=1):
        dev_addr = self.ADDRESS
        return self.i2c_read_long(dev_addr, [reg_addr],
                                  data_length=num_bytes)

    def save_histogram(self):
        """
           reads and returns raw histograms data for 8x8 capture
           sends command to measure
           does not record header or sub-header

           Returns
           -------
           hist : dictionary of raw histogram data refer to datasheet for ordering
           meas : dictionary of raw measurement data
           """
        hist = {}
        measure = {}
        start_time = time.time()
        self.write(0x10, 'CMD_STAT')
        sleep(0.01)
        for sub in range(4):
            cnt = 0
            bit_3 = 0
            bit_1 = 0
            cnt_limit = 100
            for i in range(60):
                while (cnt < cnt_limit) and (bit_3 == 0):  # checks if bit_3 has changed
                    st = self.read_by_addr(0xe1)
                    bit_3 = st[0] & 0x08  # check bit 3
                    cnt = cnt + 1
                    if cnt == cnt_limit:
                        print('Timeout waiting for INT4 HIST')
                print(str(time.time() - start_time))
                self.write(st[0], 'INT_STATUS')
                buf, e = self.i2c_read_long(self.ADDRESS, [0x27], data_length=128, data_transfer='pipe')
                hist[i + sub * 60] = np.asarray(buf)
                print(str(time.time() - start_time))
            cnt = 0
            while (cnt < cnt_limit) and (bit_1 == 0):  # checks if bit_1 has changed
                st = self.read_by_addr(0xe1)
                bit_1 = st[0] & 0x02  # check bit 1 (int2 bit)
                cnt = cnt + 1
                if cnt == cnt_limit:
                    print('Timeout waiting for INT2 MEAS')
            print(str(time.time() - start_time))
            self.write(st[0], 'INT_STATUS')
            buf, e = self.i2c_read_long(self.ADDRESS, [0x24], data_length=128, data_transfer='pipe')
            measure[sub] = np.asarray(buf)
            print(str(time.time() - start_time))
        self.write(0xff, 'CMD_STAT')
        stop_time = time.time()
        print('TIME TO CAPTURE:' + str(stop_time - start_time) + ' sec')
        return hist, measure


"""
Definitions to add
    - Process_measurement
    - Process_ Histogram
    - Capture_to_HDF5
    - 
"""

