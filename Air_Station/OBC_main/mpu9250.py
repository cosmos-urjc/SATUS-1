import smbus
import time
import math

class MPU9250:
    def __init__(self, bus_num=1, mpu_addr=0x68):
        self.bus = smbus.SMBus(bus_num)
        time.sleep(1)
        self.mpu_addr = mpu_addr
        self.mag_addr = 0x0C  # Magnetometer address
        
        # Initialize MPU-9250
        self._init_mpu()
        #self._init_magnetometer()
        
        # Calibration values (you should run calibration to get these)
        self.mag_calibration = [0, 0, 0]  # Replace with your calibration
        
    def _init_mpu(self):
        """Initialize the MPU-9250 accelerometer and gyroscope"""
        # Wake up the device
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x00)
        time.sleep(0.1)
        
        # Configure gyro range (±1000 dps)
        self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x10)
        # Configure accel range (±8g)
        self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x10)
        # Configure DLPF (bandwidth 184Hz)
        self.bus.write_byte_data(self.mpu_addr, 0x1A, 0x01)
        time.sleep(0.1)
        
    def _init_magnetometer(self):
        """Initialize the AK8963 magnetometer"""
        # Enable bypass mode to access magnetometer directly
        self.bus.write_byte_data(self.mpu_addr, 0x37, 0x02)
        time.sleep(0.1)
        
        # Check WHO_AM_I register
        whoami = self.bus.read_byte_data(self.mag_addr, 0x00)
        time.sleep(1)
        if whoami != 0x48:
            raise RuntimeError("Magnetometer not found or wrong ID")
        
        # Set magnetometer to continuous mode 2 (100Hz)
        self.bus.write_byte_data(self.mag_addr, 0x0A, 0x16)
        time.sleep(0.1)
        
    def read_accel(self):
        """Read accelerometer data in m/s²"""
        data = self.bus.read_i2c_block_data(self.mpu_addr, 0x3B, 6)
        x = self._bytes_to_int(data[0], data[1]) / 4096.0 * 9.80665
        y = self._bytes_to_int(data[2], data[3]) / 4096.0 * 9.80665
        z = self._bytes_to_int(data[4], data[5]) / 4096.0 * 9.80665 
        return [x, y, z]
        
    def read_gyro(self):
        """Read gyroscope data in rad/s"""
        data = self.bus.read_i2c_block_data(self.mpu_addr, 0x43, 6)
        x = self._bytes_to_int(data[0], data[1]) / 32.8 * (math.pi/180)
        y = self._bytes_to_int(data[2], data[3]) / 32.8 * (math.pi/180)
        z = self._bytes_to_int(data[4], data[5]) / 32.8 * (math.pi/180)
        return [x, y, z]
        
    def read_magnetometer(self):
        """Read magnetometer data in μT"""
        # Check data ready status
        status = self.bus.read_byte_data(self.mag_addr, 0x02)
        if (status & 0x01) == 0:
            return None  # Data not ready
            
        data = self.bus.read_i2c_block_data(self.mag_addr, 0x03, 7)
        
        # Check for overflow
        if (data[6] & 0x08) == 0x08:
            return None  # Magnetic overflow
            
        x = self._bytes_to_int(data[0], data[1]) * 0.15 - self.mag_calibration[0]
        y = self._bytes_to_int(data[2], data[3]) * 0.15 - self.mag_calibration[1]
        z = self._bytes_to_int(data[4], data[5]) * 0.15 - self.mag_calibration[2]
        return [x, y, z]
        
    def get_yaw(self):
        """Calculate yaw angle from magnetometer (in degrees)"""
        mag = self.read_magnetometer()
        if mag is None:
            return None
            
        # Calculate yaw (azimuth)
        yaw = math.atan2(mag[1], mag[0]) * (180 / math.pi)
        if yaw < 0:
            yaw += 360
        return yaw
        
    def _bytes_to_int(self, msb, lsb):
        """Convert two bytes to signed integer"""
        val = (msb << 8) | lsb
        return val if val < 32768 else val - 65536
        
    def calibrate_magnetometer(self, samples=500):
        """Simple magnetometer calibration"""
        print("Move sensor in figure-8 pattern for calibration...")
        min_x = max_x = min_y = max_y = min_z = max_z = 0
        
        for i in range(samples):
            mag = self.read_magnetometer()
            if mag:
                min_x = min(min_x, mag[0])
                max_x = max(max_x, mag[0])
                min_y = min(min_y, mag[1])
                max_y = max(max_y, mag[1])
                min_z = min(min_z, mag[2])
                max_z = max(max_z, mag[2])
            time.sleep(0.05)
            
        self.mag_calibration = [
            (max_x + min_x) / 2,
            (max_y + min_y) / 2,
            (max_z + min_z) / 2
        ]
        print(f"Calibration complete. Offsets: {self.mag_calibration}")

# Example usage
if __name__ == "__main__":
    try:
        mpu = MPU9250()
        # mpu.calibrate_magnetometer()  # Run this once in your actual code
        
        while True:
            accel = mpu.read_accel()
            gyro = mpu.read_gyro()
            # mag = mpu.read_magnetometer()
            # yaw = mpu.get_yaw()
            
            print(f"Accel: {accel} m/s²")
            print(f"Gyro: {gyro} rad/s")
            #print(f"Mag: {mag} μT")
            #print(f"Yaw: {yaw}°")
            print("------------------")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
