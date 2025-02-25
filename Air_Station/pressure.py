import adafruit_bmp085
import busio
import time
import board

i2c1 = busio.I2C(board.SCL, board.SDA)

bmp1 = adafruit_bmp085.Adafruit_BMP085_I2c(i2c1, adress = 0x76)

while True
    temp1 = bmp1.temperature
    pressure1 = bmp1.pressure

    print(f"Sensor: {temp1:.2f} °C, {pressure1:.2f} hPa\n")