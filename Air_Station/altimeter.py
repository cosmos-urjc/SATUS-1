import time
import board
import busio
import adafruit_bmp280

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)

while True:
    temp = bmp280.temperature
    altitude = bmp280.altitude
    pressure = bmp280.pressure

    print("Temperature: %0.1f C" % temp)
    print("Altitude = %0.2f meters" % altitude)
    print("Pressure: %0.1f hPa" % pressure)

    time.sleep(2)


