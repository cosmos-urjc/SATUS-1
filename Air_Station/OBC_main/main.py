import time
import smbus
from bmp180 import bmp180
import board
import busio
import digitalio
import adafruit_rfm9x
import adafruit_adxl34x
import cbor2
import os
import csv
from datetime import datetime
from mpu9250 import MPU9250


# locally save sensor data in CSV format
def save_data(ARCHIVO,datos):
    with open(ARCHIVO, 'a') as f:
        csv.writer(f).writerow([
            datos['altitude'],
            datos['acc_x'],
            datos['acc_y'],
            datos['acc_z']
        ])
        f.flush()  
def inicializar_archivo(ARCHIVE, HEADER):
    """create a CSV file with the header"""
    with open(ARCHIVE, 'w', newline='') as f:
        csv.writer(f).writerow(HEADER)


def main():
  
    i2c = board.I2C()
    sensor = bmp180(address=0x77)
    mpu = MPU9250(i2c, address=0x68)

    print("BMP180 initialized")
    # Standard sea level pressure in Pascals
    STANDARD_SEA_LEVEL_PRESSURE_PA = 101325.0
    # Define radio parameters
    RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in MHz. 868.0 MHz in Europe, 915.0 MHz in North America.

    # Declare pins for the board
    CS = digitalio.DigitalInOut(board.CE1) 
    RESET = digitalio.DigitalInOut(board.D25)
   


    # Initialize SPI bus
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    # Initialize RFM9x radio
    try:
        accelerometer = adafruit_adxl34x.ADXL343(i2c)
        print("accelerometer initialized")
    except RuntimeError as error:
        print('Could not initialize accelerometer', error)
        accelerometer = None
      # Configuración básica
    try:
        rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        print("RFM9x initialized")
    except RuntimeError as error:
        print('Could not initialize RFM9x', error)
        rfm9x = None
    ARCHIVE = datetime.now().strftime("/home/cosmos/OBC/%Y-%m-%d_%H-%M-%S_sensores.csv")
    HEADER = ['altitud', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']

    # Crear archivo si no existe
    inicializar_archivo(ARCHIVE, HEADER)
    if not os.path.exists(ARCHIVE):
        with open(ARCHIVE, 'w') as f:
            csv.writer(f).writerow(HEADER)

    while True:

        if accelerometer is not None:
            try:
                accel = accelerometer.acceleration
            except RuntimeError as e:
                print("Error reading accelerometer:", e)
            if accel is None:
                accel = (-1, -1, -1) # Handle error
                continue
            try:
                gyro = mpu.get_gyro_data() 
            except RuntimeError as e:
                print("Error reading gyroscope:", e)  
            if gyro is None:
                gyro = [-1, -1, -1]
                continue
        else:
            accel = (-1, -1, -1)
            #gyro = [-1, -1, -1]
        if sensor is not None:
            try:
                altitude = sensor.get_altitude()
            except RuntimeError as e:
                print("Error reading altitude:", e)
            if altitude is None:
                altitude = -1
        else:
            altitude = -1
        Data_package = {
                "altitude": altitude,
                "acc_x": accel[0],
                "acc_y": accel[1],
                "acc_z": accel[2],
                "gyro_x": gyro[0],
                "gyro_y": gyro[1],
                "gyro_z": gyro[2]}
        
        message = cbor2.dumps(Data_package)
        rfm9x.send(message)
        print("Data sent")
        time.sleep(0.05)
        print(Data_package)
        save_data(ARCHIVE,Data_package)

if __name__ == "__main__":
    main()
        

       
            
            

