import board
import busio
import digitalio
import cbor2
import adafruit_rfm9x
import time
import serial

# RFM9X LoRa radio configuration
CS = digitalio.DigitalInOut(board.CE1)     # Chip Select (puede ser D5, D6, etc.)
RESET = digitalio.DigitalInOut(board.D25)  # Reset del RFM95
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in MHz.

# Initialize LoRa radio
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
except RuntimeError as error:
    print('Couldnt initialize LoRa', error)
    exit()

print("LoRa radio initializated, waiting for data.....\n")
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)  # Ajusta la velocidad de baudios según necesites
while True:
    packet = rfm9x.receive(timeout=5.0)  # wait up to 5 seconds for other package
    if packet is not None:
        try:
            data = cbor2.loads(packet)
            print("Data received!")
            print(data)
            print()
            # Enviar datos por serial
            try:
                # Convertir los datos a formato string (o el formato que prefieras)
                serial_data = packet + b'\n'  # Añade un salto de línea para separar mensajes
                ser.write(serial_data)
                print("Data sent via serial")
            except Exception as serial_error:
                print("Error sending data via serial")
        except Exception as e:
            print("error with CBOR protocol")
    else:
        print("Waiting for data.....")

    time.sleep(0.03)