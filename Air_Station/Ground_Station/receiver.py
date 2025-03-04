import time
import board
import busio
import digitalio
import adafruit_rfm9x

# Define radio parameters
RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in MHz. 868.0 MHz in Europe, 915.0 MHz in North America.

# Declare pins for the board
CS = digitalio.DigitalInOut(board.CE0) 
RESET = digitalio.DigitalInOut(board.D25)
INT = digitalio.DigitalInOut(board.D27)


# Initialize SPI bus
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize RFM9x radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, crc=False)

# start receiving packages
count=0
while True:
        count=count+1
        packet = rfm9x.receive() 
        print(packet) # Wait for a packet to be received (up to 0.5 seconds)
        if packet is not None:
            packet_text = str(packet, 'ascii')
            print('Received: {0}'.format(packet_text))
        else:
             print('Received nothing!')
        #time.sleep(6)  # Wait for some time before sending the next command