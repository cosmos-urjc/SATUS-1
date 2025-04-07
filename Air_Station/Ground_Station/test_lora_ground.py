import serial
import cbor2

# Configure serial port for LoRa (adjust port and baud rate)
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

def decode_cbor(data):
    try:
        decoded_data = cbor2.loads(data)
        return decoded_data
    except cbor2.CBORDecodeError:
        print("Invalid CBOR data received")
        return None

def main():
    print("Waiting for LoRa data...")
    while True:
        if ser.in_waiting > 0:
            # Read the incoming data
            data = ser.readline()

            # Decode the CBOR data
            telemetry = decode_cbor(data)
            if telemetry:
                print("Received Telemetry Data:")
                print(f"Temperature: {telemetry['temperature']} °C")
                print(f"Humidity: {telemetry['humidity']} %")
                print(f"Battery Voltage: {telemetry['batteryVoltage']} V")
                print("-----------------------------")

if __name__ == "__main__":
    main()