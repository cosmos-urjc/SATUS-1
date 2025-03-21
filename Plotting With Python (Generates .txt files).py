import serial
import matplotlib.pyplot as plt
from collections import deque
import re

# Configuración del puerto serial
port = 'COM3'  # Cambia esto al puerto correcto
baudrate = 115200

# Inicializa el puerto serial
ser = serial.Serial(port, baudrate, timeout=1)

# Variables para almacenar los datos
time_data = deque(maxlen=100)  # Almacena los últimos 100 puntos
temperature_data = deque(maxlen=100)
pressure_data = deque(maxlen=100)
altitude_data = deque(maxlen=100)

# Archivos de texto para guardar los datos
temperature_file = open("temperature.txt", "w")
pressure_file = open("pressure.txt", "w")
altitude_file = open("altitude.txt", "w")

# Función para extraer los valores de los datos recibidos
def parse_data(line):
    # Usamos expresiones regulares para extraer los valores
    temp_match = re.search(r"Temperature = ([\d.-]+)\*C", line)
    press_match = re.search(r"Pressure = ([\d.-]+)hPa", line)
    alt_match = re.search(r"Approx\. Altitude = ([\d.-]+)m", line)

    if temp_match and press_match and alt_match:
        temperature = float(temp_match.group(1))
        pressure = float(press_match.group(1))
        altitude = float(alt_match.group(1))
        return temperature, pressure, altitude
    return None

# Configuración de la gráfica
plt.ion()  # Modo interactivo
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle("Datos del Sensor BME280")

# Inicializa las líneas de la gráfica
line_temp, = ax1.plot([], [], label="Temperatura (°C)")
line_press, = ax2.plot([], [], label="Presión (hPa)")
line_alt, = ax3.plot([], [], label="Altitud (m)")

# Configura los ejes
ax1.set_ylabel("Temperatura (°C)")
ax2.set_ylabel("Presión (hPa)")
ax3.set_ylabel("Altitud (m)")
ax3.set_xlabel("Tiempo")

# Mostrar leyendas
ax1.legend()
ax2.legend()
ax3.legend()

# Función para actualizar la gráfica
def update_plot():
    line_temp.set_data(range(len(temperature_data)), temperature_data)
    line_press.set_data(range(len(pressure_data)), pressure_data)
    line_alt.set_data(range(len(altitude_data)), altitude_data)

    # Ajustar los límites de los ejes
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()
    ax3.relim()
    ax3.autoscale_view()

    # Redibujar la gráfica
    fig.canvas.flush_events()

# Bucle principal
try:
    time_step = 0
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(line)  # Opcional: imprime los datos en la consola

            # Procesa los datos
            parsed_data = parse_data(line)
            if parsed_data:
                temperature, pressure, altitude = parsed_data

                # Almacena los datos
                temperature_data.append(temperature)
                pressure_data.append(pressure)
                altitude_data.append(altitude)
                time_data.append(time_step)
                time_step += 1

                # Guarda los datos en los archivos de texto
                temperature_file.write(f"{temperature}\n")
                pressure_file.write(f"{pressure}\n")
                altitude_file.write(f"{altitude}\n")

                # Actualiza la gráfica
                update_plot()

except KeyboardInterrupt:
    print("Programa terminado")

finally:
    # Cierra los archivos de texto
    temperature_file.close()
    pressure_file.close()
    altitude_file.close()

    # Cierra el puerto serial
    ser.close()