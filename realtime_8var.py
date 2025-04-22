import serial
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

# Configura el puerto serial y la velocidad de baudios del receptor
n = int(input("Número de archivo: "))
puerto_serial = 'COM4'  # Cambia esto al puerto que estés usando
baud_rate = 115200

# Pide al usuario que especifique el directorio donde guardar el archivo CSV
directorio = input("Especifica el directorio para guardar el archivo CSV (o déjalo en blanco para usar el directorio actual): ")

# Si el directorio está vacío, usa el directorio actual
if not directorio:
    directorio = os.getcwd()  # Obtener el directorio actual

Nombre_del_archivo = os.path.join(directorio, f'kalman_debug{n}.csv')

# Tamaño del buffer de datos
BUFFER_SIZE = 100

def inicializar_csv(nombre_archivo):
    """Inicializa el archivo CSV con encabezados si no existe."""
    if not os.path.isfile(nombre_archivo):
        with open(nombre_archivo, mode='w', newline='') as archivo_csv:
            escritor = csv.writer(archivo_csv)
            escritor.writerow(['Tiempo', 'yaw', 'pitch', 'servoYaw', 'servoPitch'])  # Encabezados de las columnas

def graficar_datos_en_tiempo_real(dato_lista):
    """Configura y muestra gráficos de yaw y pitch en tiempo real en el mismo gráfico."""
    fig, ax1 = plt.subplots(figsize=(12, 8))  # Usar solo un eje (ax1)
    fig.patch.set_facecolor('black')
    ax1.set_facecolor('black')  # Color de fondo del gráfico

    # Inicializa las líneas para yaw y pitch
    line_yaw, = ax1.plot([], [], 'red', label='yaw')
    line_pitch, = ax1.plot([], [], 'blue', label='pitch')
    line_servoYaw, = ax1.plot([], [], 'brown', label='servoYaw')
    line_servoPitch, = ax1.plot([], [], 'pink', label='servoPitch')

    # Configura los gráficos
    ax1.set_xlim(0, 10)  # Rango fijo de X
    ax1.set_ylim(-1000, 1000)  # Inicialmente el rango de Y para el yaw

    ax1.set_xlabel('Tiempo', color='white', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Ángulos (°)', color='white', fontsize=14, fontweight='bold')
    ax1.set_title('Yaw y Pitch en Tiempo Real', color='white', fontsize=18, fontweight='bold')

    ax1.grid(True, color='white', linestyle='--', linewidth=0.5)
    ax1.tick_params(axis='both', colors='white', labelsize=12)
    ax1.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')

    def init():
        line_yaw.set_data([], [])
        line_pitch.set_data([], [])
        line_servoYaw.set_data([], [])
        line_servoPitch.set_data([], [])
        return line_yaw, line_pitch, line_servoYaw, line_servoPitch

    def update(frame):
        try:
            # Obtener los datos de la lista
            if not dato_lista:
                return line_yaw, line_pitch, line_servoYaw, line_servoPitch

            tiempo, yaw, pitch, servo_yaw, servo_pitch = zip(*dato_lista)

            # Convertir a listas para poder usar en set_data
            tiempo = list(tiempo)
            yaw = list(yaw)
            pitch = list(pitch)
            servo_yaw = list(servo_yaw)
            servo_pitch = list(servo_pitch)

            # Ajusta los límites del eje Y según los datos actuales
            ax1.set_xlim(min(tiempo), max(tiempo) + 1)

            # Actualiza las líneas de yaw y pitch
            line_yaw.set_data(tiempo, yaw)
            line_pitch.set_data(tiempo, pitch)
            line_servoYaw.set_data(tiempo, servo_yaw)
            line_servoPitch.set_data(tiempo, servo_pitch)

        except Exception as e:
            print(f"Error al actualizar el gráfico: {e}")

        return line_yaw, line_pitch, line_servoYaw, line_servoPitch

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=1)  # Set a faster interval here
    plt.show()


def main():
    dato_lista = []  # Lista para almacenar todos los datos
    ultima_yaw_valido = None  # Para guardar el último yaw válido
    ultima_pitch_valido = None  # Para guardar el último pitch válido
    tiempo_actual = 0  # Inicializa el tiempo

    puerto = None  # Define puerto outside the try block

    try:
        puerto = serial.Serial(puerto_serial, baud_rate)
        print("Leyendo datos del puerto serial y guardándolos en tiempo real en el archivo CSV...")
        
        inicializar_csv(Nombre_del_archivo)

        # Inicia la gráfica en tiempo real en el hilo principal
        graficar_thread = threading.Thread(target=graficar_datos_en_tiempo_real, args=(dato_lista,))
        graficar_thread.start()

        while True:
            try:
                dato = puerto.readline().decode().strip()
                print(f"Raw data received: {dato}")  # Debugging output
                medicion = dato.split(',')

                # Verificar si el formato es el esperado: yaw y pitch
                if len(medicion) == 4:
                    yaw = float(medicion[0])
                    pitch = float(medicion[1])
                    servo_yaw = float(medicion[2])
                    servo_pitch = float(medicion[3])

                    # Calcular el tiempo basado en la frecuencia de lectura (0.004 ms)
                    tiempo_actual += 0.04  # Incrementa el tiempo en 0.004 ms por cada lectura

                    # Guardar los datos en el archivo CSV
                    with open(Nombre_del_archivo, mode='a', newline='') as archivo_csv:
                        escritor = csv.writer(archivo_csv)
                        escritor.writerow([tiempo_actual, yaw, pitch,servo_yaw, servo_pitch])  # Escribir los datos en el archivo CSV

                    # Agrega los datos a la lista
                    dato_lista.append((tiempo_actual, yaw, pitch,servo_yaw, servo_pitch))

                    # Limita la longitud de la lista para no usar demasiada memoria
                    if len(dato_lista) > BUFFER_SIZE:
                        dato_lista.pop(0)  # Elimina el dato más antiguo si la lista está llena

                    print(f'Dato recibido y guardado: {dato}')
                else:
                    print("Error: El formato de los datos no es el esperado.")

            except ValueError:
                print("Error: Los datos recibidos no se pueden convertir a números.")

    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")

    except KeyboardInterrupt:
        print("Lectura detenida por el usuario.")
    
    finally:
        if puerto and puerto.is_open:  # Check if puerto is defined and open
            puerto.close()
        print("Conexión serial cerrada.")

if __name__ == "__main__":
    main()
