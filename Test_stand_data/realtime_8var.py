import serial
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

# Configura el puerto serial y la velocidad de baudios del receptor
n = int(input("Número de archivo: "))
puerto_serial = 'COM5'  # Cambia esto al puerto que estés usando
baud_rate = 115200

# Pide al usuario que especifique el directorio donde guardar el archivo CSV
directorio = input("Especifica el directorio para guardar el archivo CSV (o déjalo en blanco para usar el directorio actual): ")

# Si el directorio está vacío, usa el directorio actual
if not directorio:
    directorio = os.getcwd()  # Obtener el directorio actual

Nombre_del_archivo = os.path.join(directorio, f'angles_from_raw_{n}.csv')

# Tamaño del buffer de datos
BUFFER_SIZE = 100

def inicializar_csv(nombre_archivo):
    """Inicializa el archivo CSV con encabezados si no existe."""
    if not os.path.isfile(nombre_archivo):
        with open(nombre_archivo, mode='w', newline='') as archivo_csv:
            escritor = csv.writer(archivo_csv)
            escritor.writerow(['Tiempo', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'])  # Updated headers

def graficar_datos_en_tiempo_real(dato_lista):
    """Configura y muestra gráficos en tiempo real."""
    fig, ax1 = plt.subplots(figsize=(12, 8))  # Usar solo un eje (ax1)
    fig.patch.set_facecolor('black')
    ax1.set_facecolor('black')  # Color de fondo del gráfico

    # Inicializa las líneas para las variables
    line_ax, = ax1.plot([], [], 'green', label='ax')
    line_ay, = ax1.plot([], [], 'yellow', label='ay')
    line_az, = ax1.plot([], [], 'cyan', label='az')
    line_gx, = ax1.plot([], [], 'orange', label='gx')
    line_gy, = ax1.plot([], [], 'purple', label='gy')
    line_gz, = ax1.plot([], [], 'magenta', label='gz')
    line_mx, = ax1.plot([], [], 'red', label='mx')
    line_my, = ax1.plot([], [], 'blue', label='my')
    line_mz, = ax1.plot([], [], 'brown', label='mz')

    # Configura los gráficos
    ax1.set_xlim(0, 10)  # Rango fijo de X
    ax1.set_ylim(-90, 90)  # Inicialmente el rango de Y para las variables

    ax1.set_xlabel('Tiempo', color='white', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Valores', color='white', fontsize=14, fontweight='bold')
    ax1.set_title('Variables en Tiempo Real', color='white', fontsize=18, fontweight='bold')

    ax1.grid(True, color='white', linestyle='--', linewidth=0.5)
    ax1.tick_params(axis='both', colors='white', labelsize=12)
    ax1.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')

    def init():
        line_ax.set_data([], [])
        line_ay.set_data([], [])
        line_az.set_data([], [])
        line_gx.set_data([], [])
        line_gy.set_data([], [])
        line_gz.set_data([], [])
        line_mx.set_data([], [])
        line_my.set_data([], [])
        line_mz.set_data([], [])
        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz, line_mx, line_my, line_mz

    def update(frame):
        try:
            # Obtener los datos de la lista
            if not dato_lista:
                return line_ax, line_ay, line_az, line_gx, line_gy, line_gz, line_mx, line_my, line_mz

            tiempo, ax, ay, az, gx, gy, gz, mx, my, mz = zip(*dato_lista)

            # Convertir a listas para poder usar en set_data
            tiempo = list(tiempo)
            ax = list(ax)
            ay = list(ay)
            az = list(az)
            gx = list(gx)
            gy = list(gy)
            gz = list(gz)
            mx = list(mx)
            my = list(my)
            mz = list(mz)

            # Ajusta los límites del eje Y según los datos actuales
            ax1.set_xlim(min(tiempo), max(tiempo) + 1)

            # Actualiza las líneas de las variables
            line_ax.set_data(tiempo, ax)
            line_ay.set_data(tiempo, ay)
            line_az.set_data(tiempo, az)
            line_gx.set_data(tiempo, gx)
            line_gy.set_data(tiempo, gy)
            line_gz.set_data(tiempo, gz)
            line_mx.set_data(tiempo, mx)
            line_my.set_data(tiempo, my)
            line_mz.set_data(tiempo, mz)

        except Exception as e:
            print(f"Error al actualizar el gráfico: {e}")

        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz, line_mx, line_my, line_mz

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=1)
    plt.show()


def main():
    dato_lista = []  # Lista para almacenar todos los datos
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

                # Verificar si el formato es el esperado
                if len(medicion) == 9:  # Updated to include only the required variables
                    ax = float(medicion[0])
                    ay = float(medicion[1])
                    az = float(medicion[2])
                    gx = float(medicion[3])
                    gy = float(medicion[4])
                    gz = float(medicion[5])
                    mx = float(medicion[6])
                    my = float(medicion[7])
                    mz = float(medicion[8])

                    # Calcular el tiempo basado en la frecuencia de lectura
                    tiempo_actual += 0.04

                    # Guardar los datos en el archivo CSV
                    with open(Nombre_del_archivo, mode='a', newline='') as archivo_csv:
                        escritor = csv.writer(archivo_csv)
                        escritor.writerow([tiempo_actual, ax, ay, az, gx, gy, gz, mx, my, mz])

                    # Agrega los datos a la lista
                    dato_lista.append((tiempo_actual, ax, ay, az, gx, gy, gz, mx, my, mz))

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
