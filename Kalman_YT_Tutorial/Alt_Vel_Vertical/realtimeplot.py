import serial
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

# Configura el puerto serial y la velocidad de baudios del receptor
n = int(input("Número de archivo: "))
puerto_serial = 'COM11'  # Cambia esto al puerto que estés usando
baud_rate = 57600

# Pide al usuario que especifique el directorio donde guardar el archivo CSV
directorio = input("Especifica el directorio para guardar el archivo CSV (o déjalo en blanco para usar el directorio actual): ")

# Si el directorio está vacío, usa el directorio actual
if not directorio:
    directorio = os.getcwd()  # Obtener el directorio actual


Nombre_del_archivo = os.path.join(directorio, f'Flight_Data_angles_{n}.csv')

# Tamaño del buffer de datos
BUFFER_SIZE = 100

def inicializar_csv(nombre_archivo):
    """Inicializa el archivo CSV con encabezados si no existe."""
    if not os.path.isfile(nombre_archivo):
        with open(nombre_archivo, mode='w', newline='') as archivo_csv:
            escritor = csv.writer(archivo_csv)
            escritor.writerow(['Tiempo', 'Altura', 'Velocidad'])  # Encabezados de las columnas

def graficar_datos_en_tiempo_real(dato_lista):
    """Configura y muestra gráficos de altura y velocidad en tiempo real en el mismo gráfico."""
    fig, ax1 = plt.subplots(figsize=(12, 8))  # Usar solo un eje (ax1)
    fig.patch.set_facecolor('black')
    ax1.set_facecolor('black')  # Color de fondo del gráfico

    # Inicializa las líneas para altura y velocidad
    line_altura, = ax1.plot([], [], 'cyan', label='Altura')
    line_velocidad, = ax1.plot([], [], 'magenta', label='Velocidad')

    # Configura los gráficos
    ax1.set_xlim(0, 10)  # Rango fijo de X
    ax1.set_ylim(-10, 5000)  # Inicialmente el rango de Y para el altura

    ax1.set_xlabel('Tiempo', color='white', fontsize=14, fontweight='bold')
    ax1.set_ylabel('m | m/s', color='white', fontsize=14, fontweight='bold')
    ax1.set_title('Altura y Velocidad en Tiempo Real', color='white', fontsize=18, fontweight='bold')

    ax1.grid(True, color='white', linestyle='--', linewidth=0.5)
    ax1.tick_params(axis='both', colors='white', labelsize=12)
    ax1.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')

    def init():
        line_altura.set_data([], [])
        line_velocidad.set_data([], [])
        return line_altura, line_velocidad

    def update(frame):
        try:
            # Obtener los datos de la lista
            if not dato_lista:
                return line_altura, line_velocidad

            tiempo, altura, velocidad = zip(*dato_lista)

            # Convertir a listas para poder usar en set_data
            tiempo = list(tiempo)
            altura = list(altura)
            velocidad = list(velocidad)

            # Ajusta los límites del eje Y según los datos actuales
            ax1.set_xlim(min(tiempo), max(tiempo) + 1)

            # Actualiza las líneas de altura y velocidad
            line_altura.set_data(tiempo, altura)
            line_velocidad.set_data(tiempo, velocidad)

        except Exception as e:
            print(f"Error al actualizar el gráfico: {e}")

        return line_altura, line_velocidad

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=1)  # Set a faster interval here
    plt.show()

def main():
    dato_lista = []  # Lista para almacenar todos los datos
    ultima_altura_valido = None  # Para guardar el último altura válido
    ultima_velocidad_valido = None  # Para guardar el último velocidad válido
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

                # Verificar si el formato es el esperado: altura y velocidad
                if len(medicion) == 2:
                    altura = float(medicion[0])
                    velocidad = float(medicion[1])

                    # Calcular el tiempo basado en la frecuencia de lectura (0.004 ms)
                    tiempo_actual += 0.004  # Incrementa el tiempo en 0.004 ms por cada lectura

                    # Verificar si el altura o velocidad son cero
                    if altura == 0 and ultima_altura_valido is not None:
                        altura = ultima_altura_valido  # Reemplaza con el último altura válido
                    if velocidad == 0 and ultima_velocidad_valido is not None:
                        velocidad = ultima_velocidad_valido  # Reemplaza con el último velocidad válido

                    # Si el altura o velocidad no son cero, actualiza los valores válidos
                    if altura != 0:
                        ultima_altura_valido = altura
                    if velocidad != 0:
                        ultima_velocidad_valido = velocidad

                    # Guardar los datos en el archivo CSV
                    with open(Nombre_del_archivo, mode='a', newline='') as archivo_csv:
                        escritor = csv.writer(archivo_csv)
                        escritor.writerow([tiempo_actual, altura, velocidad])

                    # Agrega los datos a la lista
                    dato_lista.append((tiempo_actual, altura, velocidad))

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
