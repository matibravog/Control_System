import serial
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

n = int(input("Número de archivo: "))
puerto_serial = 'COM11'
baud_rate = 115200

directorio = input("Especifica el directorio para guardar el archivo CSV (o déjalo en blanco para usar el directorio actual): ")
if not directorio:
    directorio = os.getcwd()

Nombre_del_archivo = os.path.join(directorio, f'inertial_accel_{n}.csv')
BUFFER_SIZE = 300

def inicializar_csv(nombre_archivo):
    if not os.path.isfile(nombre_archivo):
        with open(nombre_archivo, mode='w', newline='') as archivo_csv:
            escritor = csv.writer(archivo_csv)
            escritor.writerow(['Tiempo', 'kalmanPitch', 'kalmanRoll', 'kalmanYaw',
                               'ax_inertial', 'ay_inertial', 'az_inertial',
                               'vx', 'vy', 'vz', 'px', 'py', 'pz'])

def graficar_datos_en_tiempo_real(dato_lista):
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor('black')
    axs = axs.flatten()

    colores = ['green', 'yellow', 'blue', 'red', 'cyan', 'magenta', 'orange', 'lime', 'white', 'violet', 'deepskyblue', 'tomato']
    etiquetas = ['kalmanPitch', 'kalmanRoll', 'kalmanYaw',
                 'ax_inertial', 'ay_inertial', 'az_inertial',
                 'vx', 'vy', 'vz',
                 'px', 'py', 'pz']

    indices_subplots = [
        [1, 2, 3],     # Ángulos
        [4, 5, 6],     # Aceleración
        [7, 8, 9],     # Velocidad
        [10, 11, 12]   # Posición
    ]

    lineas = []
    for i in range(4):
        ax = axs[i]
        ax.set_facecolor('black')
        ax.set_title(['Ángulos', 'Aceleración', 'Velocidad', 'Posición'][i], color='white', fontsize=16, fontweight='bold')
        ax.set_xlabel('Tiempo (s)', color='white', fontsize=12, fontweight='bold')
        ax.set_ylabel('Valor', color='white', fontsize=12, fontweight='bold')
        ax.grid(True, color='white', linestyle='--', linewidth=0.5)
        ax.tick_params(axis='both', colors='white', labelsize=10)
        ax.set_ylim(-100, 100)  # y fijo

        lines = []
        for j in indices_subplots[i]:
            line, = ax.plot([], [], label=etiquetas[j-1], color=colores[j-1], linewidth=1)
            lines.append(line)
        ax.legend(loc='upper right', fontsize=10, frameon=False, labelcolor='white')
        lineas.append(lines)

    def init():
        for grupo in lineas:
            for line in grupo:
                line.set_data([], [])
        return [l for grupo in lineas for l in grupo]

    def update(frame):
        try:
            if not dato_lista:
                return [l for grupo in lineas for l in grupo]

            datos_transpuestos = list(zip(*dato_lista))
            tiempo = list(datos_transpuestos[0])
            t_max = tiempo[-1]
            t_min = max(0, t_max - 5)

            # Filtrar solo los datos dentro de la ventana de 5 segundos
            indices = [i for i, t in enumerate(tiempo) if t_min <= t <= t_max]
            tiempo_filtrado = [tiempo[i] for i in indices]

            datos_filtrados = []
            for serie in datos_transpuestos:
                datos_filtrados.append([serie[i] for i in indices])

            for i in range(4):
                axs[i].set_xlim(t_min, t_max)  # x dinámico
                for j, idx in enumerate(indices_subplots[i]):
                    lineas[i][j].set_data(tiempo_filtrado, datos_filtrados[idx])

        except Exception as e:
            print(f"Error en update: {e}")

        return [l for grupo in lineas for l in grupo]

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

def main():
    dato_lista = []
    tiempo_actual = 0
    puerto = None

    try:
        puerto = serial.Serial(puerto_serial, baud_rate)
        print("Leyendo datos del puerto serial...")

        inicializar_csv(Nombre_del_archivo)
        graficar_thread = threading.Thread(target=graficar_datos_en_tiempo_real, args=(dato_lista,))
        graficar_thread.start()

        while True:
            try:
                dato = puerto.readline().decode().strip()
                print(f"Raw: {dato}")
                medicion = dato.split(',')

                if len(medicion) == 12:
                    datos_float = list(map(float, medicion))
                    tiempo_actual += 0.02

                    with open(Nombre_del_archivo, mode='a', newline='') as archivo_csv:
                        escritor = csv.writer(archivo_csv)
                        escritor.writerow([tiempo_actual] + datos_float)

                    dato_lista.append([tiempo_actual] + datos_float)
                    if len(dato_lista) > BUFFER_SIZE:
                        dato_lista.pop(0)

                    print(f"Guardado: {datos_float}")
                else:
                    print("Error: Se esperaban 12 valores.")

            except ValueError:
                print("Error: Conversión a float fallida.")

    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")

    except KeyboardInterrupt:
        print("Lectura detenida por el usuario.")

    finally:
        if puerto and puerto.is_open:
            puerto.close()
        print("Puerto serial cerrado.")

if __name__ == "__main__":
    main()
