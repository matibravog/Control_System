import csv
import matplotlib.pyplot as plt

# Especifica el nombre del archivo CSV que quieres leer
n = int(input("Número de archivo: "))
Nombre_del_archivo = f'angles_from_raw_{n}.csv'

def graficar_datos_desde_csv(nombre_archivo):
    """Lee el archivo CSV y grafica los datos de ax, ay, az, gx, gy, gz, mx, my, mz."""
    tiempos = []
    ax = []
    ay = []
    az = []
    gx = []
    gy = []
    gz = []
    mx = []
    my = []
    mz = []

    # Lee el archivo CSV y almacena los datos
    with open(nombre_archivo, mode='r') as archivo_csv:
        lector = csv.reader(archivo_csv)
        next(lector)  # Omitir el encabezado
        for fila in lector:
            if len(fila) >= 10:  # Verificar si la fila tiene al menos diez valores
                tiempo, ax_val, ay_val, az_val, gx_val, gy_val, gz_val, mx_val, my_val, mz_val = fila[:10]
                tiempos.append(float(tiempo))
                ax.append(float(ax_val))
                ay.append(float(ay_val))
                az.append(float(az_val))
                gx.append(float(gx_val))
                gy.append(float(gy_val))
                gz.append(float(gz_val))
                mx.append(float(mx_val))
                my.append(float(my_val))
                mz.append(float(mz_val))

    # Crear el gráfico
    fig, ax1 = plt.subplots(figsize=(12, 8))

    # Graficar datos
    ax1.plot(tiempos, ax, 'green', label='ax')
    ax1.plot(tiempos, ay, 'yellow', label='ay')
    ax1.plot(tiempos, az, 'cyan', label='az')
    ax1.plot(tiempos, gx, 'orange', label='gx')
    ax1.plot(tiempos, gy, 'purple', label='gy')
    ax1.plot(tiempos, gz, 'magenta', label='gz')
    ax1.plot(tiempos, mx, 'red', label='mx')
    ax1.plot(tiempos, my, 'blue', label='my')
    ax1.plot(tiempos, mz, 'brown', label='mz')

    # Etiquetas y título
    ax1.set_xlabel('Tiempo (s)', color='white', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Valores', color='white', fontsize=14, fontweight='bold')
    ax1.set_title('Datos en el Tiempo', color='white', fontsize=18, fontweight='bold')

    # Estilo de la cuadrícula y leyenda
    ax1.grid(True, color='white', linestyle='--', linewidth=0.5)
    ax1.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')

    # **Fondo negro en todo el gráfico**
    fig.patch.set_facecolor('black')  # Fondo negro del gráfico (márgenes)
    ax1.set_facecolor('black')  # Fondo negro dentro del área de ploteo

    # Cambiar color de los ejes y labels
    ax1.tick_params(axis='both', colors='white', labelsize=12)

    # Mostrar el gráfico
    plt.show()

# Llamar a la función para graficar los datos
graficar_datos_desde_csv(Nombre_del_archivo)
