import csv
import matplotlib.pyplot as plt

# Especifica el nombre del archivo CSV que quieres leer
n = int(input("Número de archivo: "))
Nombre_del_archivo = f'lpf_kf_a_g_m_{n}.csv'

def graficar_datos_desde_csv(nombre_archivo):
    """Lee el archivo CSV y grafica los datos de kalmanPitch, kalmanRoll, kalmanYaw."""
    tiempos = []
    kalmanPitch = []
    kalmanRoll = []
    kalmanYaw = []

    # Lee el archivo CSV y almacena los datos
    with open(nombre_archivo, mode='r') as archivo_csv:
        lector = csv.reader(archivo_csv)
        next(lector)  # Omitir el encabezado
        for fila in lector:
            if len(fila) >= 4:  # Verificar si la fila tiene al menos cuatro valores
                tiempo, kalmanPitch_val, kalmanRoll_val, kalmanYaw_val = fila[:4]
                tiempos.append(float(tiempo))
                kalmanPitch.append(float(kalmanPitch_val))
                kalmanRoll.append(float(kalmanRoll_val))
                kalmanYaw.append(float(kalmanYaw_val))

    # Crear el gráfico
    fig, ax1 = plt.subplots(figsize=(12, 8))

    # Graficar datos
    ax1.plot(tiempos, kalmanPitch, 'green', label='kalmanPitch')
    ax1.plot(tiempos, kalmanRoll, 'yellow', label='kalmanRoll')
    ax1.plot(tiempos, kalmanYaw, 'blue', label='kalmanYaw')

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
