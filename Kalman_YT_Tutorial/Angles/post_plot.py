import csv
import matplotlib.pyplot as plt

# Especifica el nombre del archivo CSV que quieres leer
n = int(input("Número de archivo: "))
Nombre_del_archivo = f'Flight_Data_angles_{n}.csv'

def graficar_datos_desde_csv(nombre_archivo):
    """Lee el archivo CSV y grafica los datos de roll y pitch."""
    tiempos = []
    rolls = []
    pitches = []

    # Lee el archivo CSV y almacena los datos
    with open(nombre_archivo, mode='r') as archivo_csv:
        lector = csv.reader(archivo_csv)
        next(lector)  # Omitir el encabezado
        for fila in lector:
            if len(fila) == 3:  # Verificar si la fila tiene los tres valores (tiempo, roll, pitch)
                tiempo, roll, pitch = fila
                tiempos.append(float(tiempo))
                rolls.append(float(roll))
                pitches.append(float(pitch))

    # Crear el gráfico
    plt.figure(figsize=(12, 8))
    plt.plot(tiempos, rolls, 'cyan', label='Roll')
    plt.plot(tiempos, pitches, 'magenta', label='Pitch')

    plt.xlabel('Tiempo (s)', color='white', fontsize=14, fontweight='bold')
    plt.ylabel('Ángulos (°)', color='white', fontsize=14, fontweight='bold')
    plt.title('Roll y Pitch en el Tiempo', color='white', fontsize=18, fontweight='bold')

    plt.grid(True, color='white', linestyle='--', linewidth=0.5)
    plt.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')
    plt.gca().set_facecolor('black')  # Color de fondo negro

    plt.tick_params(axis='both', colors='white', labelsize=12)
    plt.show()

# Llamar a la función para graficar los datos
graficar_datos_desde_csv(Nombre_del_archivo)
