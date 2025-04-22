import csv
import matplotlib.pyplot as plt

# Especifica el nombre del archivo CSV que quieres leer
n = int(input("Número de archivo: "))
Nombre_del_archivo = f'kalman_debug{n}.csv'

def graficar_datos_desde_csv(nombre_archivo):
    """Lee el archivo CSV y grafica los datos de tiempo, roll, pitch, servoRoll y servoPitch."""
    tiempos = []
    rolls = []
    pitches = []
    servoRolls = []
    servoPitches = []

    # Lee el archivo CSV y almacena los datos
    with open(nombre_archivo, mode='r') as archivo_csv:
        lector = csv.reader(archivo_csv)
        next(lector)  # Omitir el encabezado
        for fila in lector:
            if len(fila) >= 5:  # Verificar si la fila tiene al menos cinco valores
                tiempo, roll, pitch, servoRoll, servoPitch = fila[:5]
                tiempos.append(float(tiempo))
                rolls.append(float(roll))
                pitches.append(float(pitch))
                servoRolls.append(float(servoRoll))
                servoPitches.append(float(servoPitch))

    # Crear el gráfico
    fig, ax = plt.subplots(figsize=(12, 8))

    # Graficar datos
    ax.plot(tiempos, rolls, 'cyan', label='Roll')
    ax.plot(tiempos, pitches, 'magenta', label='Pitch')
    ax.plot(tiempos, servoRolls, 'red', label='Servo Roll')
    ax.plot(tiempos, servoPitches, 'blue', label='Servo Pitch')

    # Etiquetas y título
    ax.set_xlabel('Tiempo (s)', color='white', fontsize=14, fontweight='bold')
    ax.set_ylabel('Valores', color='white', fontsize=14, fontweight='bold')
    ax.set_title('Datos en el Tiempo', color='white', fontsize=18, fontweight='bold')

    # Estilo de la cuadrícula y leyenda
    ax.grid(True, color='white', linestyle='--', linewidth=0.5)
    ax.legend(loc='best', fontsize=12, frameon=False, labelcolor='white')

    # **Fondo negro en todo el gráfico**
    fig.patch.set_facecolor('black')  # Fondo negro del gráfico (márgenes)
    ax.set_facecolor('black')  # Fondo negro dentro del área de ploteo

    # Cambiar color de los ejes y labels
    ax.tick_params(axis='both', colors='white', labelsize=12)

    # Mostrar el gráfico
    plt.show()

# Llamar a la función para graficar los datos
graficar_datos_desde_csv(Nombre_del_archivo)
