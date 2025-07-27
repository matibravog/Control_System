import numpy as np
import matplotlib.pyplot as plt

# Constants
inner_radius = 0.05  # Inner radius of the cylinder in meters (5 cm)
outer_radius = inner_radius + 0.0046  # Outer radius of the cylinder in meters (4.6 mm wall thickness)
thermal_conductivity = 25.2  # Thermal conductivity of steel alloy in W/(m·K)
density = 7850  # Density of steel alloy in kg/m³
specific_heat = 500  # Specific heat capacity of steel alloy in J/(kg·K)
ambient_temperature = 25  # Ambient temperature in °C
initial_internal_temperature = 400  # Initial internal temperature in °C
time_step = 0.1  # Time step in seconds
total_time = 100  # Total simulation time in seconds
num_radial_points = 50  # Number of radial points for discretization

# Radial discretization
radii = np.linspace(inner_radius, outer_radius, num_radial_points)
dr = radii[1] - radii[0]

# Initialize temperature array
temperatures = np.ones(num_radial_points) * ambient_temperature
temperatures[0] = initial_internal_temperature  # Set the internal face temperature

# Time integration using the explicit finite difference method
time_points = np.arange(0, total_time, time_step)
temperature_history = []

for t in time_points:
    new_temperatures = temperatures.copy()
    for i in range(1, num_radial_points - 1):
        # Heat conduction equation in cylindrical coordinates
        new_temperatures[i] = temperatures[i] + (
            thermal_conductivity * time_step / (density * specific_heat * dr**2)
        ) * (
            (temperatures[i + 1] - 2 * temperatures[i] + temperatures[i - 1])
            + (1 / radii[i]) * (temperatures[i + 1] - temperatures[i - 1]) / (2 * dr)
        )
    # Boundary conditions
    new_temperatures[0] = initial_internal_temperature  # Internal face
    new_temperatures[-1] = ambient_temperature  # External face
    temperatures = new_temperatures
    temperature_history.append(temperatures.copy())

# Convert temperature history to a NumPy array for plotting
temperature_history = np.array(temperature_history)

# Plot the transient temperature distribution
plt.figure(figsize=(10, 6))
for i in range(0, len(time_points), len(time_points) // 10):  # Plot 10 time steps
    plt.plot(radii, temperature_history[i], label=f"t = {time_points[i]:.1f} s")
plt.xlabel("Radius (m)")
plt.ylabel("Temperature (°C)")
plt.title("Transient Temperature Distribution in Cylindrical Wall")
plt.legend()
plt.grid(True)
plt.show()

# Plot the temperature at the external surface over time
plt.figure(figsize=(10, 6))
plt.plot(time_points, temperature_history[:, -1], label="External Surface Temperature", color="blue")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (°C)")
plt.title("External Surface Temperature vs Time")
plt.grid(True)
plt.legend()
plt.show()
