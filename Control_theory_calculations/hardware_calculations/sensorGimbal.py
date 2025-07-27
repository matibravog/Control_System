g = 9.81 # m/s^2

motorTorque = 0.45# Nm
motorMass = 0.35 # kg
d = 0.065 # m

torque = d*2*motorMass*g # Nm
sf = motorTorque/torque # scale factor

# print(f"Torque: {torque} Nm")
print(f"requierd Torque to move 1 motor: {torque*100} N cm")
print(f"Scale factor: {sf}")


