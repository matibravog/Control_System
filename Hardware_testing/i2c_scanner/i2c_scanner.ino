#include <Wire.h>

void setup() {
  Wire.begin();               // Iniciar el bus I2C
  Serial.begin(9600);         // Iniciar la comunicación serial
  while (!Serial);            // Esperar a que el monitor serial esté listo (opcional)
  Serial.println("Escaneando dispositivos I2C...");
}

void loop() {
  int nDevices = 0;

  for (byte address = 1; address < 127; address++) { // Las direcciones I2C válidas son de 1 a 127
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en la dirección 0x");
      if (address < 16) Serial.print("0"); // Añadir un cero al inicio si es menor a 0x10
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Error desconocido en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No se encontraron dispositivos I2C.");
  } else {
    Serial.println("Escaneo completado.");
  }

  delay(5000); // Esperar 5 segundos antes de volver a escanear
}
