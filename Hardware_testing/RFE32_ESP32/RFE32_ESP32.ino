/*

  This example shows how to connect to an EBYTE transceiver
  using an ESP32

  This example shows how to connect to an EBYTE transceiver
  using an ESP32

  This code for for the sender

  ESP32 won't allow SoftwareSerial (at least I can't get that lib to work
  so you will just hardwire your EBYTE directly to the Serial2 port



*/

#include "EBYTE.h"


/*
WARNING: IF USING AN ESP32
DO NOT USE THE PIN NUMBERS PRINTED ON THE BOARD
YOU MUST USE THE ACTUAL GPIO NUMBER
*/
#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)

#define PIN_M0 4    // D4 on the board (possibly pin 24)
#define PIN_M1 2   // D2 on the board (possibly called pin 22)
#define PIN_AX 15   // D15 on the board (possibly called pin 21)


// i recommend putting this code in a .h file and including it
// from both the receiver and sender modules
struct DATA {
  unsigned long Count;
  int Bits;
  float Volts;
  float Amps;

};

// these are just dummy variables, replace with your own
int Chan;
DATA MyData;
unsigned long Last;


// create the transceiver object, passing in the serial and pins
EBYTE Transceiver(&Serial2, PIN_M0, PIN_M1, PIN_AX);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Starting Reader");
  Transceiver.init();
  Transceiver.PrintParameters();


}

void loop() {

  // if the transceiver serial is available, proces incoming data
  // you can also use Transceiver.available()


  if (Serial2.available()) {

    // i highly suggest you send data using structures and not
    // a parsed data--i've always had a hard time getting reliable data using
    // a parsing method

    Transceiver.GetStruct(&MyData, sizeof(MyData));

    // note, you only really need this library to program these EBYTE units
    // you can call readBytes directly on the EBYTE Serial object
    // Serial2.readBytes((uint8_t*)& MyData, (uint8_t) sizeof(MyData));


    // dump out what was just received
    Serial.print("Count: "); Serial.println(MyData.Count);
    Serial.print("Bits: "); Serial.println(MyData.Bits);
    Serial.print("Volts: "); Serial.println(MyData.Volts);
    // if you got data, update the checker
    Last = millis();

  }
  else {
    // if the time checker is over some prescribed amount
    // let the user know there is no incoming data
    if ((millis() - Last) > 1000) {
      Serial.println("Searching: ");
      Last = millis();
    }

  }
}
