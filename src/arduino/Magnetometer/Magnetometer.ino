#include <Tlv493d.h>

// Tlv493d Object
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
const int electromagnetPin = 13;  // Define pin for electromagnet control

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Tlv493dMagnetic3DSensor.begin();

  pinMode(electromagnetPin, OUTPUT);
  digitalWrite(electromagnetPin, LOW);  // Ensure electromagnet is initially off

}

void loop() {
  Tlv493dMagnetic3DSensor.updateData();
  delay(50);

  float x = Tlv493dMagnetic3DSensor.getX();
  float y = Tlv493dMagnetic3DSensor.getY();
  float z = Tlv493dMagnetic3DSensor.getZ();

  Serial.print("X = ");
  Serial.print(x);
  Serial.print(" mT; Y = ");
  Serial.print(y);
  Serial.print(" mT; Z = ");
  Serial.print(z);
  Serial.println(" mT");

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command == "ON") {
      digitalWrite(13, HIGH);  // Replace '13' with your electromagnet pin
      Serial.println("Turning Electromagnet On");
    } else if (command == "OFF") {
      digitalWrite(13, LOW);  // Replace '13' with your electromagnet pin
      Serial.println("Turning Electromagnet Off");
    } 
  }
  
  // Check the state of the electromagnet based on magnetic field
    if (x > -1 && x < 1 && y > -1 && y < 1 && z > -1 && z < 1) {
        Serial.println("Electromagnet off without brick");
    } else if (x > 5 && x < 10 && y > -4 && y < 4 && z > 30 && z < 38) {
        Serial.println("Electromagnet on without brick");
    } else if (x > 5 && x < 10 && y > -4 && y < 4 && z > 38) {
        Serial.println("Electromagnet on with brick");
    } else {
        Serial.println("Check magnetometer value range for electromagnets");
    }


  delay(500);
}
