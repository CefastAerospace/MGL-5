#include <SoftwareSerial.h>
#include <LoRa.h> 
 
SoftwareSerial loraSerial(2, 3); // TX, RX
 
void setup() {
  Serial.begin(9600);
  loraSerial.begin(9600);  
}
 
void loop() { 
  while (loraSerial.available() > 1) {
      Serial.print((char)LoRa.read());
    }
  delay(20);
}
