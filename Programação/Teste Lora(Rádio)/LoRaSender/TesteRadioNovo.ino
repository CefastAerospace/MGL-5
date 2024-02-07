#include <SoftwareSerial.h>

SoftwareSerial loraSerial(2, 3); // TX, RX pinos digitais do lora e32
int counter = 0;
int testinho = 0;

void setup() {
  Serial.begin(9600);
  loraSerial.begin(9600); // velocidade da porta serial do mdulo configurada antes
}

void loop() {
   loraSerial.println("Hello RX");

  if(testinho%10 == 0)
  {
  Serial.println(" // teste+2 ");
  Serial.println(testinho);
  }
  
  counter++;
  testinho+=2;
  delay(5000);
}
