#include <SPI.h>
#include <LoRa.h>

/* Configuração dos pinos
 *  Modulo | Placa (Esp-32/barramento)
 *  NSS    |   GPIO26  
 *  MOSI   |   GPIO23(MOSI)
 *  MISO   |   GPIO19(MISO)
 *  SCK    |   GPIO18(SCK)
 *  RST    |   GPIO13 
 *  DI00   |   GPIO14
 *  DI02   |   GPIO05
 */

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(26,13,14); // Setano Pinos SS, RST e DI00

  if (!LoRa.begin(433E6)) { //Se o módulo não iniciar na frequencia 433Mhz faça:
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}
