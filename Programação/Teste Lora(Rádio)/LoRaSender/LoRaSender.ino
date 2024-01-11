#include <SPI.h>
#include <LoRa.h>

/* Configuração dos pinos
 *  Modulo | Placa (Esp-32/barramento) | Placa Node MCU (Esp8266) | Arduino Uno
 *  NSS    |   GPIO26                  |  GPIO05                  | D2
 *  MOSI   |   GPIO23(MOSI)            |  GPIO08(MOSI)            | 
 *  MISO   |   GPIO19(MISO)            |  GPIO07(MISO)            | 
 *  SCK    |   GPIO18(SCK)             |  GPIO06(SCK)             |
 *  RST    |   GPIO13                  |  GPIO04                  | D3
 *  DI00   |   GPIO14                  |  GPIO2                   | D4
 *  DI02   |   GPIO05                  |  GPIO0                   |
 */

int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(2,3,4); // Setano Pinos SS, RST e DI00

 if (!LoRa.begin(433E6)) { //Se o módulo não iniciar na frequencia 433Mhz faça:
    Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    Serial.println("Lora Iniciado");
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
