#include <SPI.h>
#include <LoRa.h>


/* Configuração dos pinos
 *  Modulo | Placa (Esp-32/barramento)
 *  NSS    |   GPIO2
 *  MOSI   |   GPIO23(MOSI)
 *  MISO   |   GPIO19(MISO)
 *  SCK    |   GPIO18(SCK)
 *  RST    |   GPIO13 
 *  DI00   |   GPIO14
 *  DI02   |   GPIO05
 */


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  LoRa.setPins(2,13,14); // Setano Pinos SS, RST e DI00

  if (!LoRa.begin(433E6)) { //Se o módulo não iniciar na frequencia 433Mhz faça:
    Serial.println("Starting LoRa failed!");
    while (1);
  } else{
    Serial.println("Lora Iniciado");
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
