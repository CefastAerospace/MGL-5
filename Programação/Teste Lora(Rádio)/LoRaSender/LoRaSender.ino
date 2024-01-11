#include <SPI.h>
#include <LoRa.h>

/* Configuração dos pinos
 *  Modulo | Placa (Esp-32/barramento) | placa Node MCU
 *  NSS    |   GPIO26                  |  GPIO05
 *  MOSI   |   GPIO23(MOSI)            |  GPIO08(MOSI)
 *  MISO   |   GPIO19(MISO)            |  GPIO07(MISO)
 *  SCK    |   GPIO18(SCK)             |  GPIO06(SCK)
 *  RST    |   GPIO13                  |  GPIO04
 *  DI00   |   GPIO14                  |  GPIO2
 *  DI02   |   GPIO05                  |  GPIO0
 */

int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(D1,D2,D4); // Setano Pinos SS, RST e DI00

  int attempts = 0;
  while (!LoRa.begin(433E6) && attempts < 10) { // Tente iniciar o LoRa até 10 vezes
    Serial.println("Starting LoRa failed! Trying again...");
    delay(10000); // Espere um segundo antes de tentar novamente
    attempts++;
  }

  if (attempts == 10) {
    Serial.println("Failed to start LoRa after 10 attempts. Please check your connections and try again.");
  } else {
    Serial.println("LoRa started successfully!");
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
