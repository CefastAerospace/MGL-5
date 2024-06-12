#include <SPI.h>
#include <LoRa.h>

#define LORA_SS 15
#define LORA_RST 16
#define LORA_DIO0 4

unsigned long lastSendTime = 0;
const long interval = 5000; // Intervalo de 5 segundos

void IRAM_ATTR interrupcao(int packetSize){
  Serial.println("ENTROU interrupcao1");
  if (packetSize == 0) return;
  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }
  Serial.print("Recebido: ");
  Serial.println(received);
}

// // Verificar se há pacotes recebidos
//   Serial.println("Interrupt");
//   // int packetSize = LoRa.parsePacket();
//   if (packetSize) {
//     String received = "";
//     while (LoRa.available()) {
//       received += (char)LoRa.read();
//     }
//     Serial.print("Recebido: ");
//     Serial.println(received);
//   }

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  LoRa.onReceive(interrupcao);
  LoRa.receive();
}

void loop() {
  // Verificar se é hora de enviar um comando
  if (millis() - lastSendTime > interval) {
    sendCommand();
    lastSendTime = millis();
    LoRa.receive();
  }
}

void sendCommand() {
  String mensagemEnviar = "CMD: Ajuste Angulacao"; // Exemplo de comando
  LoRa.beginPacket();
  LoRa.print(mensagemEnviar);
  LoRa.endPacket();
  Serial.print("Enviado: ");
  Serial.println(mensagemEnviar);
}
