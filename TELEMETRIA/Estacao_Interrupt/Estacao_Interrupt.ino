#include <SPI.h>
#include <LoRa.h>


#define LORA_RST 9
#define LORA_DIO0 16

volatile bool packetReceived = false;

unsigned long lastSendTime = 0;
const long interval = 5000; // Intervalo de 5 segundos

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");

  // Configurar interrupção no pino DIO0
  pinMode(LORA_DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onReceive, RISING);
}

void loop() {
  // Verificar se é hora de enviar um comando
  if (millis() - lastSendTime > interval) {
    sendCommand();
    lastSendTime = millis();
  }

  if (packetReceived) {
    packetReceived = false;

    // Processar o pacote recebido
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String received = "";
      while (LoRa.available()) {
        received += (char)LoRa.read();
      }
      Serial.print("Recebido: ");
      Serial.println(received);
    }
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

void onReceive() {
  packetReceived = true;
}