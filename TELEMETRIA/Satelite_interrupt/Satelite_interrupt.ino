#include <SPI.h>
#include <LoRa.h>

#define BUZZER_PIN 25
#define LORA_SS 26
#define LORA_RST 13
#define LORA_DIO0 14

volatile bool packetReceived = false;

unsigned long lastSendTime = 0;
const long interval = 2000; // Intervalo de 2 segundos

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");

  // Configurar interrupção no pino DIO0
  pinMode(LORA_DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onReceive, RISING);
}

void loop() {
  // Verificar se é hora de enviar um pacote
  if (millis() - lastSendTime > interval) {
    sendTelemetry();
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

      // Enviar ACK de volta
      if (received.startsWith("CMD:")) {
        sendAck();
      }
    }
  }
}

void sendTelemetry() {
  String mensagemEnviar = "Angulacao: 45"; // Exemplo de telemetria
  LoRa.beginPacket();
  LoRa.print(mensagemEnviar);
  LoRa.endPacket();
  Serial.print("Enviado: ");
  Serial.println(mensagemEnviar);
}

void sendAck() {
  LoRa.beginPacket();
  LoRa.print("ACK");
  LoRa.endPacket();
  Serial.println("ACK enviado!");
}

void onReceive() {
  packetReceived = true;
}