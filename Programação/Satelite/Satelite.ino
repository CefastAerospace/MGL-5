#include <SPI.h>
#include <LoRa.h>

#define BUZZER_PIN 25
#define LORA_SS 26
#define LORA_RST 13
#define LORA_DIO0 14


unsigned long lastSendTime = 0;
const long interval = 5000; // Intervalo de 2 segundos


void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(BUZZER_PIN, OUTPUT);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // Verificar se é hora de enviar um pacote
  if (millis() - lastSendTime > interval) {
    sendTelemetry();
    lastSendTime = millis();
    Serial.println(millis()/1000);
  }

  // Verificar se há pacotes recebidos
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

    if (received == "CMD: Pisca Luz"){
      Serial.println("Piscou");
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
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
