#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>


#define BUZZER_PIN 25
#define DHT11_PIN 19
#define LORA_SS 26
#define LORA_RST 13
#define LORA_DIO0 14

DHT dht11(DHT11_PIN, DHT11);
Adafruit_BMP280 bmp;  //I2C

unsigned long lastSendTime = 0;
const long interval = 5000; // Intervalo de 2 segundos


void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(BUZZER_PIN, OUTPUT);
  
  dht11.begin();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");

  if (!bmp.begin(0x76)) { //Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)/

    //Imprime mensagem de erro no caso de endereço invalido ou não localizado. Modifique o valor
    Serial.println(F(" Não foi possível encontrar um sensor BMP280 válido, verifique a fiação ou "
                     "tente outro endereço!"));
    while (1) delay(10);
  }
 }


void loop() {
  // printDTH11();
  // printBMP280()

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

// Imprime a humidade e temperatura lidos pelo sensor DTH11
void printDTH11(){
  Serial.print("Humidity: ");
  Serial.print(dht11.readHumidity());
  Serial.print("% ");
  Serial.print("\tTemperature: ");
  Serial.print(dht11.readTemperature());
  Serial.println("°C");
}

void printBMP280(){
  //Imprimindo os valores de Temperatura
  Serial.print(F("Temperatura = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  //Imprimindo os valores de Pressão.
  Serial.print(F("Pressão = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  //Imprimindo os valores de Altitude Aproximada
  Serial.print(F("Altitude Aprox = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Ajustar a pressão de nível do mar de acordo com o local!*/
  Serial.println(" m");
}
