#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Cria uma instância da biblioteca TinyGPS++
TinyGPSPlus gps;

// Configura o hardware serial
HardwareSerial mySerial(1);

// Configuração dos pinos RX e TX
const int RX_PIN = 3;
const int TX_PIN = 1;

void setup() {
  // Inicializa a comunicação serial com o computador
  Serial.begin(115200);
  
  // Inicializa a comunicação serial com o módulo GPS
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Mensagem de boas-vindas
  Serial.println("GPS Test");
}

void loop() {
  // Verifica se há novos dados disponíveis no módulo GPS
  while (mySerial.available() > 0) {
    // Lê os dados do GPS e passa para a biblioteca TinyGPS++
    gps.encode(mySerial.read());
    
    // Se há uma nova localização disponível
    if (gps.location.isUpdated()) {
      // Imprime a latitude e longitude no monitor serial
      Serial.print("Latitude= "); Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); Serial.println(gps.location.lng(), 6);
    }
  }
}
