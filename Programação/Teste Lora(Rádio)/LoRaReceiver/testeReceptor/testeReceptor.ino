#include <SoftwareSerial.h>
#include <LoRa.h> 
#define M0 3
#define M1 4
#define Aux 5
#define RX 0
#define TX 1


SoftwareSerial mySerial(RX, TX);

void setup() {
  // Inicie a comunicação serial
  Serial.begin(9600);
  while (!Serial) {
    ; // aguarde a porta serial conectar. Necessário para placas Leonardo apenas
  }

  // Inicie a comunicação serial com o módulo LoRa E32
  mySerial.begin(9600);
  while (!mySerial) {
    ; // aguarde a porta serial conectar. Necessário para placas Leonardo apenas
  }

  // Defina os pinos M0 e M1 como OUTPUT
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  // Defina M0 e M1 para o modo normal de operação (M0=0, M1=0)
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void loop() { // Executa continuamente
  // Verifique se há dados disponíveis para leitura
  if (mySerial.available()) {
    // Leia os dados recebidos
    String data = mySerial.readString();
    
    // Imprima os dados recebidos
    Serial.println("Dados recebidos: " + data);
  }
}