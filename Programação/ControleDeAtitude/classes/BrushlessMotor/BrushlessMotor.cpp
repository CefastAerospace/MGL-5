
#include "BrushlessMotor.h"

// Construtor
BrushlessMotor::BrushlessMotor(int pwmPin, int startPin, int breakPin, int directionPin){
    this-> pwmPin = pwmPin;
    this-> startPin = startPin;
    this-> breakPin = breakPin;
    this-> directionPin = directionPin;
    this-> velocidade = 0;

    pinMode(pwmPin, OUTPUT);
    pinMode(startPin, OUTPUT);
    pinMode(breakPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
}

// Controla o terminal PWM de acordo com o argumento
void BrushlessMotor::acelera(bool estado){
    if(estado)
        tone(pwmPin, velocidade); // Emite a frequência padrão de giro
    else
        tone(pwmPin, 0);
}

// Controla o terminal BREAK de acordo com o argumento
void BrushlessMotor::freia(bool estado){
    if(estado)
        digitalWrite(breakPin, LOW); // Freia
    else
        digitalWrite(breakPin, HIGH); // Libera freio
}

// Controla o terminal DIRECTION de acordo com o sentido de rotação fornecido
void BrushlessMotor::rotacao(bool sentido){
    if(sentido)
        digitalWrite(directionPin, LOW); // Muda a direção para HORÁRIO
    else
        digitalWrite(directionPin, HIGH); // Muda a direção para ANTI-HORÁRIO
}

// Controla o terminal START de acordo com o argumento
void BrushlessMotor::start(bool estado){
    if(estado)
        digitalWrite(startPin, HIGH);
    else
        digitalWrite(startPin, LOW);
}

// Faz o eixo do motor girar no sentido e durante o tempo especificados
void BrushlessMotor::gira(bool sentido){
  rotacao(sentido);
  acelera(true); // Emite a frequência de giro
  freia(false); // Libera o freio
  start(true);  // Começa a girar
}

void BrushlessMotor::paraGiro(){
    acelera(false); // Para de emitir a frequência de giro
    start(false);  // Para de girar
    freia(true);   // Freia
}

void BrushlessMotor::setVelocidade(int velocidade){
    this->velocidade = velocidade;
}