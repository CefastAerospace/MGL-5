
#include "BrushlessMotor.h"

// Construtor
BrushlessMotor::BrushlessMotor(int pwmPin, int startPin, int breakPin, int directionPin) :
    velocidadeGiro(DEFAULT_VELOCIDADE), // Velocidade padrão de giro
    estado(STOPPED)
{
    anexaPinos(pwmPin, startPin, breakPin, directionPin);
}

// Construtor vazio
BrushlessMotor::BrushlessMotor() :
    pwmPin(0),
    startPin(0),
    breakPin(0),
    directionPin(0),
    velocidadeGiro(DEFAULT_VELOCIDADE), // Velocidade padrão de giro
    estado(STOPPED)
{
    anexaPinos(pwmPin, startPin, breakPin, directionPin);
}

// Informa o estado atual do motor
BrushlessMotor::EstadoMotor BrushlessMotor::getEstado() const{
    return estado;
}

// anexa os pinos ao motor
void BrushlessMotor::anexaPinos(int pwmPin, int startPin, int breakPin, int directionPin){
    this-> pwmPin = pwmPin;
    this-> startPin = startPin;
    this-> breakPin = breakPin;
    this-> directionPin = directionPin;

    pinMode(pwmPin, OUTPUT);
    pinMode(startPin, OUTPUT);
    pinMode(breakPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
}

// Controla o terminal PWM de acordo com o argumento
void BrushlessMotor::acelera(bool estado){
    if(estado)
        tone(pwmPin, velocidadeGiro); // Emite a frequência padrão de giro
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

  // Determina o estado de rotação do motor conforme a flag 'sentido'
  estado = sentido ? ROTATING_CLOCKWISE : ROTATING_COUNTER_CLOCKWISE;
}

// Para o giro do motor
void BrushlessMotor::paraGiro(){
    acelera(false); // Para de emitir a frequência de giro
    start(false);  // Para de girar
    freia(true);   // Freia
    estado = STOPPED;
}

// Altera a velocidade de giro do motor
void BrushlessMotor::setVelocidade(int velocidade){
    if(velocidade < 26000 && velocidade > 0)
        this->velocidadeGiro = velocidade;
}

