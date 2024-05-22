#include <LinkedList.h>

#include <classes.h>

#define DIRECTION 2
#define BREAK 3
#define START 4
#define PWM 5
#define VELOCIDADE 18000  // Frequência de giro padrão do motor: 18KHz = 2700rpm

BrushlessMotor *motor;

void setup() {
  motor = new BrushlessMotor(PWM, START, BREAK, DIRECTION);
  //motor->setVelocidade(VELOCIDADE);
}

void loop() {
  // motor.gira(HORARIO);
  // delay(5000);
  // motor.paraGiro();
  // delay(5000);
  // motor.gira(ANTI_HORARIO);
  // delay(5000);
  // motor.paraGiro();
  // delay(5000);
}
