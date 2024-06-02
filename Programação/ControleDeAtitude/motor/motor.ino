#include <ReactionController.h>

#define DIRECTION 2
#define BREAK 3
#define START 4
#define PWM 5
#define VELOCIDADE 18000  // Frequência de giro padrão do motor: 18KHz = 2700rpm

ReactionController controller;
int angulo;

void setup() {
  controller = ReactionController(PWM, START, BREAK, DIRECTION);
  controller.setVelocidadeGiro(VELOCIDADE);
  controller.setToleranciaAngulo(5);
  
  angulo = 0; // Ângulo de posicionamento inicial
}

void loop() {
  controller.atualizaDados();
  if(controller.estabilizado(angulo)){
    delay(5000);  // Aguarda 5 segundos após ter estabilizado o satélite
    angulo += 30; // Incrementa o ângulo para o próximo posicionamento
  }
}
