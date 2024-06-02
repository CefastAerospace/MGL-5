#include <ReactionController.h> // Biblioteca para controle do motor
#include <neotimer.h>  // Biblioteca para controle de tempo

#define DIRECTION 2
#define BREAK 3
#define START 4
#define PWM 5
#define VELOCIDADE 18000  // Frequência de giro padrão do motor: 18KHz = 2700rpm

ReactionController controller;  // Controlador do motor com sensores internos
Neotimer timer; // Temporizador

int angulo;

void setup() {
  controller = ReactionController(PWM, START, BREAK, DIRECTION);  // Inicializa o controlador definindo os pinos do motor
  controller.setVelocidadeGiro(VELOCIDADE);  // Define a velocidade de giro do motor por meio da frequência
  controller.setToleranciaAngulo(5);  // Define a tolerância de 5 graus para o ângulo no qual o motor é considerado estabilizado
  
  angulo = 0; // Ângulo de posicionamento inicial
}

void loop() {
  controller.atualizaDados();
  if(controller.estabilizado(angulo)){
    // Se o motor estiver estabilizado na posição, começa a contar o tempo antes de incrementar o ângulo
    if(!timer.started()){
      timer.set(4000);  // Define a duração do temporizador em 4 segundos
      timer.start();  // Conta 4 segundos a partir do momento que o motor estabiliza na posição
    }
    if(timer.done()){ // Se o tempo acabou
      timer.stop();  // Para o temporizador
      angulo += 30; // Incrementa o ângulo para o próximo posicionamento
    }
  }
}
