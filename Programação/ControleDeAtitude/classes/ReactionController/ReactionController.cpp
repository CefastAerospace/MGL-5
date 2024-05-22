#include "ReactionController.h"

// Construtor
ReactionController::ReactionController(BrushlessMotor motor) {
    this->motor = motor;
    this->luxList = LuxList();
}

// Método para leitura do ambiente
void ReactionController::leituraAmbiente(){
    estabiliza(0);
    while(LuxAtual.angulo < 360){
        estabiliza(luxAtual.angulo + 10);   // Gira 10 graus no sentido horário
        luxList.add(luxAtual);  // Adiciona a leitura atual à lista
    }
}

// Método para ler e atualizar o lux do ângulo atual
void ReactionController::atualizaLuxAtual(){

    // TODO: Implementar com o sensor MPU6050 e fotoresistor
    // OBS: LuxAtual.angulo deve estar restrito a [0, 360]
    // O ângulo deve aumentar no sentido HORÁRIO
    // Reduzir a velocidade de giro se o intervalo de leitura for muito longo 

    // luxAtual.angulo = sensor.getAngulo();
    // luxAtual.iluminancia = sensor.getIluminancia();
    // luxAtual.angulo = corrigeAngulo(luxAtual.angulo);

}

// Método para estabilizar o ângulo
void ReactionController::estabiliza(int angulo){
    // Corrige o ângulo fornecido
    angulo = angulo % 360;
    int diferenca;
    do{
        atualizaLuxAtual ();
        diferenca = luxAtual.angulo - angulo;
        if(diferenca < 0){
            motor.gira(HORARIO);
        } else {
            motor.gira(ANTI_HORARIO);
        }
    } while(abs(diferenca) > 5); // 5 graus de tolerância
    motor.paraGiro();
}

// Método para corrigir o ângulo e restringi-lo ao intervalo [0, 360]
int ReactionController::corrigeAngulo(int angulo){
    angulo = angulo % 360; 
    return angulo < 0 ? angulo + 360 : angulo; // Corrige o ângulo negativo
}