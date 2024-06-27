#include "ReactionController.h"

// Construtor
ReactionController::ReactionController(int pwmPin, int startPin, int breakPin, int directionPin) :
    // Lista de inicialização dos membros
    sensor(73, -55, 44, MPU6050_ADDR), // Tres numeros sao os offsets do acc, depois o endereco do Mpu, Sensibilidade Acc=16G,Sensibilidade Giro=500,
    motor(pwmPin, startPin, breakPin, directionPin),
    //luxList(),
    //luxAtual(),
    toleranciaAngulo(5) // Tolerância padrão de 5 graus
{}

// Construtor vazio
// ReactionController::ReactionController() :
//     sensor(73, -55, 44, MPU6050_ADDR, 16, 500), // Tres numeros sao os offsets do acc, depois o endereco do Mpu, Sensibilidade Acc=16G,Sensibilidade Giro=500,
//     motor(),
//     luxList(),
//     luxAtual(),
//     toleranciaAngulo(5) // Tolerância padrão de 5 graus
// {}
void ReactionController::inicializaSensor(char acc_sensi, int_least16_t giro_sensi){
    sensor.inicializa(acc_sensi, giro_sensi);
   
}
// Configura a sensibilidade do acelerômetro
void ReactionController::setSensibilidadeAcc(char sensibilidade){
    sensor.set_sensi_acc(sensibilidade);
}

// Configura a sensibilidade do giroscópio
void ReactionController::setSensibilidadeGiro(int_least16_t sensibilidade){
    sensor.set_sensi_giro(sensibilidade);
}

// Anexa os pinos ao motor
void ReactionController::setMotorPins(int pwmPin, int startPin, int breakPin, int directionPin){
    motor.anexaPinos(pwmPin, startPin, breakPin, directionPin);
}

// Define a velocidade de giro
void ReactionController::setVelocidadeGiro(int velocidade){
    motor.setVelocidade(velocidade);
}

// Define a tolerância de ângulo
void ReactionController::setToleranciaAngulo(int tolerancia){
    if(tolerancia < 0){
        this->toleranciaAngulo = 0; return;
    }
    else if(tolerancia > 90){
        this->toleranciaAngulo = 90; return;
    }
    toleranciaAngulo = tolerancia;
}

// Realiza a leitura dos 360 lux's do ambiente
void ReactionController::leituraAmbiente(){
    // estabilizado(0);
    // while((luxAtual.angulo + toleranciaAngulo) < 360){
    //     estabilizado(luxAtual.angulo + toleranciaAngulo);   // Gira para o próximo ângulo
    //     luxList.addLuxItem(luxAtual);  // Adiciona a leitura atual à lista
    //}
}

// Lê e atualiza o lux do ângulo atual
void ReactionController::atualizaDados(){

    // TODO: Implementar com o sensor MPU6050 e um fotosensor
    // OBS: LuxAtual.angulo deve estar restrito a [0, 359]
    // O sentido ANTI_HORÁRIO é o de aumento do ângulo
    // Reduzir a velocidade de giro se o intervalo de leitura for muito longo 

    sensor.atualiza_leitura();

    // luxAtual.iluminancia = fotoSensor.getIluminancia();
}

// Verifica se o satélite está posicionado no ângulo fornecido
bool ReactionController::estabilizado(int angulo){
    int anguloAtual = sensor.getIntYaw(); // Lê o ângulo atual da rotação no eixo Z (Yaw) do sensor

    // Corrige os ângulos
    anguloAtual = corrigeAngulo(anguloAtual);
    angulo = corrigeAngulo(angulo);

    int diferenca = anguloAtual - angulo;

    /* Interrompe o giro do motor se a diferença estiver dentro da tolerância, 
       e se o motor ainda não estiver parado */
    if(abs(diferenca) < toleranciaAngulo){
        if(motor.getEstado() != EstadoMotor::STOPPED){
            motor.paraGiro();
            return true;
        }  

    /* Se a diferença for NEGATIVA, gira o motor no sentido anti horário 
        caso o motor ainda não esteja girando neste sentido */
    } else if(diferenca < 0 && diferenca > -180){
        if(motor.getEstado() != EstadoMotor::ROTATING_COUNTER_CLOCKWISE){
            motor.gira(ANTI_HORARIO);   // O ângulo atual deve aumentar no sentido ANTI-HORÁRIO
        }

    /* Se a diferença for POSITIVA, gira o motor no sentido horário 
        caso o motor ainda não esteja girando neste sentido */
    } else if (motor.getEstado() != EstadoMotor::ROTATING_CLOCKWISE){ 
        motor.gira(HORARIO); // O ângulo atual deve diminuir no sentido HORÁRIO
    }
    return false;
}

// Corrige o ângulo e o restringe ao intervalo [0, 359]
int ReactionController::corrigeAngulo(int angulo){
    angulo = angulo % 360; 
    return angulo < 0 ? angulo + 360 : angulo; // Corrige o ângulo negativo
}