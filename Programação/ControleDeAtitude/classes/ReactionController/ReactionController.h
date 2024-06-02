#ifndef ReactionController_H
#define ReactionController_H

#include <Arduino.h>
#include <LuxList.h>
#include <BrushlessMotor.h>
#include <MPU6050_leitura.h>

#define MPU6050_ADDR 0x69

class ReactionController {
    public:
        // Cria um alias para referenciar o enum da classe BrushlessMotor
        using EstadoMotor = BrushlessMotor::EstadoMotor;

        // Métodos públicos
        ReactionController(int pwmPin, int startPin, int breakPin, int directionPin); // Construtor
        ReactionController();  // Construtor vazio
        void setMotorPins(int pwmPin, int startPin, int breakPin, int directionPin);
        void leituraAmbiente();
        void atualizaDados();
        void setVelocidadeGiro(int velocidade);
        void setToleranciaAngulo(int tolerancia);
        void setSensibilidadeAcc(char sensibilidade);
        void setSensibilidadeGiro(int_least16_t sensibilidade);
        bool estabilizado(int angulo);

    private:
        // Atributos privados
        MPU6050_leitura sensor;
        BrushlessMotor motor;
        LuxList luxList;
        LuxItem luxAtual;
        int toleranciaAngulo;
        
        // Métodos privados
        int corrigeAngulo(int angulo);
    
};

#endif