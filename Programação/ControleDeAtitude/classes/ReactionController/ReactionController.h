#ifndef ReactionController_H
#define ReactionController_H

#include <Arduino.h>
#include <LuxList.h>
#include <BrushlessMotor.h>
#include <MPU6050_leitura.h>

#define MPU6050_ADDR 0x69

class ReactionController {
    public:
        // Cria um alias para o enum dentro de BrushlessMotor
        using EstadoMotor = BrushlessMotor::EstadoMotor;

        ReactionController(int pwmPin, int startPin, int breakPin, int directionPin);
        ReactionController();
        void setMotorPins(int pwmPin, int startPin, int breakPin, int directionPin);
        void leituraAmbiente();
        void atualizaDados();
        void setVelocidadeGiro(int velocidade);
        void setToleranciaAngulo(int tolerancia);
        void setSensibilidadeAcc(char sensibilidade);
        void setSensibilidadeGiro(int_least16_t sensibilidade);
        bool estabilizado(int angulo);

    private:
        MPU6050_leitura sensor;
        BrushlessMotor motor;
        LuxList luxList;
        LuxItem luxAtual;
        int toleranciaAngulo;
        
        int corrigeAngulo(int angulo);
    
};

#endif