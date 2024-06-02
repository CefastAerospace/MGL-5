#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

#include <Arduino.h>

// Constantes
#define HORARIO true
#define ANTI_HORARIO false
#define DEFAULT_VELOCIDADE 18000


class BrushlessMotor {
    public:
        // Define o estado do motor
        enum EstadoMotor{
            STOPPED,
            ROTATING_CLOCKWISE,
            ROTATING_COUNTER_CLOCKWISE
        };

        BrushlessMotor(int pwmPin, int startPin, int breakPin, int directionPin);
        BrushlessMotor();
        void anexaPinos(int pwmPin, int startPin, int breakPin, int directionPin);
        void gira(bool sentido);
        void paraGiro();
        void setVelocidade(int velocidade);
        EstadoMotor getEstado() const;

    private:
        int pwmPin;
        int startPin;
        int breakPin;
        int directionPin;
        int velocidadeGiro;

        EstadoMotor estado;

        void acelera(bool estado);
        void freia(bool estado);
        void start(bool estado);
        void rotacao(bool sentido);

};

#endif