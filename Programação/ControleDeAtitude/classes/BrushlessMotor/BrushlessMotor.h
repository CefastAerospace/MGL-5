#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

// Constantes
#define HORARIO true
#define ANTI_HORARIO false
#define VELOCIDADE 18000

class BrushlessMotor {
    public:
        BrushlessMotor(int pwmPin, int startPin, int breakPin, int directionPin);
        void gira(bool sentido);
        void paraGiro();
        void setVelocidade(int velocidade);

    private:
        int pwmPin;
        int startPin;
        int breakPin;
        int directionPin;

        int velocidade;

        void acelera(bool estado);
        void freia(bool estado);
        void start(bool estado);
        void rotacao(bool sentido);

};

#endif