/*
    Escrito por Henrique Costa (https://github.com/riquetret)
*/

#ifndef MPU_KALMANFILTER_H
#define MPU_KALMANFILTER_H

#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <cmath>

using namespace BLA;

#define BIT8 0x80
#define BIT7 0x40
#define BIT6 0x20
#define BIT5 0x10
#define BIT4 0x08
#define BIT3 0x04
#define BIT2 0x02
#define BIT1 0x01

#define MPU6050_ADDR 0x69
#define GRAVIDADE 9.80665
#define SENSI_GIRO 65.5
#define SENSI_ACC 2048.0


class MPU_KalmanFilter {
    public:
    // ============== Métodos públicos do Filtro de Kalman ==============

        MPU_KalmanFilter(float Q_angle, float Q_gyro, float R_angle);
        void prepara_dados(float* ptr); // Usada no setup
        void get_roll_pitch(float* ptr); // Usada no loop

    private:
        // ============== Variáveis do Filtro de Kalman ==============

        //Gravidade UFMG = 9,7838163 m/s2, como sensibilidade = 2048LSB/g, sendo 1g = 9,80665m/s2, temos que Gravidade UFMG = 2043,231458LSB
        //Calibrando o offset abaixo para o sensor do Henrique 
        int16_t ACC_X_OFF,ACC_Y_OFF,ACC_Z_OFF; //Offset/Erro para calibrar
        int16_t GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z;    //Contagens LSB do MPU6050
        int16_t GYRO_X_OFF, GYRO_Y_OFF, GYRO_Z_OFF; //Offset/Erro para calibrar

        unsigned long tempo_anterior, tempo_atual, t_amostra;
        char iter_salva;    //Iterador para iterar sobre as matrizes
        bool etapa_inicial;

        BLA::Matrix<6, 30> dados_sensor;
        BLA::Matrix<4, 1> previsao_final_resultado;
        BLA::Matrix<4, 4> covar_anterior;
        BLA::Matrix<4, 1> dados_anteriores;
        BLA::Matrix<4, 1> previsao_inicial;
        BLA::Matrix<4, 4> matriz_R;
        BLA::Matrix<4, 4> ganho_kalman;
        BLA::Matrix<4, 4> matriz_F;

        // ============== Funções genéricas ==============

        float deg2rad(float degrees);
        float rad2deg(float radians);
        void desvio_padrao(float desvio[6]);
        char iter_dados();
        char acessa_iter_anterior();

        // =========== Funções Formatação Kalman ===========
        
        void formata_covar_inicial();
        float roll_pelo_acc(char coluna);
        float pitch_pelo_acc(char coluna);
        void formata_dados_entrada(char coluna);
        void novo_estado_dados_previsto();
        void novo_estado_covariancia_prevista();
        void formata_matriz_R();
        void calculo_ganho_kalman();
        void calculo_previsao_final_resultado();
        void nova_matriz_covariancia_anterior();

        // ========== Funções associadas à leitura e formatação inicial ===========

        void envia_i2c(char id, char registro, char informacao2 = 0, bool info2 = false, char informacao1 = 0, bool info1 = false);
        void leitura_MPU(int16_t *VarX, int16_t *VarY, int16_t *VarZ, char id, char registrador);
        unsigned long formata_leitura();
        void calibracao_MPU(int16_t *VarX, int16_t *VarY, int16_t *VarZ, int16_t *OffX, int16_t *OffY, int16_t *OffZ, char id, char registrador, int iteracoes);
};


#endif