#ifndef MPU6050_LEITURA_H
#define MPU6050_LEITURA_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050_leitura {
public:
    enum etapa {
        coleta_e_calculo = 0,
        deslocamento = 1,
        invalido = 2,
    };
    char endereco_mpu;
    int16_t GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z; // Contagens LSB do MPU6050
    int16_t GYRO_X_OFF, GYRO_Y_OFF, GYRO_Z_OFF; // Offset/Erro para calibrar
    int16_t ACC_X_OFF, ACC_Y_OFF, ACC_Z_OFF; // Offset/Erro para calibrar
    double gx, gy, gz; // deg/s em cada eixo
    double acx, acy, acz; // Aceleracao em cada eixo[m/s^2]
    double sensi_acc, sensi_giro; // Sensibilidades do Acelerômetro


    MPU6050_leitura(int16_t acx_off, int16_t acy_off, int16_t acz_off, char endereco);
    void atualiza_leitura();
    void set_sensi_acc(char acc_sensi);
    void set_sensi_giro(int_least16_t giro_sensi);
    void inicializa(char acc_sensi, int_least16_t giro_sensi);

    double getRotationX();
    double getRotationY();
    double getRotationZ();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    int getIntYaw();

    // Ler valores do acelerômetro e giroscópio 
    void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
    static float deg2rad(float degrees);
    static float rad2deg(float radians);
    
private:
    double tempos[2];
    double amostras[2];
    double acumulador_yaw;

    void prepara_wire();
    void set_config_energia();
    void set_filters();
    static void envia_i2c(char id, char registro, char informacao2 = 0, bool info2 = false, char informacao1 = 0, bool info1 = 0);
    static void leitura_MPU_bruta(int16_t* VarX, int16_t* VarY, int16_t* VarZ, char id, char registrador);
    static void calibracao_MPU(int16_t* VarX, int16_t* VarY, int16_t* VarZ, int16_t* OffX, int16_t* OffY, int16_t* OffZ, char id, char registrador, int iteracoes);
    void leitura_MPU();

};

#endif // MPU6050_LEITURA_H
