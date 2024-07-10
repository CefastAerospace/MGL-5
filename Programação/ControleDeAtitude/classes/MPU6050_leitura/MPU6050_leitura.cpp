#include "MPU6050_leitura.h"

#define BIT8 0x80
#define BIT7 0x40
#define BIT6 0x20
#define BIT5 0x10
#define BIT4 0x08
#define BIT3 0x04
#define BIT2 0x02
#define BIT1 0x01
#define GRAVIDADE 9.80665

// Construtor
MPU6050_leitura::MPU6050_leitura(int16_t acx_off, int16_t acy_off, int16_t acz_off, char endereco) :
    ACC_X_OFF(acx_off),
    ACC_Y_OFF(acy_off),         // Inicialização das variáveis
    ACC_Z_OFF(acz_off),
    gx(0), gy(0), gz(0),
    acx(0), acy(0), acz(0),
    tempos{0, 0}, amostras{0, 0},
    acumulador_yaw(0),
    compass(0),
    endereco_mpu(endereco) 
{
    
//     // Métodos de inicialização
//     prepara_wire();
//     set_config_energia();
//     set_sensi_giro(giro_sensi);
//     set_sensi_acc(acc_sensi);
//     set_filters();
//     calibracao_MPU(&GYRO_X, &GYRO_Y, &GYRO_Z, &GYRO_X_OFF, &GYRO_Y_OFF, &GYRO_Z_OFF, endereco_mpu, 0x43, 100); // Calibracao GYRO
 }

void MPU6050_leitura::inicializa(char acc_sensi, int_least16_t giro_sensi){
    // Métodos de inicialização
    prepara_wire();
  
    set_config_energia();

    set_sensi_giro(giro_sensi);
    
    set_sensi_acc(acc_sensi);

    set_filters();
    delay(2000);
    calibracao_MPU(&GYRO_X, &GYRO_Y, &GYRO_Z, &GYRO_X_OFF, &GYRO_Y_OFF, &GYRO_Z_OFF, endereco_mpu, 0x43, 100); // Calibracao GYRO
   
}

// Prepara o barramento I2C
void MPU6050_leitura::prepara_wire(){
    Serial.begin(115200);
    Wire.setClock(100000); // Clock I2C em 100kHz
    Wire.begin(); // Configuração do I2C, Pinos GPIO22(SCL), GPIO21(SDA)
    delay(1000); // Delay para que MPU6050 funcione
}

// Configura o registrador de gerenciamento de energia
void MPU6050_leitura::set_config_energia(){
    envia_i2c(endereco_mpu, 0x6B, BIT4, true); // Device reset = 0, Sleep = 0, Cycle = 0, Bit5 = x, Temp_Dis=1=Temperatura OFF, CLKSEL = 0 = Oscilador Interno 8 MHz
}

// Configura o filtro passa-baixa digital e FSYNC
void MPU6050_leitura::set_filters(){
    // Filtros MPU6050
    envia_i2c(endereco_mpu, 0x1A, 0x05, true); // FSYNC=OFF,DLPF_CFG = 5 = Passa-Baixa 10Hz para ACC e GYRO, Delay de 13,4ms na resposta
}

// Configura a sensibilidade do giroscópio
void MPU6050_leitura::set_sensi_giro(int_least16_t giro_sensi){
    switch (giro_sensi) {
        case 250:
            envia_i2c(endereco_mpu, 0x1B, 0, true); // FS_SEL=0=+-250º/s=131LSB/º/s
            sensi_giro = 131;
            break;
        case 500:
            envia_i2c(endereco_mpu, 0x1B, BIT4, true); // FS_SEL=1=+-500º/s=65.5LSB/º/s
            sensi_giro = 65.5;
            break;
        case 1000:
            envia_i2c(endereco_mpu, 0x1B, BIT5, true); // FS_SEL=2=+-1000º/s=32.8LSB/º/s
            sensi_giro = 32.8;
            break;
        default:
            envia_i2c(endereco_mpu, 0x1B, BIT4 + BIT5, true); // FS_SEL=3=+-2000º/s=16.4LSB/º/s
            sensi_giro = 16.4;
            break;
    }
}

// Configura a sensibilidade do acelerômetro
void MPU6050_leitura::set_sensi_acc(char acc_sensi){
    switch (acc_sensi) {
        case 2:
            envia_i2c(endereco_mpu, 0x1C, 0, true); // AFS_SEL=0=+-2G=16384LSB/g
            sensi_acc = 16384;
            break;
        case 4:
            envia_i2c(endereco_mpu, 0x1C, BIT4, true); // AFS_SEL=1=+-4G=8192LSB/g
            sensi_acc = 8192;
            break;
        case 8:
            envia_i2c(endereco_mpu, 0x1C, BIT5, true); // AFS_SEL=2=+-8G=4096LSB/g
            sensi_acc = 4096;
            break;
        default:
            envia_i2c(endereco_mpu, 0x1C, BIT4 + BIT5, true); // AFS_SEL=3=+-16G=2048LSB/g
            sensi_acc = 2048;
            break;
    }
}

// Retorna o valor do giroscópio no eixo X
double MPU6050_leitura::getRotationX(){
    return this->gx;
}

// Retorna o valor do giroscópio no eixo Y
double MPU6050_leitura::getRotationY(){
    return this->gy;
}

// Retorna o valor do giroscópio no eixo Z
double MPU6050_leitura::getRotationZ(){
    return this->gz;
}

// Retorna o valor do acelerômetro no eixo X
double MPU6050_leitura::getAccelerationX(){
    return this->acx;
}

// Retorna o valor do acelerômetro no eixo Y
double MPU6050_leitura::getAccelerationY(){
    return this->acy;
}

// Retorna o valor do acelerômetro no eixo Z
double MPU6050_leitura::getAccelerationZ(){
    return this->acz;
}

// Retorna o valor inteiro do Yaw (rotação em torno do eixo Z em graus)
int MPU6050_leitura::getIntYaw(){
    return static_cast<int>(round(this->acumulador_yaw));;
}

// Ler todos os valores do acelerômetro e giroscópio
void MPU6050_leitura::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz){
    *ax = this->acx;
    *ay = this->acy;
    *az = this->acz;
    *gx = this->gx;
    *gy = this->gy;
    *gz = this->gz;
}

// Envia informações via I2C
void MPU6050_leitura::envia_i2c(char id, char registro, char informacao2, bool info2, char informacao1, bool info1) {
    Wire.beginTransmission(id);
    Wire.write(registro);
    if (info2 == true) Wire.write(informacao2);
    if (info1 == true) Wire.write(informacao1);
    Wire.endTransmission();
}

// Lê informações via I2C e armazena nas variáveis
void MPU6050_leitura::leitura_MPU_bruta(int16_t* VarX, int16_t* VarY, int16_t* VarZ, char id, char registrador) {
    envia_i2c(id, registrador); // Informa que deseja ler a partir do registro "registrador"
    Wire.requestFrom(id, 6); // Vamos Ler 6 Bytes em que cada informação tem 16 bits, logo 3*Gyros/Acc=48bits=6Bytes

    // O deslocamento de 8 bits eh preciso pq nosso registrador são de 16bits, o numero esta em complemento de 2, mas nao eh garantido que o compilador entenda. O valor indicado são as contagens (LSB) que deve ser transformado 
    *VarX = Wire.read() << 8 | Wire.read();
    *VarY = Wire.read() << 8 | Wire.read();
    *VarZ = Wire.read() << 8 | Wire.read();
}

// Lê os valores brutos do acelerômetro e giroscópio e converte para valores reais
void MPU6050_leitura::leitura_MPU() {
    leitura_MPU_bruta(&GYRO_X, &GYRO_Y, &GYRO_Z, endereco_mpu, 0x43); // Leitura GYRO
    leitura_MPU_bruta(&ACC_X, &ACC_Y, &ACC_Z, endereco_mpu, 0x3B); // Leitura ACC

    // gx = deg2rad((GYRO_X - GYRO_X_OFF) / sensi_giro); // Valores em radianos por segundo
    // gy = deg2rad((GYRO_Y - GYRO_Y_OFF) / sensi_giro);
    // gz = deg2rad((GYRO_Z - GYRO_Z_OFF) / sensi_giro);

    gx = deg2rad((GYRO_X - GYRO_X_OFF) / sensi_giro); // Valores em graus por segundo
    gy = deg2rad((GYRO_Y - GYRO_Y_OFF) / sensi_giro);
    gz = deg2rad((GYRO_Z - GYRO_Z_OFF) / sensi_giro);


    acx = static_cast<double>(ACC_X - ACC_X_OFF) / sensi_acc * GRAVIDADE; // Valores em m/s^2
    acy = static_cast<double>(ACC_Y - ACC_Y_OFF) / sensi_acc * GRAVIDADE;
    acz = static_cast<double>(ACC_Z - ACC_Z_OFF) / sensi_acc * GRAVIDADE;
}

// Atualiza a leitura do sensor integrando o giro no eixo Z
void MPU6050_leitura::atualiza_leitura() {
    for (etapa etapa_atual = coleta_e_calculo; etapa_atual < invalido; etapa_atual = static_cast<etapa>(etapa_atual + 1)) {
        if (etapa_atual == coleta_e_calculo) {
            leitura_MPU();
            tempos[1] = micros() * pow(10, -6);
            amostras[1] = gz;
            Serial.print("Yaw Acumulado=");
            Serial.println(rad2deg(compass));
                
            // Integração do acumulador yaw e restrição de seu valor para (-360, 360)
            acumulador_yaw = fmod((acumulador_yaw + amostras[0] * (tempos[1] - tempos[0])), 2*M_PI);
            compass = acumulador_yaw > 0 ? 2*M_PI - acumulador_yaw : acumulador_yaw * (-1);
        } else {
            tempos[0] = tempos[1];
            amostras[0] = amostras[1];
        }
    }
}

// Calibra o sensor
void MPU6050_leitura::calibracao_MPU(int16_t* VarX, int16_t* VarY, int16_t* VarZ, int16_t* OffX, int16_t* OffY, int16_t* OffZ, char id, char registrador, int iteracoes) {
    long int contagensX = 0, contagensY = 0, contagensZ = 0;
    for (int i = 0; i < iteracoes; i++) {
        leitura_MPU_bruta(VarX, VarY, VarZ, id, registrador);
        contagensX = contagensX + *VarX;
        contagensY = contagensY + *VarY;
        contagensZ = contagensZ + *VarZ;
    }
    *OffX = contagensX / iteracoes;
    *OffY = contagensY / iteracoes;
    *OffZ = contagensZ / iteracoes;
}

// Converte graus para radianos
float MPU6050_leitura::deg2rad(float degrees) {
    return degrees * M_PI / 180.0;
}

// Converte radianos para graus
float MPU6050_leitura::rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}
