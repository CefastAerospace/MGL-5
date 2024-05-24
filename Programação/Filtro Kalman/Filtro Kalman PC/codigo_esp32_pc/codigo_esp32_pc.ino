/*
    Escrito por Henrique Costa (https://github.com/riquetret)
*/
#include <Wire.h>

#define BIT8 0x80
#define BIT7 0x40
#define BIT6 0x20
#define BIT5 0x10
#define BIT4 0x08
#define BIT3 0x04
#define BIT2 0x02
#define BIT1 0x01

#define MPU6050_ADDR 0x69

float roll,pitch,yaw; //Roll=x,pitch=y,yaw=z, Giroscopio [º/s]
float acx,acy,acz;    //Aceleracao em cada eixo[m/s^2]
int16_t GYRO_X,GYRO_Y,GYRO_Z,ACC_X,ACC_Y,ACC_Z;//Contagens LSB do MPU6050
int16_t GYRO_X_OFF,GYRO_Y_OFF,GYRO_Z_OFF,ACC_X_OFF,ACC_Y_OFF,ACC_Z_OFF; //Offset/Erro para calibrar

/// @brief Envia uma informacao via I2C
/// @param id Id do dispositivo
/// @param registro Qual registro do dispositivo
/// @param informacao2 Primeira informacao a ser enviada (MSB)
/// @param info2 Se falso, não envia informacao2
/// @param informacao1 Segunda informacao a ser enviada (LSB)
/// @param info1 Se falso, não envia informacao1
void envia_i2c(char id,char registro,char informacao2=0,bool info2=false,char informacao1=0,bool info1=0){
    Wire.beginTransmission(id);
    Wire.write(registro);
    if(info2==true)Wire.write(informacao2);
    if(info1==true)Wire.write(informacao1);
    Wire.endTransmission();
}

/// @brief Le ou giroscopio, ou aceleracao
/// @param VarX Ponteiro para variavel X, exemplo GYRO_X
/// @param VarY Ponteiro para variavel Y
/// @param VarZ Ponteiro para variavel Z
/// @param id Id do dispositivo
/// @param registrador Registrador para início da leitura
void leitura_MPU(int16_t *VarX,int16_t *VarY,int16_t *VarZ,char id,char registrador){
    envia_i2c(id,registrador);//Informa que deseja ler a partir do registro "registrador"
    Wire.requestFrom(MPU6050_ADDR,6);//Vamos Ler 6 Bytes em que cada informação tem 16 bits, logo 3*Gyros/Acc=48bits=6Bytes

    //O deslocamento de 8 bits eh preciso pq nosso registrador são de 16bits, o numero esta em complemento de 2, mas nao eh garantido que o compilador entenda. O valor indicado são as contagens (LSB) que deve ser transformado 
    *VarX = Wire.read()<<8 | Wire.read(); 
    *VarY = Wire.read()<<8 | Wire.read();
    *VarZ = Wire.read()<<8 | Wire.read();
}


// void calibracao_MPU(int16_t *VarX,int16_t *VarY,int16_t *VarZ,
//                     int16_t *OffX,int16_t *OffY,int16_t *OffZ,
//                     char id,char registrador){
//     int16_t contagensX=0,contagensY=0,contagensZ=0;
//     for(int i=0;i<50;i++){
//         leitura_MPU(VarX,VarY,VarZ,id,registrador);
//         contagensX = contagensX + *VarX;
//         contagensY = contagensY + *VarY;
//         contagensZ = contagensZ + *VarZ;
//     }
//     *OffX = contagensX/50;
//     *OffY = contagensY/50;
//     *OffZ = contagensZ/50;
// }

void setup() {
    Serial.begin(115200); //Leitura Serial em 115200 baud rate
    Wire.setClock(100000); //Clock I2C em 100kHz
    Wire.begin(); // Configuração do I2C, Pinos GPIO22(SCL), GPIO21(SDA)
    delay(1000); // Delay para que MPU6050 funcione

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); //Registrador de gerenciamento de energia
    // É interessante usar o Cycle = 1, para manter dormindo durante cada captura, mas apenas dados de acelerometro
    Wire.write(BIT4); //Device reset = 0, Sleep = 0, Cycle = 0, Bit5 = x, Temp_Dis=1=Temperatura OFF, CLKSEL = 0 = Oscilador Interno 8 MHz
    Wire.endTransmission(); //Fim de transmissão

    //Sensibilidades e Filtros MPU6050
    envia_i2c(MPU6050_ADDR,0x1A,0x05,true); //FSYNC=OFF,DLPF_CFG = 5 = Passa-Baixa 10Hz para ACC e GYRO, Delay de 13,4ms na resposta 
    envia_i2c(MPU6050_ADDR,0x1B,BIT3,true); //FS_SEL=1=+-500º/s=65.5LSB/º/s
    envia_i2c(MPU6050_ADDR,0x1C,BIT5+BIT4,true); //AFS_SEL=3=+-16G=2048LSB/g
    //calibracao_MPU(&GYRO_X,&GYRO_Y,&GYRO_Z,&GYRO_X_OFF,&GYRO_Y_OFF,&GYRO_Z_OFF,MPU6050_ADDR,0x43); //Calibracao GYRO
    //calibracao_MPU(&ACC_X,&ACC_Y,&ACC_Z,&ACC_X_OFF,&ACC_Y_OFF,&ACC_Z_OFF,MPU6050_ADDR,0x3B); //Calibracao ACC
}
 
void loop() {
    leitura_MPU(&GYRO_X,&GYRO_Y,&GYRO_Z,MPU6050_ADDR,0x43); //Leitura GYRO
    leitura_MPU(&ACC_X,&ACC_Y,&ACC_Z,MPU6050_ADDR,0x3B); //Leitura ACC

    Serial.print("gx:");Serial.print(GYRO_X);Serial.print(" gy:");Serial.print(GYRO_Y);Serial.print(" gz:");Serial.print(GYRO_Z);
    Serial.print(" acx:");Serial.print(ACC_X);Serial.print(" acy:");Serial.print(ACC_Y);Serial.print(" acz:");Serial.println(ACC_Z);
    
    //roll = (float)(GYRO_X-GYRO_X_OFF)/65.5;
    //pitch = (float)(GYRO_Y-GYRO_Y_OFF)/65.5;
    //yaw = (float)(GYRO_Z-GYRO_Z_OFF)/65.5;

    //acx = (float)(ACC_X)/2048.0 - 0.03;
    //acy = (float)(ACC_Y)/2048.0 + 0.02;
    //acz = (float)(ACC_Z)/2048.0 - 0.02;
    //Serial.print("acx:");Serial.print(ACC_X_OFF);Serial.print(" acy:");Serial.print(ACC_Y_OFF);Serial.print(" acz:");Serial.println(ACC_Z_OFF);
    //Serial.print("acx:");Serial.print(ACC_X);Serial.print(" acy:");Serial.print(ACC_Y);Serial.print(" acz:");Serial.println(ACC_Z);
    //Serial.print("acx:");Serial.print(acx,2);Serial.print("\tacy:");Serial.print(acy,2);Serial.print("\tacz:");Serial.println(acz,2);
    //Serial.print("Roll:");Serial.print(roll,2);Serial.print("\tPitch:");Serial.print(pitch,2);Serial.print("\tYaw:");Serial.println(yaw,2);
}
