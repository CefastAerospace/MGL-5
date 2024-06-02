
#include "MPU_KalmanFilter.h"


// Construtor
MPU_KalmanFilter::MPU_KalmanFilter(){
    iter_salva = 0;
    etapa_inicial = true;
    ACC_X_OFF = 73,ACC_Y_OFF = -55,ACC_Z_OFF = 44; //Offset/Erro para calibrar
}


// ============== Funções genéricas ==============

float MPU_KalmanFilter::deg2rad(float degrees) {
    return degrees * M_PI / 180.0;
}

float MPU_KalmanFilter::rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}

void MPU_KalmanFilter::desvio_padrao(float desvio[6]) {
    float media[6],acumulador;
    for (char i = 0; i < 6; i++)
    {
        acumulador=0;
        for (char j = 0; j < 30; j++)
        {
            acumulador = acumulador + dados_sensor(i,j);
        }
        media[i] = acumulador/30.0;
    }
    for (char i = 0; i < 6; i++)
    {
        acumulador=0;
        for (char j = 0; j < 30; j++)
        {
            acumulador =  pow(dados_sensor(i,j) - media[i],2) + acumulador;
        }
        desvio[i] = sqrt(acumulador/30); //Esse desvio vai levar em conta os dados iniciais em zero, mas esse erro vai sumir apos 30 capturas de dados
    }
}


char MPU_KalmanFilter::iter_dados()
{
    if (iter_salva == 30)iter_salva = 0;
    else iter_salva = iter_salva+1;
    if(iter_salva == 0) return 29;
    return iter_salva - 1;
}

char MPU_KalmanFilter::acessa_iter_anterior(){
    if (iter_salva-1<0) return 29;
    else return iter_salva-1;
}


// ============== Funções Formatação Kalman ==============

void MPU_KalmanFilter::formata_covar_inicial()
{
    float A_ccx = dados_sensor(3,0);
    float A_ccy = dados_sensor(4,0);
    float A_ccz = dados_sensor(5,0);

    float droll = 0.031446603773522 * sqrt(
    (pow(A_ccx, 2) * pow(A_ccy, 2) + 0.280898876404494 * pow((pow(A_ccx, 2) + pow(A_ccz, 2)), 2)) /
    ((pow(A_ccx, 2) + pow(A_ccz, 2)) * pow((pow(A_ccx, 2) + pow(A_ccy, 2) + pow(A_ccz, 2)), 2)));

    
    float dpitch = 0.031446603773522 * sqrt(
    (0.280898876404494 * pow(A_ccx, 2) * pow(A_ccy, 2) + pow((pow(A_ccy, 2) + pow(A_ccz, 2)), 2)) /
    ((pow(A_ccy, 2) + pow(A_ccz, 2)) * pow((pow(A_ccx, 2) + pow(A_ccy, 2) + pow(A_ccz, 2)), 2)));

    float dgiro = deg2rad(20/3);
    covar_anterior.Fill(0);
    covar_anterior(0,0) = droll*droll;covar_anterior(1,1) = dpitch*dpitch;
    covar_anterior(2,2) = dgiro*dgiro;covar_anterior(3,3) = covar_anterior(2,2);
}

float MPU_KalmanFilter::roll_pelo_acc(char coluna)
{
    float A_ccx = dados_sensor(3,coluna);
    float A_ccy = dados_sensor(4,coluna);
    float A_ccz = dados_sensor(5,coluna);
    float expressao = atan(A_ccy/sqrt((A_ccx*A_ccx+A_ccz*A_ccz)));
    return expressao;
}

float MPU_KalmanFilter::pitch_pelo_acc(char coluna)
{
    float A_ccx = dados_sensor(3,coluna);
    float A_ccy = dados_sensor(4,coluna);
    float A_ccz = dados_sensor(5,coluna);
    float expressao = atan(-A_ccx/sqrt((A_ccy*A_ccy+A_ccz*A_ccz)));
    return expressao;
}

void MPU_KalmanFilter::formata_dados_entrada(char coluna)
{
    dados_anteriores(0,0) = roll_pelo_acc(coluna);
    dados_anteriores(1,0) = pitch_pelo_acc(coluna);
    dados_anteriores(2,0) = dados_sensor(0,coluna);
    dados_anteriores(3,0) = dados_sensor(1,coluna);
}

void MPU_KalmanFilter::novo_estado_dados_previsto()
{
    previsao_inicial = matriz_F * dados_anteriores;
    previsao_inicial(0,0) = previsao_inicial(0,0) + pow(10,-12);
    previsao_inicial(1,0) = previsao_inicial(1,0) + pow(10,-12);
    previsao_inicial(2,0) = previsao_inicial(2,0) + pow(10,-12);
    previsao_inicial(3,0) = previsao_inicial(3,0) + pow(10,-12); 
}

void MPU_KalmanFilter::novo_estado_covariancia_prevista()
{
    covar_anterior=(matriz_F*covar_anterior)*~matriz_F;
    for (char i = 0; i < 4; i++)
    {
        for (char j = 0; j < 4; j++)
        {
            covar_anterior(i,j) = covar_anterior(i,j) + pow(10,-8); //Offset que influencia na velocidade de convergencia
            if(i!=j) covar_anterior(i,j) = 0;
        }
    }
}

void MPU_KalmanFilter::formata_matriz_R()
{
    float desvios_padrao[6];
    desvio_padrao(desvios_padrao);
    float A_ccx = dados_sensor(3,acessa_iter_anterior());
    float A_ccy = dados_sensor(4,acessa_iter_anterior());
    float A_ccz = dados_sensor(5,acessa_iter_anterior());
    float daccx = desvios_padrao[3];float daccy = desvios_padrao[4];float daccz = desvios_padrao[5];
    float dgirox = desvios_padrao[0]; float dgiroy = desvios_padrao[1];

    float droll = sqrt(
    (pow(A_ccx, 2) * pow(A_ccy, 2) * (pow(daccx, 2) + pow(daccz, 2)) + pow(daccy, 2) * pow(A_ccx * A_ccx + A_ccz * A_ccz, 2)) / 
    ((pow(A_ccx, 2) + pow(A_ccz, 2)) * pow(A_ccx * A_ccx + A_ccy * A_ccy + A_ccz * A_ccz, 2)));

    float dpitch = sqrt(
    (pow(A_ccx, 2) * pow(A_ccy, 2) * pow(daccy, 2) + pow(A_ccy * A_ccy + A_ccz * A_ccz, 2) * (pow(daccx, 2) + pow(daccz, 2))) /
    ((pow(A_ccy, 2) + pow(A_ccz, 2)) * pow(A_ccx * A_ccx + A_ccy * A_ccy + A_ccz * A_ccz, 2)));

    matriz_R.Fill(0);
    matriz_R(0,0) = droll*droll + pow(10,-12);
    matriz_R(1,1) = dpitch*dpitch + pow(10,-12);
    matriz_R(2,2) = dgirox*dgirox + pow(10,-12);
    matriz_R(3,3) = dgiroy*dgiroy + pow(10,-12);
}

void MPU_KalmanFilter::calculo_ganho_kalman()
{
    BLA::Matrix<4, 4> soma,matriz_inversa;
    soma = covar_anterior + matriz_R;
    matriz_inversa = soma;
    matriz_inversa = Inverse(matriz_inversa);
    ganho_kalman = covar_anterior*matriz_inversa;
    for (char i = 0; i < 4; i++)
    {
        for (char j = 0; j < 4; j++)
        {
            if(i!=j)ganho_kalman(i,j)=0;
        }
    }
}

void MPU_KalmanFilter::calculo_previsao_final_resultado()
{
    BLA::Matrix<4, 1> diferenca = dados_anteriores-previsao_inicial;
    previsao_final_resultado = previsao_inicial + ganho_kalman * (diferenca);
}

void nova_matriz_covariancia_anterior()
{
    BLA::Matrix<4, 4> matriz_identidade = {1,0,0,0,
                                  0,1,0,0,
                                  0,0,1,0,
                                  0,0,0,1};
    covar_anterior = (matriz_identidade-ganho_kalman)*covar_anterior;
}


// ========== Funções associadas à leitura e formatação inicial ===========

void MPU_KalmanFilter::envia_i2c(char id,char registro,char informacao2=0,bool info2=false,char informacao1=0,bool info1=0){
    Wire.beginTransmission(id);
    Wire.write(registro);
    if(info2==true)Wire.write(informacao2);
    if(info1==true)Wire.write(informacao1);
    Wire.endTransmission();
}

void MPU_KalmanFilter::leitura_MPU(int16_t *VarX,int16_t *VarY,int16_t *VarZ,char id,char registrador){
    envia_i2c(id,registrador);//Informa que deseja ler a partir do registro "registrador"
    Wire.requestFrom(MPU6050_ADDR,6);//Vamos Ler 6 Bytes em que cada informação tem 16 bits, logo 3*Gyros/Acc=48bits=6Bytes

    //O deslocamento de 8 bits eh preciso pq nosso registrador são de 16bits, o numero esta em complemento de 2, mas nao eh garantido que o compilador entenda. O valor indicado são as contagens (LSB) que deve ser transformado 
    *VarX = Wire.read()<<8 | Wire.read(); 
    *VarY = Wire.read()<<8 | Wire.read();
    *VarZ = Wire.read()<<8 | Wire.read();
}

unsigned long MPU_KalmanFilter::formata_leitura()
{
    float gx,gy,gz;       //rad/s em cada eixo
    float acx,acy,acz;    //Aceleracao em cada eixo[m/s^2]

    leitura_MPU(&GYRO_X,&GYRO_Y,&GYRO_Z,MPU6050_ADDR,0x43); //Leitura GYRO
    leitura_MPU(&ACC_X,&ACC_Y,&ACC_Z,MPU6050_ADDR,0x3B); //Leitura ACC
    unsigned long tempo = micros();

    gx = deg2rad((GYRO_X-GYRO_X_OFF)/SENSI_GIRO); //Valores em radianos por segundo
    gy = deg2rad((GYRO_Y-GYRO_Y_OFF)/SENSI_GIRO);gz = deg2rad((GYRO_Z-GYRO_Z_OFF)/SENSI_GIRO);

    acx = (ACC_X-ACC_X_OFF)/SENSI_ACC * GRAVIDADE; //Valores em m/s^2
    acy = (ACC_Y-ACC_Y_OFF)/SENSI_ACC * GRAVIDADE;acz = (ACC_Z-ACC_Z_OFF)/SENSI_ACC * GRAVIDADE;

    float dados_processados[] = {gx,gy,gz,acx,acy,acz};
    char coluna = iter_dados();
    for (char i = 0; i < 6; i++)
    {
        dados_sensor(i,coluna) = dados_processados[i]; 
    }
    return tempo;
}

void MPU_KalmanFilter::calibracao_MPU(int16_t *VarX,int16_t *VarY,int16_t *VarZ,int16_t *OffX,int16_t *OffY,int16_t *OffZ,char id,char registrador,int iteracoes)
{
    long int contagensX=0,contagensY=0,contagensZ=0;
    for(int i=0;i<iteracoes;i++){
        leitura_MPU(VarX,VarY,VarZ,id,registrador);
        contagensX = contagensX + *VarX;
        contagensY = contagensY + *VarY;
        contagensZ = contagensZ + *VarZ;
    }
    *OffX = contagensX/iteracoes;
    *OffY = contagensY/iteracoes;
    *OffZ = contagensZ/iteracoes;
}

void MPU_KalmanFilter::getRollPitch(float* ptr)
{
    tempo_atual=formata_leitura(); // Segundo Dado e Captura do tempo da segunda leitura
    t_amostra = tempo_atual-tempo_anterior;
    matriz_F =  {1,0,t_amostra*pow(10,-6),0,
                0,1,0,t_amostra*pow(10,-6),
                0,0,1,0,
                0,0,0,1};
    if(etapa_inicial==true)
    {
        etapa_inicial=false;
        formata_covar_inicial(); //Matriz de Covariancia Inicial
        formata_dados_entrada(0); //Coloca o primeiro dado no vetor "dados_anteriores"
    }
    novo_estado_dados_previsto(); //Calcula a previsao inicial baseado em dados_anteriores e matriz_F ETAPA 1
    novo_estado_covariancia_prevista(); //Calcula nova covariancia ETAPA 3, AJUSTAR OFFSET 10^-X PARA AJUSTE DE VELOCIDADE DE CONVERGENCIA
    formata_matriz_R();
    calculo_ganho_kalman(); //ETAPA 4
    tempo_atual=formata_leitura(); //Leitura dos novos dados para comparar na previsao final ETAPA 5
    formata_dados_entrada(acessa_iter_anterior()); //Coloca dado capturado acima no array "dados_anteriores"
    calculo_previsao_final_resultado(); //ETAPA 6

    float roll,pitch;
    roll = rad2deg(previsao_final_resultado(0,0));
    
    Serial.print("Roll = ");Serial.print(roll,2);
    Serial.print(" Roll Bruto = ");Serial.print(rad2deg(dados_anteriores(0,0)));
    if(ACC_Z<0){
        pitch = rad2deg(previsao_final_resultado(1,0));
    }
    else{
        pitch = -rad2deg(previsao_final_resultado(1,0));
    }
    Serial.print(" Pitch = ");Serial.print(pitch);
    Serial.print(" Pitch Bruto = ");Serial.println(rad2deg(dados_anteriores(1,0)));

    //Para evitar explosões e perda de foco do filtro kalman
    if ( rad2deg(previsao_final_resultado(0,0))>90 || rad2deg(previsao_final_resultado(0,0))<-90 ||
         rad2deg(previsao_final_resultado(1,0))>90 || rad2deg(previsao_final_resultado(1,0))<-90 ) 
    {
        covar_anterior = matriz_R;
    }
    nova_matriz_covariancia_anterior(); //ETAPA 7
    dados_anteriores = previsao_final_resultado; //ETAPA 8
    tempo_anterior = tempo_atual;

    *ptr = roll; //Angulo de Roll
    *(ptr+1) = pitch; //Angulo de Pitch
}

void MPU_KalmanFilter::prepara_dados()
{
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
    calibracao_MPU(&GYRO_X,&GYRO_Y,&GYRO_Z,&GYRO_X_OFF,&GYRO_Y_OFF,&GYRO_Z_OFF,MPU6050_ADDR,0x43,100); //Calibracao GYRO
    //calibracao_MPU(&ACC_X,&ACC_Y,&ACC_Z,&ACC_X_OFF,&ACC_Y_OFF,&ACC_Z_OFF,MPU6050_ADDR,0x3B,100); //Calibracao ACC
    tempo_anterior=formata_leitura(); //Primeira Leitura e Captura do tempo da primeira leitura
}

 
// void loop() {
//     float angulo[2];
//     get_roll_pitch(&angulo[0]);
// }
