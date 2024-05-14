import numpy as np
import re
import sys
import time
from copy import deepcopy
import math

#Bibliotecas Externas na Pasta
from ports import *
from kalman_mpu import *

def salva_dados(objeto_serial : serial.Serial,iteracoes : int,sensibilidade_giro : float,sensibilidade_acc : float,nome_txt):
    valores = np.empty((iteracoes,6))
    for k in range(iteracoes):
        dados_sensor = leitura_mpu(obj_ser)
        dados_sensor = np.append((dados_sensor[:3])/sensibilidade_giro,dados_sensor[3:]/sensibilidade_acc)
        valores[k:] = dados_sensor
    #fmt="%.8f"
    np.savetxt(nome_txt, valores, delimiter=", ")  # Example format

def calibracao_mpu_giro(objeto_serial : serial.Serial,iteracoes : int,sensibilidade_giro,sensibilidade_acc):
    GyroX_off = GyroY_off = GyroZ_off =  0
    contador = iteracoes
    while (True):
        lido = leitura_mpu(objeto_serial,sensibilidade_giro,sensibilidade_acc)
        if np.size(lido) != 0:
            GyroX_off = GyroX_off + lido[0]
            GyroY_off = GyroY_off + lido[1]
            GyroZ_off = GyroZ_off + lido[2]
            contador = contador - 1
        if(contador==0): break
    return np.array([GyroX_off/iteracoes,GyroY_off/iteracoes,GyroZ_off/iteracoes])

def leitura_mpu(objeto_serial : serial.Serial,sensibilidade_giro,sensibilidade_acc,gyro_off = 0):
    try:    
        output = objeto_serial.readline()
    except serial.SerialException as e:
        print("Connection error:", e)
        sys.exit(1)
    output_str = str(output, encoding='ascii', errors='backslashreplace')
    lido = np.array(re.findall(r'-?\d+', output_str)).astype(float)
    if np.size(lido) != 0:
        lido = np.append(lido[:3]/sensibilidade_giro-gyro_off,lido[3:]/sensibilidade_acc)
        #lido = lido.reshape(6,1)
    return lido

def roll_pelo_acc(dados):
    A_ccx = dados[3]
    A_ccy = dados[4]
    A_ccz = dados[5]
    expressao = math.degrees(math.atan(A_ccy/math.sqrt((A_ccx)**2+(A_ccz)**2)))
    return expressao

def pitch_pelo_acc(dados):
    A_ccx = dados[3]
    A_ccy = dados[4]
    A_ccz = dados[5]
    expressao = math.degrees(math.atan(A_ccx/math.sqrt((A_ccy)**2+(A_ccz)**2)))
    return expressao

def formata_covar_inicial(dados):
    A_ccx = dados[3]
    A_ccy = dados[4]
    A_ccz = dados[5]
    droll = 0.031446603773522*math.sqrt((A_ccx**2*A_ccy**2 + 0.280898876404494*(A_ccx**2 + A_ccz**2)**2)/((A_ccx**2 + A_ccz**2)*(A_ccx**2 + A_ccy**2 + A_ccz**2)**2))
    dpitch = 0.031446603773522*math.sqrt((0.280898876404494*A_ccx**2*A_ccy**2 + (A_ccy**2 + A_ccz**2)**2)/((A_ccy**2 + A_ccz**2)*(A_ccx**2 + A_ccy**2 + A_ccz**2)**2))

    covariancia = np.array([        [droll**2,      droll*dpitch,   droll*20/3,     droll*20/3], 
                                    [dpitch*droll,  dpitch**2,      dpitch*20/3,    dpitch*20/3],
                                    [20/3*droll,    20/3*dpitch,    400/9,          400/9],
                                    [20/3*droll,    20/3*dpitch,    400/9,          400/9] ]) 
    #Define uma matriz de covariancia inicial partindo dos dados
    return covariancia

def formata_dados_entrada(dados):
    A_ccx = dados[3]
    A_ccy = dados[4]
    A_ccz = dados[5]
    return np.array([[roll_pelo_acc(dados)],[pitch_pelo_acc(dados)],[dados[0]],[dados[1]]])

def formata_matriz_R(dados,iterador):
    desvios_padrao = np.nanstd(dados, axis=1)
    A_ccx = dados[3,iterador-1]
    A_ccy = dados[4,iterador-1]
    A_ccz = dados[5,iterador-1]
    daccx = desvios_padrao[3];daccy = desvios_padrao[4];daccz = desvios_padrao[5]
    dgirox = desvios_padrao[0];dgiroy = desvios_padrao[1]

    droll = math.sqrt((A_ccx**2*A_ccy**2*(daccx**2 + daccz**2) + daccy**2*(A_ccx**2 + A_ccz**2)**2)/((A_ccx**2 + A_ccz**2)*(A_ccx**2 + A_ccy**2 + A_ccz**2)**2))
    dpitch = math.sqrt((A_ccx**2*A_ccy**2*daccy**2 + (A_ccy**2 + A_ccz**2)**2*(daccx**2 + daccz**2))/((A_ccy**2 + A_ccz**2)*(A_ccx**2 + A_ccy**2 + A_ccz**2)**2))
    
    matriz_R = np.array([           [droll**2,      droll*dpitch,   droll*dgirox,     droll*dgiroy], 
                                    [dpitch*droll,  dpitch**2,      dpitch*dgirox,    dpitch*dgiroy],
                                    [dgirox*droll,  dgirox*dpitch,  dgirox**2,        dgirox*dgiroy],
                                    [dgiroy*droll,  dgiroy*dpitch,  dgiroy*dgirox,    dgiroy**2] ]) 
    return matriz_R

def iter_dados():
    global iter_salva
    if (iter_salva == 30):
        iter_salva = 0
    else:
        iter_salva = iter_salva+1
    return iter_salva - 1

def main():
    np.set_printoptions(formatter={'float': lambda x: f'{x:.2e}'}) # Para plotar arrays Numpy mais elegante
    print(serial_ports()) #Printa portas seriais disponiveis
    porta_serial = 'COM3' 

    try:
        global obj_ser    
        obj_ser = connect_with_port(porta_serial) #Tenta conectar com a porta serial, caso falhe o programa finaliza
    except serial.SerialException as e:
        print("Connection error:", e)
        sys.exit(1)

    sensi_giro = 65.5; sensi_acc = 2048; #Sensibilidade do Giro e do Acelerometro
    tempo_anterior = tempo_atual = 0; #Para capturar o tempo, obtendo tempo de amostragem
    etapa_inicial=True #Flag para a etapa inicial
    global iter_salva #Iterador para acumular dados do sensor num array 6x30
    iter_salva = 0 #Inicia Iterador

    #Para iniciar as comunicações
    calibracao_mpu_giro(obj_ser,60,sensi_giro,sensi_acc) #Essa funcao obtem calibracao, mas so estamos usando para retirar o lixo inicial da comunicacao
    #salva_dados(obj_ser,10000,sensibilidade_giro,sensibilidade_acc,"dados1.txt") #Para salvar 10000 dados num arquivo txt

    #gyro_off = np.array([4.492303816793902,0.683581679389309,0.160436641221373])
    #gyro_off = np.array([0,0,0])
    gyro_off = 0 #A correcao de erro/offset inicial do gyro nao eh necessario mas caso queira usar descomente acima

    dados_sensor = np.full((6, 30), np.nan) #Aqui serao salvos os dados do sensor, num total 6x30 = 180 valores
    dados_sensor[:,iter_dados()] = leitura_mpu(obj_ser,sensi_giro,sensi_acc,gyro_off) #Captura dados iniciais apenas para pegar t_amostragem
    tempo_anterior = time.time() #Captura o tempo para obter t_amostra
    
    while(True):
        dados_sensor[:,iter_dados()] = leitura_mpu(obj_ser,sensi_giro,sensi_acc,gyro_off) #"Segundo Dado"
        tempo_atual = time.time() #Captura o tempo para obter t_amostra
        t_amostra = tempo_atual-tempo_anterior #Calcula tempo de amostragem para usar na matriz F
        matriz_F = np.array([[1,0,t_amostra,0],[0,1,0,t_amostra],[0,0,1,0],[0,0,0,1]]) #Matriz F = Matriz de Transicao
        if(etapa_inicial==True):
            #Etapa 2
            covar_anterior = formata_covar_inicial(dados_sensor[:,0]) #Obtem covariancia inicial a partir do primeiro dado
            #covar_anterior[~np.eye(covar_anterior.shape[0], dtype=bool)] = 0 #Zera Elementos fora da diagona principal
            dados_anteriores = formata_dados_entrada(dados_sensor[:,0]) #Formata dados capturados para o formato em 4 Linhas [Roll,Pitch,Gx,Gy]
            etapa_inicial = False #Flag para desabilitar a etapa inicial
        #Etapa 1
        previsao_inicial = novo_estado_dados_previsto(matriz_F,dados_anteriores) #Calcula previsao inicial dos dados
        #Etapa 3
        covar_anterior = novo_estado_covariancia_prevista(matriz_F,covar_anterior) #Atualiza matriz de covariância
        #covar_anterior[~np.eye(covar_anterior.shape[0], dtype=bool)] = 0 #Zera Elementos fora da diagona principal

        matriz_R = formata_matriz_R(dados_sensor,iter_salva)
        #matriz_R[~np.eye(matriz_R.shape[0], dtype=bool)] = 10**-9

        #Etapa 4
        ganho_kalman = calculo_ganho_kalman(covar_anterior,matriz_R) #Calcula Ganho Kalman a partir da covariancia e da matriz de erros R

        ganho_kalman[~np.eye(ganho_kalman.shape[0], dtype=bool)] = 0 #Zera Elementos fora da diagona principal, eh necessario caso contrario a solucao nao converge

        #Etapa 5
        dados_sensor[:,iter_dados()] = leitura_mpu(obj_ser,sensi_giro,sensi_acc,gyro_off) #Le novamente o sensor
        tempo_atual = time.time() #Captura o tempo novamente para usar na proxima iteracao do loop, com isso corrigimos o tempo de amostragem
        matriz_entrada = formata_dados_entrada(dados_sensor[:,iter_salva-1]) #Formata dados capturados para o formato em 4 Linhas [Roll,Pitch,Gx,Gy]

        #Etapa 6    
        previsao_final_resultado = calculo_previsao_final_resultado(previsao_inicial,ganho_kalman,matriz_entrada) # Previsao final do resultado
        
        print(f"Roll = {previsao_final_resultado[0][0]:.2f} Roll_Bruto = {matriz_entrada[0][0]:.2f} Pitch = {previsao_final_resultado[1][0]:.2f} Pitch_Bruto = {matriz_entrada[1][0]:.2f}")


        #Etapa 7
        covar_anterior = nova_matriz_covariancia_anterior(covar_anterior,ganho_kalman) #Atualiza matriz de covariância
        #covar_anterior[~np.eye(covar_anterior.shape[0], dtype=bool)] = 0 #Zera Elementos fora da diagona principal

        dados_anteriores = previsao_final_resultado #Realimenta o processo com os dados obtidos da previsao final
        tempo_anterior = tempo_atual #Atualiza tempo para proximo loop
    
if __name__ == "__main__":
    main()