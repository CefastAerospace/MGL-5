import numpy as np

#Etapa 1
def novo_estado_dados_previsto(matriz_F,matriz_dados_anterior):
    return (matriz_F @ matriz_dados_anterior)

#Etapa 3
def novo_estado_covariancia_prevista(matriz_F,matriz_covariancia_anterior):
    return ((matriz_F @ matriz_covariancia_anterior) @ np.transpose(matriz_F))

#Etapa 4
def calculo_ganho_kalman(matriz_covariancia_atual,matriz_erro_dados):
    try:
        matriz_inversa = np.linalg.inv(matriz_covariancia_atual + matriz_erro_dados)
    except np.linalg.LinAlgError:
        print("A matriz não é invertível.")
    ganho = matriz_covariancia_atual @ matriz_inversa
    return ganho

#Etapa 5
def calculo_previsao_final_resultado(matriz_dados_prevista,ganho_kalman,dados_coletados):
    return matriz_dados_prevista + ganho_kalman @ (dados_coletados-matriz_dados_prevista)

#Etapa 6
def nova_matriz_covariancia_anterior(matriz_covariancia_anterior,ganho_kalman):
    return ((np.eye(ganho_kalman.shape[0])-ganho_kalman) @ matriz_covariancia_anterior)