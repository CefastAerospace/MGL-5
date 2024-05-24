import matplotlib.pyplot as plt
import numpy as np
import math

class Plota:
    def __init__(self):
        self.fig, self.ax = plt.subplots()  # Cria a figura e o subplot
        self.ax.set_xlabel('Número de Iterações')
        self.ax.set_ylabel('Valor da Variável')
        self.ax.set_title('Plot em Tempo Real')

        self.line1, = self.ax.plot([], [], linestyle='solid', color='red',linewidth=4.0, label='Roll')
        self.line2, = self.ax.plot([], [], linestyle='dashed', color='green',linewidth=2.0, label='Roll Bruto')
        self.line3, = self.ax.plot([], [], linestyle='solid', color='blue',linewidth=4.0, label='Pitch')
        self.line4, = self.ax.plot([], [], linestyle='dashed', color='black',linewidth=2.0, label='Pitch Bruto')
        self.max_iter = 200  # Limite máximo de pontos (opcional)
        self.iterador = 0

        # Define os limites do eixo y manualmente
        self.y_min = -110
        self.y_max = 110
        self.ax.set_ylim(self.y_min, self.y_max)

    def atualiza_grafico(self,dados_previsto,dados_captados):
        x_value = self.iterador
        roll = math.degrees(dados_previsto[0][0])
        roll_bruto = math.degrees(dados_captados[0][0])
        pitch = math.degrees(dados_previsto[1][0])
        pitch_bruto = math.degrees(dados_captados[1][0])

        #Atualizando os dados das séries
        if self.iterador >= self.max_iter:
            # Deleta os dados anteriores
            self.line1.set_xdata(self.line1.get_xdata()[1:])
            self.line1.set_ydata(self.line1.get_ydata()[1:])

            self.line2.set_xdata(self.line2.get_xdata()[1:])
            self.line2.set_ydata(self.line2.get_ydata()[1:])

            self.line3.set_xdata(self.line3.get_xdata()[1:])
            self.line3.set_ydata(self.line3.get_ydata()[1:])

            self.line4.set_xdata(self.line4.get_xdata()[1:])
            self.line4.set_ydata(self.line4.get_ydata()[1:])

        # Atualiza com os novos dados
        self.line1.set_xdata(np.append(self.line1.get_xdata(), x_value))
        self.line1.set_ydata(np.append(self.line1.get_ydata(), roll))

        self.line2.set_xdata(np.append(self.line2.get_xdata(), x_value))
        self.line2.set_ydata(np.append(self.line2.get_ydata(), roll_bruto))

        self.line3.set_xdata(np.append(self.line3.get_xdata(), x_value))
        self.line3.set_ydata(np.append(self.line3.get_ydata(), pitch))

        self.line4.set_xdata(np.append(self.line4.get_xdata(), x_value))
        self.line4.set_ydata(np.append(self.line4.get_ydata(), pitch_bruto))

        # Ajustando os limites dos eixos para ambas as curvas
        self.ax.relim()
        self.ax.autoscale_view()

        # Mostrando a legenda
        self.ax.legend(loc='upper left',fontsize='larger')

        #if(iterador==50):break
        # Definindo um tempo de pausa entre as atualizações
        #plt.waitforbuttonpress()  # Espera até que o usuário interaja com o gráfico

        # Incrementando o iterador
        self.iterador = self.iterador + 1

