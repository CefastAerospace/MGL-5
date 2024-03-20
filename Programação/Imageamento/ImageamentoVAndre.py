import cv2
import numpy as np

#def empty(a):pass

centro = (0,0)
maiorOcorrencia = 0
maiorArea = 0

"""video_path = "MrsT.mp4"
vid = cv2.VideoCapture(video_path) 
ret, frame = vid.read() 
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Hue max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Sat min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Sat max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Val min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Val max", "TrackBars", 255, 255, empty)
while True:
    ret, frame = vid.read()

    if not ret:
        print("No more frames to read")
        break
    while True:
        h_min = cv2.getTrackbarPos("Hue min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val max", "TrackBars")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(frame, lower, upper)
        result = cv2.bitwise_and(frame,frame,mask=mask)
        cv2.imshow('Frame', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Resultado', result)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

vid.release()
cv2.destroyAllWindows()"""
    

video = int(input("Escolha um video:\n"))

# Coloca um limite de noise de acordo com o vídeo escolhido
if(video == 1):
    video_path = "MrsT.mp4"
    areaLimite = 6
else:
    video_path = "5XK0pnt.gif"
    areaLimite = 25

vid = cv2.VideoCapture(video_path) 

while(True): 
    # Captura o vídeo frame por frame
    ret, frame = vid.read() 
    # Exibe o video 
    if ret == True:
        if(video == 1):
            blur = cv2.GaussianBlur(frame,(9,9),2)
            lower = np.array([190, 0, 0])
            upper = np.array([255, 255, 230])
            # Aplica uma máscara para apenas luz azul
            mask = cv2.inRange(blur, lower, upper)
            canny = cv2.Canny(mask,50,50)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray,(9,9),2)
            # Aplica a limiarização (thresholding)
            limite = 184
            _, binary = cv2.threshold(blur, limite, 255, cv2.THRESH_BINARY)
            canny = cv2.Canny(binary,50,50)

        # Pega os contornos presentes na imagem
        contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Usa um limite de aerea pra evitar noise
            if area > areaLimite:
                if area > maiorArea : maiorArea = area
                peri = cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt,0.01*peri,True)
                x, y, w, h = cv2.boundingRect(approx)
                # Caso o centro esteja dentro de um intervalo considera como o mesmo raio
                if ((x+(w//2)-12) < centro[0] < (x+(w//2)+12) and (y+(h//2)-12) < centro[1] < (y+(h//2)+12)):
                    conta += 1
                    if conta > maiorOcorrencia : maiorOcorrencia = conta
                else:
                    conta = 1

                # Calcula o centro do raio
                centro = (x+(w//2),y+(h//2))

                # Coloca o retangulo delimitando a area do raio
                #cv2.putText(frame,str(centro),centro,2,1,(255,255,255),2)
                #cv2.circle(frame,centro,2,(0,0,0),2)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
        
        cv2.imshow('Video',frame)
    # Espera ate uma tecla ser pressioanda
        key = cv2.waitKey(150)
    # Botão 'q' para sair do vídeo
        if key & 0xFF == ord('q'): 
            break
    # Botão 'd' para passar pro próximo frame
        elif key & 0xFF == ord('d'):
            continue
    else:
        break      

print("Maior tempo de ocorrência: ",maiorOcorrencia)  
print("Maior area: ",maiorArea)
# Após o loop, libera o video
vid.release() 
# Fecha as janelas
cv2.destroyAllWindows()
