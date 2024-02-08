import cv2
import numpy as np

video = int(input("Escolha um video:\n"))

if(video == 1):
    video_path = "MrsT.mp4"
else:
    video_path = "5XK0pnt.gif"

vid = cv2.VideoCapture(video_path) 

while(True): 
    # Capture the video frame by frame 
    ret, frame = vid.read() 

    # Display the resulting frame 
    if ret == True:

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Aplicar a limiarização (thresholding)
        limite = 180
        _, binary = cv2.threshold(gray, limite, 255, cv2.THRESH_BINARY)
        cv2.imshow('Video',binary)
        cv2.imshow('Video2',frame)

    # espera ate uma tecla ser pressioanda
        key = cv2.waitKey(0)
    # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'): 
            break
    # the 'q' button is to go to next frame
        elif key & 0xFF == ord('d'):
            continue
    else:
        break        
# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows()