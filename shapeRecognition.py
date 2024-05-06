#!/usr/bin/env python

# Librerie standard Python
import time
import cv2
import numpy as np

# Classi del progetto
import dummy

class ShapeRecognition:
    font = cv2.FONT_ITALIC
    camIndex = 0 # 0 per webcam incorporata, 1 per telecamere esterne
    isExecuting = False
    numberOfShapes = 3
    frameCounter = np.empty(numberOfShapes, dtype=int) # Contatore frame
    for i in range(numberOfShapes):
        frameCounter[i] = 0

    def __init__(self):
        self.aux = "a"

    # Funzione ausiliaria per il funzionamento delle trackbars
    def nothing(x):
        # any operation
        pass

    def startRecognition(self):
        cap = cv2.VideoCapture(self.camIndex)

        # Creazione delle trackbars per aggiustare la maschera
        # self.createTrackbars()

        # fps = cap.get(cv2.CAP_PROP_FPS)
        # print('Frames per second : ', fps, 'FPS')

        while True:
            _, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            """
            # Calibrazione della maschera
            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")
            """
            #lower_red = np.array([l_h, l_s, l_v])
            #upper_red = np.array([u_h, u_s, u_v])

            lower_red = (70, 160, 30) # Valori ricavati sperimentalmente
            upper_red = (180, 255, 243)

            # Creazione maschera
            mask = cv2.inRange(hsv, lower_red, upper_red)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel)

            # Riconoscimento contorni
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]

                # Rimozione del rumore
                if area > 400:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 3)

                    # Si calcola il numero di lati
                    if len(approx) == 4:
                        x, y, w, h = cv2.boundingRect(cnt)
                        ratio = float(w) / h
                        # @TODO: implementare una chiamata a funzione in base alla forma riconosciuta
                        if 0.9 <= ratio <= 1.1:
                            index = 0
                            for i in range(self.numberOfShapes):
                                if i != index:
                                    self.frameCounter[i] = 0
                            cv2.putText(frame, "Square", (x, y), self.font, 1, (0, 0, 0))
                            self.frameCounter[index] = self.frameCounter[index] + 1
                            self.validate(index, "Square")

                        elif 1.2 <= ratio <= 2.1:
                            index = 1
                            for i in range(self.numberOfShapes):
                                if i != index:
                                    self.frameCounter[i] = 0
                            cv2.putText(frame, "Rectangle", (x, y), self.font, 1, (0, 0, 0))
                            self.frameCounter[index] = self.frameCounter[index] + 1
                            self.validate(index, "Rectangle")

                        elif 0.1 <= ratio < 0.3:
                            index = 2
                            for i in range(self.numberOfShapes):
                                if i != index:
                                    self.frameCounter[i] = 0
                            cv2.putText(frame, "Tower", (x, y), self.font, 1, (0, 0, 0))
                            self.frameCounter[index] = self.frameCounter[index] + 1
                            self.validate(index, "Tower")
                        else:
                            for i in range(self.numberOfShapes):
                                self.frameCounter[i] = 0
                            cv2.putText(frame, "Unidentified", (x, y), self.font, 1, (0, 0, 0))

            cv2.imshow("Mask", mask)
            cv2.imshow("Frame", frame)

            # Attendo che l'utente prema "esc"
            key = cv2.waitKey(1)
            if key == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    def validate(self, index, shape):
        if self.frameCounter[index] / 30 > 5 and self.isExecuting is False:
            self.isExecuting = True
            self.frameCounter[index] = 0
            if(shape=="Square"):
                dummy.dummySquare()
                time.sleep(1)
            elif(shape=="Rectangle"):
                dummy.dummyRect()
                time.sleep(1)
            elif(shape=="Tower"):
                dummy.dummyTower()
                time.sleep(1)
            self.isExecuting = False

    def createTrackbars(self):
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 70, 180, ShapeRecognition.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 160, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 30, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 180, ShapeRecognition.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 243, 255, ShapeRecognition.nothing)


recognition = ShapeRecognition()
recognition.startRecognition()
