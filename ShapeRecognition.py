#!/usr/bin/env python

# Librerie standard Python
import time
import cv2
import numpy as np


# Classe che si occupa della parte di riconoscimento delle figure su un foglio
class ShapeRecognition:

    def __init__(self, robot):
        self.robot = robot  # Il robot passato come argomento al costruttore dev'essere gia' stato inizializzato

        self.cam_index = 0  # 0 per webcam incorporata, 1 per telecamere esterne
        self.is_executing = False
        self.numer_of_shapes = 3  # Var ausiliaria
        self.frame_counter = np.empty(self.numer_of_shapes, dtype=int)  # Contatore frame
        for i in range(
                self.numer_of_shapes):  # Ogni forma ha associato un contatore che misura per quanti frame e' stata
            self.frame_counter[i] = 0  # riconosciuta. Tutti i contatori sono inizializzati a 0
        self.font = cv2.FONT_ITALIC
        self.lower_red = (70, 160, 30)  # Valori ricavati sperimentalmente
        self.upper_red = (180, 255, 243)

    # Funzione principale, avvia la webcam e la versione mascherata, termina solo quando l'utente preme esc
    def start_recognition(self):
        cap = cv2.VideoCapture(self.cam_index)

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
            lower_red = np.array([l_h, l_s, l_v])
            upper_red = np.array([u_h, u_s, u_v])
            """

            #lower_red = (70, 160, 30)  # Valori ricavati sperimentalmente
            #upper_red = (180, 255, 243)

            # Creazione maschera
            mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
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
                        if 0.9 <= ratio <= 1.1:  # Se il ratio e' circa 1:1 si assume sia un quadrato
                            index = 0
                            for i in range(
                                    self.numer_of_shapes):  # Mentre una forma e' riconosciuta, si azzerano tutti gli altri counter
                                if i != index:  # Questo impedisce conflitti dati da piu' immagini riconosciute contemporaneamente
                                    self.frame_counter[i] = 0
                            cv2.putText(frame, "Square", (x, y), self.font, 1, (0, 0, 0))
                            self.frame_counter[index] = self.frame_counter[index] + 1
                            self.validate(index, "Square")

                        elif 1.2 <= ratio <= 2.1:  # Se il ratio e' circa 2:1 si assume sia un rettangolo
                            index = 1
                            for i in range(self.numer_of_shapes):
                                if i != index:
                                    self.frame_counter[i] = 0
                            cv2.putText(frame, "Rectangle", (x, y), self.font, 1, (0, 0, 0))
                            self.frame_counter[index] = self.frame_counter[index] + 1
                            self.validate(index, "Rectangle")

                        elif 0.1 <= ratio < 0.3:  # Se il ratio e' di circa 1:4, si assume sia una "torre"
                            index = 2
                            for i in range(self.numer_of_shapes):
                                if i != index:
                                    self.frame_counter[i] = 0
                            cv2.putText(frame, "Tower", (x, y), self.font, 1, (0, 0, 0))
                            self.frame_counter[index] = self.frame_counter[index] + 1
                            self.validate(index, "Tower")
                        else:
                            for i in range(self.numer_of_shapes):
                                self.frame_counter[i] = 0
                            cv2.putText(frame, "Unidentified", (x, y), self.font, 1, (0, 0, 0))

            cv2.imshow("Frame - Press 'esc' to quit", frame)  # Mostra cio' che vede la webcam
            cv2.imshow("Mask - Press 'esc' to quit", mask)  # Mostra la versione con la maschera

            # Attendo che l'utente prema "esc"
            key = cv2.waitKey(1)
            if key == 27:
                break

        cap.release()  # Una volta che l'utente preme esc, chiudo le finestre e termino il programma
        cv2.destroyAllWindows()

    # Funzione per contare il numero di frame consecutivi per cui la forma e' stata riconosciuta
    # Se sono passati almeno 3 secondi dal suo riconoscimento, chiama la relativa procedura del braccio robotico
    def validate(self, index, shape):
        if self.frame_counter[index] / 30 > 3 and self.is_executing is False:
            self.is_executing = True
            self.frame_counter[index] = 0
            if (shape == "Square"):
                #dummy.dummySquare()
                self.robot.build_square()
                time.sleep(1)
            elif (shape == "Rectangle"):
                #dummy.dummyRect()
                self.robot.build_rectangle()
                time.sleep(1)
            elif (shape == "Tower"):
                #dummy.dummyTower()
                self.robot.build_tower()
                time.sleep(1)
            self.is_executing = False  # La variabile is_executing serve ad implementare una sorta di mutex

    # Funzione ausiliaria necessaria alle trackbars
    def nothing(self):
        pass

    # Funzione ausiliaria, serve a mostrare a schermo delle trackbars per calibrare piu' comodamente la maschera
    def create_trackbars(self):
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 70, 180, ShapeRecognition.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 160, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 30, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 180, ShapeRecognition.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, ShapeRecognition.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 243, 255, ShapeRecognition.nothing)
