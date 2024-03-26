import cv2
import numpy as np

import dummy


class RedShapesRecognition:
    font = cv2.FONT_ITALIC
    camIndex = 0
    isExecuting = False
    numberOfShapes = 3
    shapesRecognized = 0
    frameCounter = np.empty(numberOfShapes, dtype=int)
    for i in range(numberOfShapes):
        frameCounter[i] = 0

    def __init__(self):
        super(RedShapesRecognition, self).__init__()

    # Funzione ausiliaria per il funzionamento delle trackbars
    def nothing(x):
        # any operation
        pass

    def startRecognition(self):
        cap = cv2.VideoCapture(self.camIndex)

        # Creazione delle trackbars per aggiustare la maschera
        self.createTrackbars()

        # fps = cap.get(cv2.CAP_PROP_FPS)
        # print('Frames per second : ', fps, 'FPS')

        while True:
            _, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Calibrazione della maschera
            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")

            lower_red = np.array([l_h, l_s, l_v])
            upper_red = np.array([u_h, u_s, u_v])

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
                            self.validate(index)

                        elif 1.2 <= ratio <= 2.1:
                            index = 1
                            for i in range(self.numberOfShapes):
                                if i != index:
                                    self.frameCounter[i] = 0
                            cv2.putText(frame, "Rectangle 1", (x, y), self.font, 1, (0, 0, 0))
                            self.frameCounter[index] = self.frameCounter[index] + 1
                            self.validate(index)

                        elif 2.2 <= ratio <= 4.1:
                            index = 2
                            for i in range(self.numberOfShapes):
                                if i != index:
                                    self.frameCounter[i] = 0
                            cv2.putText(frame, "Rectangle 2", (x, y), self.font, 1, (0, 0, 0))
                            self.frameCounter[index] = self.frameCounter[index] + 1
                            self.validate(index)
                        else:
                            for i in range(self.numberOfShapes):
                                self.frameCounter[i] = 0
                            cv2.putText(frame, "Unidentified", (x, y), self.font, 1, (0, 0, 0))
                    else:
                        self.shapesRecognized = 0

            cv2.imshow("Mask", mask)
            cv2.imshow("Frame", frame)

            # Attendo che l'utente prema "esc"
            key = cv2.waitKey(1)
            if key == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    def validate(self, index):
        if self.frameCounter[index] / 30 > 5 and self.isExecuting is False:
            self.isExecuting = True
            self.frameCounter[index] = 0
            dummy.dummy()
            self.isExecuting = False

    def createTrackbars(self):
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 70, 180, RedShapesRecognition.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 160, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 30, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 180, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 243, 255, RedShapesRecognition.nothing)


recognition = RedShapesRecognition()
recognition.startRecognition()
