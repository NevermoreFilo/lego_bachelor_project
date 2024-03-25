import cv2
import numpy as np

font = cv2.FONT_ITALIC
camIndex = 0


class RedShapesRecognition:
    def __init__(self):
        super(RedShapesRecognition, self).__init__()

    # Funzione ausiliaria per il funzionamento delle trackbars
    def nothing(x):
        # any operation
        pass

    def startRecognition(self):

        cap = cv2.VideoCapture(camIndex)

        # Inizio linee inerenti alla creazione delle trackbars per aggiustare la maschera

        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 0, 180, RedShapesRecognition.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 120, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 154, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 180, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, RedShapesRecognition.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 243, 255, RedShapesRecognition.nothing)

        # Fine

        while True:
            _, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Inizio linee inerenti alla calibrazione della maschera
            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")
            # Fine

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
                if area > 1000:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 3)

                # Si calcola il numero di lati
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(cnt)
                    ratio = float(w) / h
                    if 0.9 <= ratio <= 1.1:
                        cv2.putText(frame, "Square", (x, y), font, 1, (0, 0, 0))
                    elif 1.2 <= ratio <= 2.1:
                        cv2.putText(frame, "Rectangle 1", (x, y), font, 1, (0, 0, 0))
                    else:
                        cv2.putText(frame, "Rectangle 2", (x, y), font, 1, (0, 0, 0))

            cv2.imshow("Mask", mask)
            cv2.imshow("Frame", frame)

            #Attendo che l'utente prema "esc"
            key = cv2.waitKey(1)
            if key == 27:
                break

        cap.release()
        cv2.destroyAllWindows()


recognition = RedShapesRecognition()
recognition.startRecognition()
