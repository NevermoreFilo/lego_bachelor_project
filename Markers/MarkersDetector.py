#!/usr/bin/env python
# Librerie standard python
import cv2
import imutils


# Classe per gestire il riconoscimento dei marker a partire da una foto
class MarkerDetector:
    def __init__(self, short_coord_list, long_coord_list):
        self.short_coord_list = short_coord_list
        self.long_coord_list = long_coord_list
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        self.dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_7X7_50"]) # uso lo stesso dizionario con cui ho generato i marker
        self.params = cv2.aruco.DetectorParameters_create()

    def detect(self):
        image = cv2.imread("/home/cross/progetto_tirocinio/lego_ws/src/lego/src/Vision/robot.png")
        #image = imutils.resize(image, width=1000)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.params)
        if len(corners) > 0:  # verifico che almeno UN marker sia stato trovato
            ids = ids.flatten()
            for (corners, ID) in zip(corners, ids):
                corners = corners.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # conversione di ogni coordinata xy in una coppia di interi
                topRight = (int(topRight[0]), int(topRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))

                # disegno le bounding box
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # calcolo e disegno le coordinate xy del centro del marker
                centrX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centrY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (centrX, centrY), 4, (0, 0, 255), -1)

                # mostro l'id del marker
                cv2.putText(image, str(ID), (topLeft[0], topRight[1] - 15),
                            cv2.FONT_ITALIC, 0.5, (0, 255, 0), 2)
                if(ID < 4):
                    self.short_coord_list[ID].active = False  # Se ho riconosciuto il marker vuol dire che la coordinata non e' attiva
                else:
                    self.long_coord_list[ID-4].active = False  # Se ho riconosciuto il marker vuol dire che la coordinata non e' attiva
            for i in range(0, 4):
                if (i not in ids):
                    self.short_coord_list[i].active = True
            for i in range(4, 10):
                if (i not in ids):
                    self.long_coord_list[i-4].active = True
            # mostro l'immagine di output
            #cv2.imshow("Image", image)
            #cv2.waitKey(0)
