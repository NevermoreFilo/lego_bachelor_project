import cv2
import numpy as np
import argparse
import sys

argparse = argparse.ArgumentParser()
argparse.add_argument("-o", "--output", required=True, help="percorso di output")
argparse.add_argument("-i", "--id", type=int, required=True, help="ID del marker ArUCo da generare (nel dizionario)")
argparse.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="nome del dizionario da usare")

args = vars(argparse.parse_args())

# Tutti i dizionari supportati da opencv2. Dizionari piu' piccoli con NxN minori sono meno proni ad errori
ARUCO_DICT = {
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

if ARUCO_DICT.get(args["type"], None) is None:
	print("il dizionario selezionato non e' supportato".format(
		args["type"]))
	sys.exit(0) # Se il dizionario non e' valido chiudo il programma

dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]]) # Altrimenti lo carica

tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(dictionary, args["id"], 300, tag, 1) # Generazione di marker grandi 300x300 pixel, in scala di grigi

cv2.imwrite(args["output"], tag)
cv2.imshow("Marker", tag)
cv2.waitKey(0)

#python Markers.py --id 0 --type DICT_7X7_50 --output Markers/DICT_7X7_50_id0.png
