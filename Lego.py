#!/usr/bin/env python

# Classi del progetto
import Robot
import ShapeRecognition


# La classe fa da "wrapper" per le altre due
class Lego:
    def __init__(self):
        self.robot = Robot.Robot()
        self.robot.init_coordinates()
        self.recognizer = ShapeRecognition.ShapeRecognition(self.robot)

    def main(self):
        self.robot.go_to_working_pose()
        self.recognizer.start_recognition()
        self.robot.go_to_starting_pose() # Una volta terminato il lavoro, il braccio ritorna ad una posizione "a riposo"


lego = Lego()
lego.main()
