#!/usr/bin/env python

# Classe per rappresentare un punto in uno spazio cartesiano a 3 dimensioni

class PickCoordinate:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


# Classe per rappresentare un punto in uno spazio cartesiano a 3 dimensioni +  la sua disponibilita'

class PlaceCoordinate:
    def __init__(self, x, y, z, active):
        self.x = x
        self.y = y
        self.z = z
        self.active = active

