#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import heapq

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.A) 
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
# Write your program here.

# Definir las conexiones y pesos para cada punto
inicio_conexiones = {"policia": 5}
policia_conexiones = {"inicio": 5, "iglesia": 10, "museo": 5}
iglesia_conexiones = {"policia": 10, "museo": 2}
museo_conexiones = {"iglesia": 2, "policia": 5}

# Definir el grafo
City = {
    "inicio": inicio_conexiones,
    "iglesia": iglesia_conexiones,
    "policia": policia_conexiones,
    "museo": museo_conexiones
}
# Aca se ponen las coordenas de cada punto 
ubicaciones={
    "inicio": (0,0),
    "policia": (0,1),
    "museo": (-1,2),
    "iglesia":(1,2)
}

# Definir la funcion de dijkstra para encontrar la ruta mas corta
def dijkstra(grafo, inicio):
    distancias = {nodo: float('infinity') for nodo in grafo}
    distancias[inicio] = 0

    caminos = {nodo: [] for nodo in grafo}
    caminos[inicio] = [inicio]

    cola_prioridad = [(0, inicio)]

    while cola_prioridad:
        distancia_actual, nodo_actual = heapq.heappop(cola_prioridad)

        if distancia_actual != distancias[nodo_actual]:
            continue

        for vecino, distancia in grafo[nodo_actual].items():
            distancia_tentativa = distancia_actual + distancia

            if distancia_tentativa < distancias[vecino]:
                distancias[vecino] = distancia_tentativa
                caminos[vecino] = caminos[nodo_actual] + [vecino]
                heapq.heappush(cola_prioridad, (distancia_tentativa, vecino))

    return distancias, caminos

#aca llama la funcion y le pasa el grafo y el nodo que se seleecione
distancias, caminos = dijkstra(City, 'inicio')

# Imprimir las distancias del nodo que se eligio y el camino mas corto de nodo que se elige 
print(distancias)
print(caminos['iglesia'])
#aca se guarda la ruta mas corta en ruta para poder usarla en el robot
ruta = caminos['iglesia']

#ev3.speaker.say('Padre nuestro que estás en el cielo, santificado sea tu Nombre; venga a nosotros tu Reino; hágase tu voluntad  en la tierra como en el cielo. Danos hoy  nuestro pan de cada día; perdona nuestras ofensas, como también nosotros perdonamos  a los que nos ofenden; no nos dejes caer en la tentación, y líbranos del mal. Amén.')

def mover_a(ruta):
    VELOCIDAD = 100  
    CUADRO_BLANCO = 60  

    # Mover el robot a lo largo de la ruta
    for nodo in ruta:
        # Mover el robot a lo largo de la línea negra
        robot.drive(VELOCIDAD, 0)

        # Esperar un poco para salir del cuadro blanco inicial
        wait(500)  # Esperar medio segundo

        # Esperar hasta que el robot detecte un cuadro blanco
        while color_sensor.reflection() < CUADRO_BLANCO:
            wait(10)  # Esperar un poco antes de comprobar de nuevo

        # Detener el robot
        robot.stop()

        # Si este no es el último nodo, esperar un poco antes de continuar
        if nodo != ruta[-1]:
            wait(2000)  # Esperar 2 segundos



# Llamar a la función mover_a con la ruta
mover_a(ruta)