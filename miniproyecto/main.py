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

#sigue la linea hasta topar un cuadro blanco
def seguir_linea():
    velocidad = 100
    Cuadro_blanco = 60

    robot.drive(velocidad, 0)
    #sigue la linea negra 
    wait(1000) #espera un seg

    while color_sensor.reflection() < Cuadro_blanco:
        wait (10) #aca espera un toque para comprobar el color

    robot.stop()


def girar(direccion):
    velocidad_giro = 100
    tiempo_giro = 1000
    if direccion == "izquierda":
        robot.drive_time(0, -velocidad_giro, tiempo_giro)
    elif direccion == "derecha":
        robot.drive_time(0, velocidad_giro, tiempo_giro)

    wait(1000)
    robot.stop()

def mover_a(ruta):
    for i in range(len(ruta)-1):
        seguir_linea()

        if i !=len(ruta) -2:
            #aca se calcula la direccion que debe girar
            nodo_actual = ruta[i]
            proximo_nodo = ruta[i+1]
            diferencia_x = ubicaciones[proximo_nodo][0] - ubicaciones[nodo_actual][0]

            if diferencia_x > 0:
                direccion = "derecha"
            else:
                direccion = "izquierda"
            #aca gira a donde debe
            girar(direccion)

# Llamar a la función mover_a con la ruta
mover_a(ruta)