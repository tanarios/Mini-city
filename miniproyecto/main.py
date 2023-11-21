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
inicio_conexiones = {"bomberos": 1}
bomberos_conexiones = {"inicio": 1, "museo": 2, "policia": 2, "iglesia": 1}
museo_conexiones = {"bomberos": 2, "restaurante": 1}
restaurante_conexiones = {"museo": 1, "iglesia": 3, "gasolinera": 1}
gasolinera_conexiones = {"restaurante": 1, "parque": 2, "centro_comercial": 1}
centro_comercial_conexiones = {"gasolinera": 1, "gym": 1}
gym_conexiones = {"centro_comercial": 1, "parque": 3, "hotel": 1}
parque_conexiones = {"gym": 3, "gasolinera": 2, "iglesia": 3, "colegio": 3}
iglesia_conexiones = {"parque": 3, "restaurante": 3, "bomberos": 1, "municipalidad": 2}
municipalidad_conexiones = {"iglesia": 2, "colegio": 2, "policia": 2}
policia_conexiones = {"municipalidad": 2, "bomberos": 2}
hotel_conexiones = {"gym": 1, "colegio": 1}
colegio_conexiones = {"hotel": 1, "parque": 3, "municipalidad": 2}


# Definir el grafo
City = {
    "inicio": inicio_conexiones,
    "bomberos": bomberos_conexiones,
    "museo": museo_conexiones,
    "restaurante": restaurante_conexiones,
    "gasolinera": gasolinera_conexiones,
    "centro_comercial": centro_comercial_conexiones,
    "gym": gym_conexiones,
    "parque": parque_conexiones,
    "iglesia": iglesia_conexiones,
    "municipalidad": municipalidad_conexiones,
    "policia": policia_conexiones,
    "hotel": hotel_conexiones,
    "colegio": colegio_conexiones

}
# Aca se ponen las coordenas de cada punto 
ubicaciones={
    "inicio": (0,0),
    "bomberos": (0,1),
    "museo": (-1,1),
    "restaurante": (-1,2),
    "gasolinera": (-1,3),
    "centro_comercial": (-1,4),
    "gym": (0,4),
    "parque": (0,3),
    "iglesia": (0,2),
    "municipalidad": (1,2),
    "policia": (1,1),
    "hotel": (1,4),
    "colegio": (1,3)

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


#ev3.speaker.say('Padre nuestro que estás en el cielo, santificado sea tu Nombre; venga a nosotros tu Reino; hágase tu voluntad  en la tierra como en el cielo. Danos hoy  nuestro pan de cada día; perdona nuestras ofensas, como también nosotros perdonamos  a los que nos ofenden; no nos dejes caer en la tentación, y líbranos del mal. Amén.')

#sigue la linea hasta topar un cuadro blanco
def seguir_linea():
    velocidad = 100
    Cuadro_blanco = 60

    robot.drive(velocidad, 0)
    #sigue la linea negra 
    wait(1000) #espera un seg

    while color_sensor.reflection() < Cuadro_blanco:
        
        wait (1000) #aca espera un toque para comprobar el color

    robot.stop()


def girar(direccion):
    velocidad_giro = 100
    tiempo_giro = 1500
    if direccion == "izquierda":
        robot.drive_time(0, -velocidad_giro, tiempo_giro)
    elif direccion == "derecha":
        robot.drive_time(0, velocidad_giro, tiempo_giro)
    elif direccion == "arriba":
        robot.drive_time(velocidad_giro, 0, tiempo_giro)
    elif direccion == "abajo":
        robot.drive_time(-velocidad_giro, 0, tiempo_giro)


    wait(1000)
    robot.stop()

def mover_a(ruta):
    for i in range(len(ruta) - 1):
        seguir_linea()
        ev3.speaker.say(ruta[i])
        ev3.speaker.beep()

        # Calcular la dirección que debe girar
        nodo_actual = ruta[i]
        proximo_nodo = ruta[i+1]
        diferencia_x = ubicaciones[proximo_nodo][0] - ubicaciones[nodo_actual][0]
        diferencia_y = ubicaciones[proximo_nodo][1] - ubicaciones[nodo_actual][1]

        if diferencia_x > 0:
            direccion = 'derecha'
            girar(direccion)
            print(ruta[i], direccion)

        elif diferencia_x < 0:
            direccion = 'izquierda'
            girar(direccion)
            print (ruta[i], direccion)
                
        
        elif diferencia_x  == 0:

            if diferencia_y > 0:
                direccion = 'izquierda'  # Girar a la izquierda para moverse hacia abajo en el mapa
                girar(direccion)
                print (ruta[i], direccion)

            elif diferencia_y < 0:
                direccion = 'derecha'  # Girar a la derecha para moverse hacia arriba en el mapa
                girar(direccion)
                print (ruta[i], direccion)

    # Para el último nodo
    seguir_linea()
    ev3.speaker.say(ruta[-1])
    ev3.speaker.beep()

        
# Llamar a la función mover_a con la ruta

#aca llama la funcion y le pasa el grafo y el nodo que se seleecione
#se pone donde quiere iniciar
distancias, caminos = dijkstra(City, 'inicio')

# Imprimir las distancias del nodo que se eligio y el camino mas corto de nodo que se elige 
print(distancias)
print(caminos['gasolinera'])
#aca se guarda la ruta mas corta en ruta para poder usarla en el robot y se pone hasta donde quiere llegar
ruta = caminos['gasolinera']
mover_a(ruta)