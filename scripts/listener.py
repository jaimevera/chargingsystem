#!/usr/bin/env python
# coding=utf-8

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import time  			#librería para las pausas.
from std_msgs.msg import Int32	#librería para los mensajes enviados.


codeL = [0,0,0]		#codeL guarda el código recibido por el receptor izquierdo (visto desde el robot). 
codeLC = [0,0,0]	#codeLC guarda el código recibido por el receptor izquierdo central.
codeRC = [0,0,0]	#codeRC guarda el código recibido por el receptor derecho central.
codeR = [0,0,0]		#codeR guarda el código recibido por el receptor derecho.

codigo = 0b1000000000000    #valores iniciales para los mensajes de las señales IR.
code = 0b1000000000000		

distanciaC=1		#valores iniciales para las distancias de los sensores ultrasónicos en cm.
distanciaR=1
distanciaL=1
volt=0				
tension=14		#Variable para guardar el valor de la tensión antes de la conexión a la base y realizar la comparación.
#recargar = 1    	#copiada más abajo #Variable que indica si el robot necesita recarga (=1) o si no la necesita (=0).

tiempo_recarga = 0


giro=0			#guarda el sentido del giro que realiza.
encoder_inicio = 0  	#valor para la lectura del encoder.

##Funciones que declaran las lecturas que se realizan de las tarjetas Arduino.
##Cada una de ellas corresponde a un tópico.

def callbackC(data):		#Lecturas de las distancias de los sensores ultrasónicos.
    global distanciaC	
    distanciaC=data.data
	
def callbackR(data):
    global distanciaR	
    distanciaR=data.data

def callbackL(data):
    global distanciaL
    distanciaL=data.data

def callbackEnc1(data):		#Lecturas de los encoders.
    global encoder1
    encoder1 = data.data

def callbackEnc2(data):
    global encoder2
    encoder2 = data.data

def callbackVolt(data):		#Lectura de la tensión.
    global volt
    volt = data.data
    rospy.loginfo("** Battery Volt %s", data.data)	

def callback(data):		#Lectura del código con la información de todos los receptores IR.
    global codigo
    rospy.loginfo("** IR data %s", bin(data.data))	#Función para ver en pantalla los mensajes recibidos. 

    
    codigo = data.data			#data.data es un binario de 9 bits.
    code = 0b1000000000000 | data.data	#se añade un 1 delante para tener siempre el mismo tamaño (ahora 3*4+1=13 bits).


## Orden Onda emisores: Derecho - Central - Izquierdo
##CODIGO EN EL SKECTH DE ARDUINO:
	#//Puerto D, que incluye los pines digitales 2, 3, 4 y 5 del Arduino UNO. (B00111100)
        #                //Receptor IR Derecho           -> pin 2  (B00000100)		"visto desde el robot"
        #                //Receptor IR Central Derecho   -> pin 3  (B00001000)
        #                //Receptor IR Central Izquierdo -> pin 4  (B00010000)
        #                //Receptor IR Izquierdo         -> pin 5  (B00100000)

##En las siguientes líneas se guardan las lecturas de cada sensor en un vector separado, tomadas del número binario de 13 bits.
##Comienza en la posición 3 porque la 0,1 y 2 son las correspontienes a 0b1 del numero binario recibido, parte que no nos interesa.
    codeL[0]= bin(code)[3]		
    codeL[1]= bin(code)[7]
    codeL[2]= bin(code)[11]

    codeLC[0]= bin(code)[4]		
    codeLC[1]= bin(code)[8]
    codeLC[2]= bin(code)[12]

    codeRC[0]= bin(code)[5]		
    codeRC[1]= bin(code)[9]
    codeRC[2]= bin(code)[13]

    codeR[0]= bin(code)[6]		
    codeR[1]= bin(code)[10]
    codeR[2]= bin(code)[14]



def listener():


    rospy.init_node('listener', anonymous=True)					#Se inicializa el nodo.

    
##Se establecen los tópicos en los que se publican y el tipo de mensaje que se envía desde este nodo. En este caso, las dos velocidades enviadas a la controladora de los motores.

    pub1 = rospy.Publisher('ArduinoMotor/speed1', Int32, queue_size=10)		
    pub2 = rospy.Publisher('ArduinoMotor/speed2', Int32, queue_size=10)

##Se establecen los tópicos a los que se subscribe este nodo, correspondientes a los declarados en las funciones callback anteriores.

    rospy.Subscriber("ArduinoIR/codigoIR", Int32, callback)
    rospy.Subscriber("ArduinoMotor/sensorRFC", Int32, callbackC)
    rospy.Subscriber("ArduinoMotor/sensorRFR", Int32, callbackR)
    rospy.Subscriber("ArduinoMotor/sensorRFL", Int32, callbackL)

    rospy.Subscriber("ArduinoMotor/encoder1", Int32, callbackEnc1)
    rospy.Subscriber("ArduinoMotor/encoder2", Int32, callbackEnc2)
    rospy.Subscriber("ArduinoMotor/batteryVolt", Int32, callbackVolt)

    lost=1				#Variable que indica el estado del robot.
    recargar=1				#Indica si necesita recarga de la batería.
    rate = rospy.Rate(10) 		#Indica la tasa de refresco en Hz.
    

    while not rospy.is_shutdown():	#Mientras el nodo esté funcionando.

	time.sleep(1) #Pausa en segundos.

    	print "---Distancia C:       %s" % distanciaC		#Se imprimen en pantalla algunos de los mensajes recibidos.
    	print "---Distancia R:       %s" % distanciaR
    	print "---Distancia L:       %s" % distanciaL
	print "---CODE-----:      %s" % code
	print "---CODIGO---:	  %s" % bin(codigo)
	print "---Battery--:      %s" %volt

##Estado en búsqueda de señales IR, sigue trayectorias evitándo obtáculos según las distancias detectadas por los sensores ultrasónicos.


	if lost==1:
		rate = rospy.Rate(10)

		print "			--LOST-- "
		
		if distanciaC < 40 and distanciaR < 40 and distanciaL < 40:
			x=-3
			y=1
    			pub1.publish(x)				#Publicación de las velocidades. "x" es el avance, "y" es el giro.
			pub2.publish(y)			
		elif distanciaC < 20:
			if distanciaL < distanciaR:
				x=0
				y=5
	    			pub1.publish(x)
				pub2.publish(y) 
			else:
				x=0
				y=-5
		    		pub1.publish(x)
				pub2.publish(y)
		elif distanciaL < 20:
			x=0
			y=5
	    		pub1.publish(x)
			pub2.publish(y)

		elif distanciaL < 40:
			x=5
			y=3
	    		pub1.publish(x)
			pub2.publish(y)
		elif distanciaR < 20:
			x=0
			y=-5
	    		pub1.publish(x)
			pub2.publish(y)
		elif distanciaR < 40:
			x=5
			y=-3
	    		pub1.publish(x)
			pub2.publish(y)
		else:
			x=25
			y=0
	    		pub1.publish(x)
			pub2.publish(y)
		rate.sleep()
		
		if codigo != 4096 and recargar == 1:  #4096 corresponde al número binario del mensaje cuando no se recibe ninguna señal IR (Un 1 seguido de 12 ceros). Si se recibe alguna señal y recargar es igual a 1, se pasa al estado 0. (1000000000000 = bin(4096)).
			lost=0
			giro=0
			girando=0
			

	    	print "------Speed x:       %s" % x
	    	print "------Speed y:       %s" % y	

##Estado Localizado Región Lejana. Tras detectar algunas de las señales IR, se analiza cúal es y se realizan los movimientos necesarios para el acercamiento a la zona central cercana.
			
	if lost==0:

		print "			-- Localized --"
		rate = rospy.Rate(10)
	
		if distanciaC < 15 or distanciaR < 15 or distanciaL < 15:	#Si aparece algún obstáculo en la zona alrededor de la estación a menos de 15 cm del robot, espera hasta que desaparezca.
			x=0
			y=0
    			pub1.publish(x)
			pub2.publish(y)

		elif codeRC[0] == '1' and codeLC[2]=='0':   	#Según las señales detectadas, realiza un avance con un giro u otro.
			girando=0
			giro=1 	
			if codeRC[1]=='1' or codeLC[1]=='1':	#Si se encuentra en la región cercana a la base, se pasa al siguiente estado. Igual si ocurre en las siguientes sentencias elif.	
				lost=2
			else:
				x=15
				y=5	
    			pub1.publish(x)
			pub2.publish(y)
		
		elif codeRC[0] == '0' and codeLC[2]=='1':
			girando=0
			giro=2
			if codeRC[1]=='1' or codeLC[1]=='1':	
				lost=2
			else:
				x=15
				y=-5
    			pub1.publish(x)
			pub2.publish(y)
		elif codeRC[0] == '1' and codeLC[2]=='1':
			girando=0
			giro=0
			if codeRC[1]=='1' or codeLC[1]=='1':	
				lost=2
			else:
				x=10
				y=0		 	
    			pub1.publish(x)
			pub2.publish(y)

		elif codeR[0] == '1' and codeR[2]=='1':   	#Zona central detectada por el sensor derecho, realiza un giro.		
			girando=1
			encoder_inicio= encoder1
			x=0
			y=5
    			pub1.publish(x)
			pub2.publish(y)
		
		elif codeL[0] == '1' and codeL[2]=='1':   	#Zona central detectada por el sensor izquierdo, realiza el giro opuesto.
			girando=1
			encoder_inicio = encoder1
			x=0
			y=-5
    			pub1.publish(x)
			pub2.publish(y)


		elif codeR[2] == '1':				#Sensor derecho recibe de sólo un emisor (regiones laterales externas)   			
			x=10
			y=3
    			pub1.publish(x)
			pub2.publish(y)
		
		elif codeL[0] == '1':			   	#Sensor izquierdo recibe de sólo un emisor.
			x=10
			y=-3
    			pub1.publish(x)
			pub2.publish(y)

	
		rate.sleep()
		if codigo==4096 and girando==0:			#Si se deja de recibir alguna señal IR, se vuelve al estado inicial.
			lost=1
		if codigo==4096 and girando==1 and (abs(encoder_inicio-encoder1) > 550):	#Esto evita entrar en un bucle en el caso de que se reciba una señal por uno de los lados, se ponga a girar y no vuelva a recibir ninguna señal que haga que salga de ese estado.
			lost=1

	    	print "------Speed x:       %s" % x
	    	print "------Speed y:       %s" % y	
		
		tension = volt		#Guarda en la variable "tension" el valor de la tensión de las baterías para usarlo en el estado siguiente.
	
##Estado Localizado Región Cercana. Se encuentra en la zona central y ha detectado la señald del emisor de región cercana.

	if lost == 2:

		print "			-- 				Localized ZONA CERCANA CENTRAL--"
		rate = rospy.Rate(10)


		if codeRC[0] == '1' and codeLC[0] == '1':   
			girando=0
			giro=1 		
			x=6 
			y=0 
    			pub1.publish(x)
			pub2.publish(y)

		elif codeRC[0] == '1':   
			girando=0
			giro=1 		
			x=6
			y=3 
    			pub1.publish(x)
			pub2.publish(y)
		elif codeLC[0] == '1':   
			girando=0
			giro=1 		
			x=6 
			y=-3 
    			pub1.publish(x)
			pub2.publish(y)

		if volt > (tension + 2):	#Si el voltímetro detecta una diferencia de tensión entre el estado anterior y la lectura actual, significa que se ha realizado la conexión correctamente. Se pasa al siguiente estado.
			lost=3
			tiempo_recarga=0	#Se iguala a 0 el tiempo de la recarga.
		else:
			rate.sleep()        	
	

##Estado de carga y desconexión. 

	
	if lost == 3:					

		rate = rospy.Rate(10)
		
		print "			--   RECARGA!!--"
		x=0
		y=0
    		pub1.publish(x)			#Se paran los motores.
		pub2.publish(y)
		rate.sleep()
		time.sleep(10) 

		tiempo_recarga += 1
		
		if recargar == 0 or tiempo_recarga >= 2:	#Se realiza un bucle para controlar el tiempo durante el que se realiza la recarga.
			
			print "			-- DESCONEXIÓN!!--"
			x=-10				#Movimiento hacia atrás de desconexión.
			y=0
    			pub1.publish(x)
			pub2.publish(y)
			rate.sleep()
			time.sleep(4) 


			x=0				#Giro sobre sí mismo para seguir su camino.
			y=10
    			pub1.publish(x)
			pub2.publish(y)
			rate.sleep()
			time.sleep(5) 
			
			lost=1				#Vuelve al estado inicial.
			recargar = 0			#Se establece igual a 0 para que no vuelva a buscar las señales IR hasta que se indique lo contrario.
			


    rospy.spin()


if __name__ == '__main__':
    listener()

