# SONAR_Scanner_HC-SR04_ATmega328P
Implementation of the HC-SR04 Ultrasonic Sensor with the ATmega328P microcontroller, in C.

The overall aim of this project is to create a SONAR scanner by interfacing an ultrasonic sensor with the ATmega328P. 
The ultrasonic sensor will be attached to a servo motor which can rotate 180 degrees. 
The servo will rotate, and at each angle of rotation the ultrasonic sensor will take a measurement. 
The sensor will record the distances of nearby objects. 
The measurement will be this recorded distance. 
The recorded distance, and its corresponding angle, will be output to the Data Visualiser terminal in Atmel Studio 7 via USART.
Additionally, an LCD screen will geometrically display the distances from the objects.
It will display a “bar” – the longer the bar, the greater the distance between the object and the sensor.
Thus, this project will implement a SONAR scanner.
