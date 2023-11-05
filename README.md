Arduino-HC SR04 - 3D NAVIGATION SYSTEM FOR BLIND USING VIBRATIONS
==================

Arduino Based Navigation System that generates Vibration Patterns on Forehead indicating objects around it in a 3d space. Uses 4 Ultrasonic Sensor to detect objects at different angles.
The algorithm generates vibrations intensity based on distance of the object from the user.

Current Features
----------------
* Senses all the objects around 180 degree space upto 3 meters, can be increasd till 5 meters.
* Detects and Produce signals for multiple Objects.
* Combitionational detection ALgorithm can detect large objects sensed in multiple sensors and classify as a single object.
* Very responsive, with high response time rates.
* Uses Arduino based microcontrollers with low frequencies. Requires low computation power and is optimised. 

Default pinout for the ATTiny85
-------------------------------
The code is pre-configured for use with Arduino Pro Mini. It should also run on any arduino board, but the pin numbers will likely need to be modified. The pin configurations can also be viewed the circuit Diiagram
Provided.

* Trigger of Sensor 1 -> Pin 3
* Echo of Sensor 1    -> Pin 4
* Trigger of Sensor 2 -> Pin 5
* Echo of Sensor 2    -> Pin 6
* Trigger of Sensor 3 -> Pin 7
* Echo of Sensor 3    -> Pin 8
* Trigger of Sensor 4 -> Pin 9
* Echo of Sensor 4    -> Pin 10

Pins For Motors
#define V1 11
#define V2 12
#define V3 A0
#define V4 A1
#define V5 A2
