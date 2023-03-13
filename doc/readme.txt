Firmware für I2C Servo Controller

Diese Firmware wird unter der GPL Lizenz veröffentlicht.
http://www.gnu.de/documents/gpl.de.html

Der Servo Controller ermöglicht die Ansteuerung von bis zu 10 Modellbau-Servos
über einen I2C Bus. Er erzeugt forlaufend PWM Steuersignale, deren Pulsbreiten
durch die I2C Schnittstelle einstellbar ist. Die Stromversorgung und 
Schnittstelle zum Mikrocontroller wird mit 3,3 - 5,5 Volt betrieben. Die 
Servo-Motoren können mit einer höheren Spannung betrieben werden, 
normalerweise 6 Volt.

Die Firmware liegt in zwei Varianten vor, nämlich eine für den ATtiny26 und 
eine für den ATtiny2313. Beide Controller werden mit 4Mhz (interner) R/C
Oszillator getaktet.

Die Leuchtdiode blinkt beim Einschalten der Stromversorgung einmal. Danach
blitzt sie jedesmal kurz, wenn der Chip über den I2C Bus mit korrekter
Adresse angesprochen wird.

Die I2C Kommunikation sieht so aus:

Sende START
Sende Adresse 0x1E (entspricht 7bit Adresse 0001111 und R/W bit=0)
Empfange ACK
Sende 10 Daten-Bytes für die Ausgänge 0 bis 9.
Empfange jeweils ein ACK
Sende STOP

Die Daten-Bytes bestimmen die Positionen der Servos. Die Werte 
liegen normalerweise im Bereich 62-125 für Impulse von 1-2ms.
Der Wert 94 erzeugt 1,5ms Impulse, was der mittleren Position entspricht.
Der spezielle Wert 0 schaltet das PWM Signal aus, dann geht
der Servo in den Standby Betrieb. Jeder Servo reagiert etwas anders,
darum müssen Sie die exakten Werte selbst herausfinden.

Durch Beschaltung der Adress-Eingänge kann man die unteren Bits der 
7bit Adresse konfigurieren. Unbeschaltete Adress-Eingänge liegen auf high.

Parallel zur Stromversorgung sollte man noch einen 100nF Kondensator
und einen 10yF Kondensator schalten. Beachte, dass Servos Motoren unter 
Umständen so viel Strom verbrauchen, dass die Ausgangsspannung der 
Batterien instabil wird. Zur Stromversorgung der Elektronik dient 
daher im Idealfall eine separate Batterie (Empfänger-Akku).

Der Reset Eingang des Mikrocontrollers kann in der Regel unbeschaltet
bleiben.

Anschlussbelegung von Servos:
    GND = schwarz oder braun
    Batterie = rot
    Signal = orange, gelb oder weiss

Anschlußbelegung der Kabel von Lego NXT Computern:
    rot = GND (das ist kein Irrtum!)
    grün = VCC
    gelb = SCL
    blau = SDA
    weiss = nicht verwenden
    schwarz = nicht verwenden

Bei Lego Mindstorms müssen die beiden Pull-Up Widerstände an SCL und SDA auf
82k Ohm erhöht werden.

