Based on http://nsd.dyndns.org/speech/ by Nicolas S Dade
Hooking up an SPO256 speech chip to an Arduino Uno without external crystal - using Uno for digital clock and reset

This is based on the setup from the above link but using no resistors, crystals, or capacitors except 
a small reversed cap from the output to the speaker for output decoupling.

Note: You MUST connect standby reset (pin 25) to reset (pin 2) to Arduino pin 10 for this to work. 
Without joining standby reset to reset it will keep waiting.
Also make sure you have Test (pin 22) to ground and VDI and SE (pins 23 and 19) to VDD (7) to power 
(I used the Arduino-supplied 5V power rail)
OSC1 (pin 27) is driven by the Arduino's clock through Arduino pin 11



Pin hookup:

Arduino :: SPO256
2 :: 18 (SPO A1)
3 :: 17 (SPO A2)
4 :: 16 (SPO A3)
5 :: 15 (SPO A4)
6 :: 14 (SPO A5)
7 :: 13 (SPO A6)
8 :: 20 (SPO ALD)
9 :: 9  (SPO LRQ)
10 :: 2 (SPO RESET), 25 (SPO SBY RST)
11 :: 27 (SPO OSC1)
Ground :: 1 (SPO GND), 22 (SPO TEST)
Power :: 23 (SPO VDI), 19 (SPO SE), 7 (SPO VDD)

To speaker :: 24 (SPO OUT) 



Next step is to add a proper op-amp to drive the speaker - this setup is weak and the oscilloscope
measured only 2V, it should have seen nearly the full 5V.

