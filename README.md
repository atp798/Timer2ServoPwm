# Timer2ServoPwm
Arduino servo driver library base on timer2

The servo driver function is compatible with official servo library.
Besides, this library provides a not very accurate pwm function on any pin.
The main point of this library is to use timer2 as servo driver timer 
interrupt source when timer1 has been used by other libraries.