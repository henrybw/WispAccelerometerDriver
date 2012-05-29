WISP ADXL346 accelerometer driver
=================================

Driver and simple public API for interacting with the ADXL346 accelerometer. Abstracts all the device-specific details and sets up an interface between the [WISP](http://sensor.cs.washington.edu/WISP.html) and the ADXL346 accelerometer, exposing the accelerometer data through the public API.

Currently this project is in prototyping stage, targeting the MSP430G2553 (the processor on-board the LaunchPad).

Where to look for stuff
-----------------------

Important stuff:

* `accel.h`: Declarations for the public API.
* `accel.c`: Implementation of the public API and accelerometer driver.

Less important stuff:

* `accel_registers.h`: All the ADXL346 registers declared as a constants. (These are generated from accel_registers.txt, which was in turn copied/pasted from the ADXL346 data sheet.)
* `main.c`: Simple test program for exercising the accelerometer API.

Notes
-----
The indentation follows Justin Reina's coding style, and thus while appearing fine and dandy in Code Composer by default (provided that your monitor is wide enough), it looks a little wonky if you view it directly on GitHub's repo browser. For the record, the standard WISP code layout conventions are:

* 135 character lines
* Tab characters (not spaces)
* Tab width of 4 spaces
