## Rotary-Encoder-Servicing-Routines

### NEW - November, 2020

A new "Arduino" folder has been added for Arduinos using the ATMEGA328P
controller.  It includes the following demonstration sketches:

 - The standard state-machine lookup-table servicing routine with
   pin-change interrupts enabled on both pins.

 - An alternate low-overhead version which enables only one interrupt
   at a time, and which may lead to servicing fewer interrupts.

Both sketches send to the Serial Monitor the total number of interrupts
which have occurred between detents.  This allows comparing the performance
of the standard and alternate methods, and the sketch for the standard
method can be used to compare the bounciness of rotary encoders.

A new PDF has been added discussing the Arduino sketches.


### Original - Polling and pin-triggered interrupt servicing routines
### for quadrature rotary encoders

These routines are efficient and work well with no hardware debouncing.
Encoders with pulses=detents, and pulses=detents/2, are supported.
Examples in assembler are included for the TI MSP430 processors.
Everything is explained in the PDF.  The TI code was updated in
November, 2020, to fix a bug.

A second PDF was added describing a polling routine for reading
multiple push-button switches.

