/*
Standard lookup table method - pin-change interrupts enabled.

This application demonstrates a pin-triggered interrupt servicing routine
for a rotary encoder.  This code specifically applies to Arduinos using the
Atmega328P microcontroller, including the Uno, Nano and Pro Mini.

Interrupts are enabled on both pins.  When either pin triggers an interrupt, the
port is read, and the change from the previous state is interpreted through a
lookup table.  So all the bouncing on both pins triggers interrupts.

With the appropriate "encoderType" definition, this will work for encoders with
the same number of pulses as detents per revolution (Type 0), as well as for those
with half as many pulses as detents (Type 1).  In either case, the code produces
one tick per detent.  For Type 0, that is only when both switches are open.
For Type 1 encoders, switches can be either both open or both closed at a detent.

The encoder pins are connected to D4 and D5, but any two pins can be used so long
as both are on the same port. The serial monitor will show the cumullative value
of the encoder at each detent, fowwoed by the total number of interrupts which have
been serviced since the last detent, which ideally would be 4 for a Type 0 encoder,
and 2 for a Type 1.

Pin change interrupts are used. In the ISR, direct register access is used to read
the port and manage the interrupt enables.
*/
                                           
const int aPIN = 4;                     // encoder pins
const int bPIN = 5;

/*  This section defines the port being used for the encoder pins, along with the PCI registers
    and interrupt vector.  The values shown here are for Port D (D0 - D7), but Port B (D8 - D13)
    or Port C(A0 - A5) could also be used.
*/

#define pinZero 0                       // data pin name of PB0 (8), PC0 (A0) or PD0 (0)
#define PORT PIND                       // port input register (port D includes D4 and D5)
#define portVECT PCINT2_vect            // The ISR vector for port D pin change interrupt
#define portINT PCIE2                   // enable pin change interrupts on port D
#define ENABLE PCMSK2                   // individual pin interrupt enabled = 1
#define FLAG PCIF2                      // only one flag for whole port

const byte encoderType = 0;             // encoder with equal # of detents & pulses per rev
//const byte encoderType = 1;             // encoder with  pulses = detents/2. pick one, commment out the other

const int THRESH =(4-(2*encoderType));  // transitions needed to recognize a tick - type 0 = 4, type 1 = 2

const byte CWPIN = bit(aPIN - pinZero);  // bit value for switch that leads on CW rotation
const byte CCWPIN = bit(bPIN - pinZero); // bit value for switch that leads on CCW rotation
const byte PINS = CWPIN + CCWPIN;       // sum of bit values of the encoder pins
const byte ZEERO = 0x80;                // byte data type doesn't do negative

byte CURRENT;                           // the current state of the switches
byte TOTAL = ZEERO;                     // accumulated transitions since last tick (0x80 = none)
byte INDEX = 0;                         // Index into lookup state table
byte NUMINTS = 0;                       // # interrupts since last detent
int Setting = 0;                        // current accumulated value set by rotary encoder

volatile byte tickArray[256];           // circular buffer of ticks
volatile byte beginTICK = 0;
volatile byte endTICK = 0;

   // Encoder state table - there are 16 possible transitions between interrupts

int ENCTABLE[]  = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

void setup() {

  Serial.begin(115200);

  pinMode(aPIN, INPUT);                 // set up encoder pins as INPUT. Assumes external 10K pullups
  pinMode(bPIN, INPUT);

  if(PORT & PINS) INDEX = 3;            // Initialize INDEX to current state
  
  ENABLE |= PINS;                       // enable interrupts on both pins in mask register
  PCIFR |= bit(FLAG);                   // clear interrupt flag if any
  PCICR |= bit(portINT);                // enable interrupts on Port D
}

void loop() {

  if(beginTICK != endTICK) {            // if anything in circular buffer

    beginTICK++;
    if(tickArray[beginTICK] == 1) Setting ++; // print Setting
    else Setting--;

    Serial.print(Setting); Serial.print(" ");
    beginTICK++;
    Serial.println(tickArray[beginTICK]); // print number of interrupts since last detent
  }
}

ISR (portVECT) {                        // pin change interrupt service routine. interrupts
                                        //     automatically disabled during execution

  CURRENT = PORT & PINS;                // read the entire port, mask out all but our pins
  INDEX     = INDEX << 2;               // Shift previous state left 2 bits (0 in)
  if(CURRENT & CWPIN) bitSet(INDEX,0);  // If CWPIN is high, set INDEX bit 0
  if(CURRENT & CCWPIN) bitSet(INDEX,1); // If CCWPIN is high, set INDEX bit 1
  INDEX &= 15;                          // Mask out all but prev and current

// INDEX is now a four-bit index into the 16-byte ENCTABLE state table

  TOTAL += ENCTABLE[INDEX];             //Accumulate transitions

  NUMINTS++;                            // just for this demo - not normally needed

  if((CURRENT == PINS) || ((CURRENT == 0) && encoderType)) {  // A valid tick can occur only at a detent

    if(TOTAL == (ZEERO + THRESH)) {
      endTICK++;
      tickArray[endTICK] = 1;

      endTICK++;
      tickArray[endTICK] = NUMINTS;
      NUMINTS = 0;
    }

    else if(TOTAL == (ZEERO - THRESH)) {
      endTICK++;
      tickArray[endTICK] = 0xFF;

      endTICK++;
      tickArray[endTICK] = NUMINTS;
      NUMINTS = 0;
    }
    TOTAL = ZEERO;                          //Always reset TOTAL to 0x80 at detent
  }
}
