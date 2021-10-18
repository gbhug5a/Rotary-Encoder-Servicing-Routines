/*
Alternate lookup table method - one pin-change interrupt enabled at a time.

This application demonstrates a pin-change interrupt servicing routine
for a rotary encoder.  This code specifically applies to Arduinos using the
Atmega328P microcontroller, including the Uno, Nano and Pro Mini.

A pin change interrupt is enabled on only one pin at a time.  When an interrupt
is triggered on that pin, we immediately switch the interrupt to the other pin.
So any bouncing which may occur on the first pin will not trigger any interrupts.
That can reduce the number of interrupts that must be serviced.

However, this causes a problem when the direction changes because we are watching
the wrong pin.  The solution is to include a fifth bit in the lookup table which is
the indentity of the interrupting pin.  With the fifth bit, the lookup table is
expanded to 32 bytes.

The code also replaces the value of the interrupting pin read from the port with
the value we know it must have had when generating the interrupt.  This eliminates
false readings caused by bouncing during the delay which always exists between
triggering the interrupt and reading the port.

With the appropriate "encoderType" definition, this method will work for encoders with
the same number of pulses as detents per revolution (Type 0), as well as for those
with half as many pulses as detents (Type 1).  In either case, the code produces
one tick per detent.  For Type 0, that is only when both switches are open.
For Type 1 encoders, switches can be either both open or both closed at a detent.

The encoder pins are connected to D4 and D5, but any two pins can be used so long
as both are on the same port.  The code displays the cumulative value of the encoder
on the serial monitor at each detent, followed by the number of interrupts serviced
since the last detent, which ideally would be "4" for a Type 0 encoder, and "2" for
a Type 1 - one interrupt for each transition, with no interrupts generated for bounces.

Pin change interrupts are used. In the ISR, direct register access is used to read
the port and manage the interrupt enables.
*/

const int aPIN = 4;                     // encoder pins
const int bPIN = 5;

/*  This section defines the port being used for the encoder pins, along with the PCI registers
    and interrupt vector.  The values shown here are for Port D (D0 - D7), but Port B (D8 - D13)
    or Port C(A0 - A5) could also be used.
*/

#define pinZero 0                       // data pin name of PD0 (0). PB0 (8) PC0 (A0) for ports B and C
#define PORT PIND                       // port input register (port D includes D4 and D5)
#define portVECT PCINT2_vect            // The ISR vector for port D pin change interrupt
#define portINT PCIE2                   // used to enable pin change interrupts on port D as a whole
#define ENABLE PCMSK2                   // individual pin interrupt enabled = 1
#define FLAG PCIF2                      // only one flag for whole port

const byte encoderType = 0;             // encoder with equal # of detents & pulses per rev
//const byte encoderType = 1;             // encoder with  pulses = detents/2. pick one, commment out the other

const int THRESH =(4-(2*encoderType));  // transitions needed to recognize a tick - type 0 = 4, type 1 = 2

const byte CWPIN = bit(aPIN - pinZero); // bit value for switch that leads on CW rotation
const byte CCWPIN = bit(bPIN - pinZero);// bit value for switch that leads on CCW rotation
const byte PINS = CWPIN + CCWPIN;       // sum of bit values of the encoder pins
const byte ZEERO = 0x80;                // "byte" data type doesn't do negative, so make 128 = zero

byte EDGE;                              // the edge direction of the next pin change interrupt
byte CURRENT;                           // the current state of the switches
byte TOTAL = ZEERO;                     // accumulated transitions since last tick (0x80 = none)
byte INDEX = 0;                         // Index into lookup state table
byte NUMINTS = 0;                       // # interrupts since last detent
int Setting = 0;                        // current accumulated value set by rotary encoder

volatile byte tickArray[256];           // circular buffer of ticks and interrupts per tick/detent
volatile byte beginTICK = 0;            // pointers to beginning and ending of circular buffer
volatile byte endTICK = 0;

// The table is now 32 bytes long so as to include the identity of the pin currently interrupting.
// The +2 and -2 entries are for a change of direction.

int ENCTABLE[]  = {0,1,0,-2,-1,0,2,0,0,2,0,-1,-2,0,1,0,0,0,-1,2,0,0,-2,1,1,-2,0,0,2,-1,0,0};

void setup() {

  Serial.begin(115200);
  pinMode(aPIN, INPUT);                 // set up encoder pins as INPUT.  Assumes external 10K pullups
  pinMode(bPIN, INPUT);

  EDGE = PINS;                          // identifies next pin-change interrupt as falling or rising
                                        //    assume current state is low, so any change will be rising
  if(PORT & PINS) {                     // but if actual current state is already high,
    EDGE = 0;                           //    make EDGE low
    INDEX = 3;                          //    and make "current" bits of INDEX match the current high state
  }

  ENABLE |= CWPIN;                      // enable only CWPIN interrupt in mask register
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

/* Time passes between the interrupt and the reading of the port.  So if there is bouncing,
the read value of the interrupting pin may be wrong.  But we know it must be the EDGE state.
So in CURRENT, we clear the bit that just caused the interrupt, and replace it with the EDGE
value. The non-interrupting pin is assumed to be stable, with a valid read. */

  CURRENT &= ~ENABLE;                   // clear the bit that just caused the interrupt
  CURRENT |= (EDGE & ENABLE);           // OR the EDGE value back in

  INDEX     = INDEX << 2;               // Shift previous state left 2 bits (0 in)
  if(CURRENT & CWPIN) bitSet(INDEX,0);  // If CW is high, set INDEX bit 0
  if(CURRENT & CCWPIN) bitSet(INDEX,1); // If CCW is high, set INDEX bit 1
  INDEX &= 15;                          // Mask out all but prev and current.  bit 4 now zero
  if(ENABLE & CCWPIN) bitSet(INDEX,4);  // if CCWPIN is the current enabled interrupt, set bit 4

// INDEX is now a five-bit index into the 32-byte ENCTABLE state table.

  TOTAL += ENCTABLE[INDEX];             // Accumulate transitions

  NUMINTS++;                            // number of interrupts - just for this demo - not normally needed

  if((CURRENT == PINS) || ((CURRENT == 0) && encoderType)) {  // A valid tick can occur only at a detent

// If we have a valid number of TOTAL transitions at a detent, add the tick direction and number
// of interrupts to the print buffer.  The MAIN loop will update the Setting total and print everything.

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
    TOTAL = ZEERO;                      // Always reset TOTAL to 0x80 at detent
  }

  if(CURRENT == EDGE) EDGE ^= PINS;     // having reached EDGE state, now switch EDGE to opposite
  ENABLE ^= PINS;                       // switch interrupt to other pin
  PCIFR |= bit(FLAG);                   // clear flag if any by writing a 1
}                                       // end of ISR - interrupts automatically re-enabled
