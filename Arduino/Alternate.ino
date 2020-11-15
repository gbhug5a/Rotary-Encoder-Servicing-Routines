/*
Alternate lookup table method - one pin-change interrupt enabled at a time.

This application demonstrates a pin-triggered interrupt servicing routine
for a rotary encoder.  This code specifically applies to Arduinos using the
Atmega328P microcontroller, including the Uno, Nano and Pro Mini.

An interrupt is enabled on only one pin at a time.  When an interrupt
is triggered on that pin, we immediately switch the interrupt to the other pin.
So any bouncing which may occur on the first pin will not trigger any interrupts.
That can reduce the number of interrupts that must be serviced.

With the appropriate "encoderType" definition, this will work for encoders with
the same number of pulses as detents per revolution (Type 0), as well as for those
with half as many pulses as detents (Type 1).  In either case, the code produces
one tick per detent.  For Type 0, that is only when both switches are open.
For Type 1 encoders, switches can be either both open or both closed at a detent.

The encoder pins are connected to D4 and D5, but any two pins can be used so long
as both are on the same port.  The code flashes an LED on A4 for every clockwise
detent and an LED on A5 for every counter-clockwise detent.  The serial monitor
will show the total number of interrupts which have been serviced between detents,
which ideally would be 4 for a Type 0 encoder, and 2 for a Type 1.

Pin change interrupts are used. In the ISR, direct register access is used to read
the port and manage the interrupt enables.
*/

const int aPIN = 4;                     // encoder pins
const int bPIN = 5;
const int CWLED = A1;                   // LED pins
const int CCWLED = A2;
const int LEDdelay = 8;                 // blink time of LEDs - ms

#define PORT PIND                       // port input register (port D includes D4 and D5)
#define portVECT PCINT2_vect            // The ISR vector for port D pin change interrupt
#define portINT PCIE2                   // enable pin change interrupts on port D
#define ENABLE PCMSK2                   // individual pin interrupt enabled = 1
#define FLAG PCIF2                      // only one flag for whole port

const byte encoderType = 0;             // encoder with equal # of detents & pulses per rev
//const byte encoderType = 1;             // encoder with  pulses = detents/2. pick one, commment out the other

const int THRESH =(4-(2*encoderType));  // transitions needed to recognize a tick - type 0 = 4, type 1 = 2

const byte CWPIN = bit(aPIN);           // bit value for switch that leads on CW rotation
const byte CCWPIN = bit(bPIN);          // bit value for switch that leads on CCW rotation
const byte PINS = CWPIN + CCWPIN;       // sum of bit values of the encoder pins
const byte ZEERO = 0x80;                // byte data type doesn't do negative

byte EDGE;                              // the edge direction of the next pin change interrupt
byte CURRENT;                           // the current state of the switches
byte TOTAL = ZEERO;                     // accumulated transitions since last tick (8 = none)
byte INDEX = 0;                         // Index into lookup state table
byte numPrints = 0;                     // # items printed on line
volatile byte NUMINTS = 0;              // # interrupts since last detent
volatile byte intArray[256];            // circular buffer of interrupts per detent/tick
volatile byte beginINT = 0;
volatile byte endINT = 0;
volatile byte tickArray[256];           // circular buffer of ticks
volatile byte beginTICK = 0;
volatile byte endTICK = 0;

// The table is now 32 bytes long so as to include the identity of the pin currently interrupting

int ENCTABLE[]  = {0,1,0,-2,-1,0,2,0,0,2,0,-1,-2,0,1,0,0,0,-1,2,0,0,-2,1,1,-2,0,0,2,-1,0,0};

byte tickTime;                          //  countdown timers for LEDs
unsigned long newMilli;
unsigned long oldMilli = 0;

void setup() {

  Serial.begin(9600);

  pinMode(CWLED, OUTPUT);               // set up LED pins - LOW = off
  digitalWrite(CWLED, LOW);
  pinMode(CCWLED, OUTPUT);
  digitalWrite(CCWLED, LOW);

  pinMode(aPIN, INPUT_PULLUP);          // set up encoder pins as INPUT-PULLUP
  pinMode(bPIN, INPUT_PULLUP);

  EDGE = PINS;                          // identifies current pin-change interrupt as falling or rising
                                        //    assume current state is low, so any change will be rising
  if(PORT & PINS) {                     // but if actual current state is high, make EDGE low
    EDGE = 0;
    INDEX = 3;                          //    and make "current" bits of INDEX match the current state
  }

  ENABLE |= CWPIN;                      // enable only CWPIN interrupt in mask register
  PCIFR |= bit(FLAG);                   // clear interrupt flag if any
  PCICR |= bit(portINT);                // enable interrupts on Port D
}

void loop() {

  if(beginINT != endINT) {              // serial print number of interrupts
    beginINT++;
    Serial.print(intArray[beginINT]); Serial.print(" ");
    numPrints++;
    if(numPrints == 20) Serial.println(), numPrints = 0;
  }

  if((beginTICK != endTICK) && (tickTime == 0)) {

    beginTICK++;
    
    if(tickArray[beginTICK] == 1) {     // flash LEDs as needed
      digitalWrite(CWLED,HIGH);
      tickTime = LEDdelay;
    }
    else if(tickArray[beginTICK] == 0xFF) {
      digitalWrite(CCWLED,HIGH);
      tickTime = LEDdelay;
    }
  }

  newMilli = millis();                  // countdown LED
  if (newMilli > oldMilli) {
    oldMilli = newMilli;
    if (tickTime) {
      tickTime--;
      if (tickTime == 0) {
        digitalWrite (CWLED, LOW);
        digitalWrite (CCWLED, LOW);
      }
    }
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

  NUMINTS++;                            // just for this demo - not normally needed

// INDEX is now a five-bit index into the 32-byte ENCTABLE state table.

  TOTAL += ENCTABLE[INDEX];             //Accumulate transitions

  if((CURRENT == PINS) || ((CURRENT == 0) && encoderType)) {  //A valid tick can occur only at a detent

    if(TOTAL == (ZEERO + THRESH)) {
      endTICK++;
      tickArray[endTICK] = 1;

      endINT++;
      intArray[endINT] = NUMINTS;
      NUMINTS = 0;
    }

    else if(TOTAL == (ZEERO - THRESH)) {
      endTICK++;
      tickArray[endTICK] = 0xFF;

      endINT++;
      intArray[endINT] = NUMINTS;
      NUMINTS = 0;
    }
    TOTAL = ZEERO;                          //Always reset TOTAL to 0x80 at detent
  }

  if(CURRENT == EDGE) EDGE ^= PINS;     // having reached EDGE state, now switch EDGE to opposite
  ENABLE ^= PINS;                       // switch interrupt to other pin
  PCIFR |= bit(FLAG);                   // clear flag if any by writing a 1
}                                       // end of ISR - interrupts automatically re-enabled
