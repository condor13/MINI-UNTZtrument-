// Simple UNTZtrument MIDI step sequencer.
// Requires an Arduino Leonardo w/TeeOnArdu config (or a PJRC Teensy),
// software on host computer for synth or to route to other devices.

#include <Wire.h>
#include "Adafruit_Trellis.h"
#include "Adafruit_UNTZtrument.h"

#define LED 13 // Pin for heartbeat LED (shows code is working)
#define N_POTS 4
#define encoder0PinA  3
#define encoder0PinB  4

int encoder0Pos = 50;
// A standard UNTZtrument has four Trellises in a 2x2 arrangement
// (8x8 buttons total).  addr[] is the I2C address of the upper left,
// upper right, lower left and lower right matrices, respectively,
// assuming an upright orientation, i.e. labels on board are in the
// normal reading direction.
#ifndef HELLA
Adafruit_Trellis     T[4];
Adafruit_UNTZtrument untztrument(&T[0], &T[1], &T[2], &T[3]);
const uint8_t        addr[] = { 0x70, 0x71, 0x72, 0x73 };
#else
Adafruit_Trellis     T[8];      // A HELLA UNTZtrument has eight Trellis boards...
Adafruit_UNTZtrument untztrument(&T[0], &T[1], &T[2], &T[3],
                                 &T[4], &T[5], &T[6], &T[7]);
const uint8_t        addr[] =   { 0x70, 0x71, 0x72, 0x73,
                                  0x74, 0x75, 0x76, 0x77
                                };
#endif // HELLA

#define WIDTH     ((sizeof(T) / sizeof(T[0])) * 2)
#define N_BUTTONS ((sizeof(T) / sizeof(T[0])) * 8)

// Encoder on pins 4,5 sets tempo.  Optional, not present in
// standard UNTZtrument, but won't affect things if unconnected.

byte MIDI_CLOCK = 0xf8; //send midi clock

// Pots
/*
  const int potPin[] = {0, 1, 2, 3 };  // Pot pins
  const uint8_t potCN[] = {0x0A, 0x0B, 0x0C, 0x0D };  // MIDI control values
  uint8_t potValues[N_POTS];  // Initial values
  uint8_t potValuePrev[] = {0, 0, 0, 0, 0}; // previous values for comparison
*/

int velocity = 100;//velocity of MIDI notes, must be between 0 and 127
int noteON = 144;//144 = 10010000 in binary, note on command
int noteOFF = 128;//128 = 10000000 in binary, note off command

uint8_t       grid[WIDTH];                 // Sequencer state
uint8_t       heart        = 0,            // Heartbeat LED counter
              col          = WIDTH - 1;    // Current column
unsigned int  bpm          = 240;          // Tempo
unsigned long beatInterval = 60000L / bpm, // ms/beat
              prevBeatTime = 0L,           // Column step timer
              prevReadTime = 0L;           // Keypad polling timer

int nota1 = 36;
int nota2 = 37;
int nota3 = 38;
int nota4 = 39;

// The note[] and channel[] tables are the MIDI note and channel numbers
// for to each row (top to bottom); they're specific to this application.
// bitmask[] is for efficient reading/writing bits to the grid[] array.
int note[4]    = {  encoder0Pos, 37, 38, 40  };
static const uint8_t PROGMEM
channel[4] = {   1,  1,  1,  1  },
             bitmask[4] = {   1,  2,  4,  8  };

void setup() {
  Serial.begin(31250);
  Serial.println("MIDI-seq Demo-v0.2");

  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);

  pinMode(LED, OUTPUT);
#ifndef HELLA
  untztrument.begin(addr[0], addr[1]);
#else
  untztrument.begin(addr[0], addr[1], addr[2], addr[3],
                    addr[4], addr[5], addr[6], addr[7]);
#endif // HELLA
  // Default Arduino I2C speed is 100 KHz, but the HT16K33 supports
  // 400 KHz.  We can force this for faster read & refresh, but may
  // break compatibility with other I2C devices...so be prepared to
  // comment this out, or save & restore value as needed.
#ifdef ARDUINO_ARCH_SAMD
  Wire.setClock(400000L);
#endif
#ifdef __AVR__
  //TWBR = 12; // 400 KHz I2C on 16 MHz AVR
#endif
  untztrument.clear();
  untztrument.writeDisplay();
  memset(grid, 0, sizeof(grid));
}

/*        MIDI USB CODE
  void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
  }
  void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
  }
*/

void doEncoder() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

//send MIDI message
void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.write(command);          //send note on or note off command
  Serial.write(MIDInote);         //send pitch data
  Serial.write(MIDIvelocity);     //send velocity data
}

// Turn on (or off) one column of the display
void line(uint8_t x, boolean set) {
  for (uint8_t mask = 1, y = 0; y < 8; y++, mask <<= 1) {
    uint8_t i = untztrument.xy2i(x, y);
    if (set || (grid[x] & mask)) untztrument.setLED(i);
    else                        untztrument.clrLED(i);
  }
}

void loop() {
  uint8_t       mask;
  boolean       refresh = false;
  unsigned long t       = millis();
  doEncoder();

  if ((t - prevReadTime) >= 20L) { // 20ms = min Trellis poll time
    if (untztrument.readSwitches()) { // Button state change?
      for (uint8_t i = 0; i < N_BUTTONS; i++) { // For each button...
        uint8_t x, y;
        untztrument.i2xy(i, &x, &y);
        mask = pgm_read_byte(&bitmask[y]);
        if (untztrument.justPressed(i)) {
          if (grid[x] & mask) { // Already set?  Turn off...
            grid[x] &= ~mask;
            untztrument.clrLED(i);
            MIDImessage(noteOFF, pgm_read_byte(&note[y]), 127); //turn note off
            //noteOff(pgm_read_byte(&channel[y]), pgm_read_byte(&note[y]), 127);     //This MIDIUSB
          } else { // Turn on
            grid[x] |= mask;
            untztrument.setLED(i);
          }
          refresh = true;
        }
      }
    }
    prevReadTime = t;
    digitalWrite(LED, ++heart & 32); // Blink = alive
  }

  if ((t - prevBeatTime) >= beatInterval) { // Next beat?
    // Turn off old column
    line(col, false);
    for (uint8_t row = 0, mask = 1; row < 8; row++, mask <<= 1) {
      if (grid[col] & mask) {
        //noteOff(pgm_read_byte(&channel[row]), pgm_read_byte(&note[row]), 127);
        MIDImessage(noteOFF, pgm_read_byte(&note[row]), 127); //turn note off
      }
    }
    // Advance column counter, wrap around
    if (++col >= WIDTH) col = 0;
    // Turn on new column
    line(col, true);
    for (uint8_t row = 0, mask = 1; row < 8; row++, mask <<= 1) {
      if (grid[col] & mask) {
        //noteOn(pgm_read_byte(&channel[row]), pgm_read_byte(&note[row]), 127);
        MIDImessage(noteON, pgm_read_byte(&note[row]), 127);
      }
    }
    prevBeatTime = t;
    refresh      = true;
    unsigned int bpm          = 240 / 4; // Div for encoder detents
    beatInterval = 60000L / bpm;
  }

  if (refresh) untztrument.writeDisplay();

}
