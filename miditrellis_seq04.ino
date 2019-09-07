// Simple UNTZtrument MIDI step sequencer.
// Requires an Arduino Leonardo w/TeeOnArdu config (or a PJRC Teensy),
// software on host computer for synth or to route to other devices.

#include <Wire.h>
#include <Adafruit_Trellis.h>
#include <Adafruit_UNTZtrument.h>
//#include "MIDIUSB.h"

int noteON = 144;//144 = 10010000 in binary, note on command
int noteOFF = 128;//128 = 10000000 in binary, note off command

#define LED 13 // Pin for heartbeat LED (shows code is working)

#ifndef HELLA
// A standard UNTZtrument has four Trellises in a 2x2 arrangement
// (8x8 buttons total).  addr[] is the I2C address of the upper left,
// upper right, lower left and lower right matrices, respectively,
// assuming an upright orientation, i.e. labels on board are in the
// normal reading direction.
Adafruit_Trellis     T[4];
Adafruit_UNTZtrument untztrument(&T[0], &T[1], &T[2], &T[3]);
const uint8_t        addr[] = { 0x70, 0x71, 0x72, 0x73 };
#else
// A HELLA UNTZtrument has eight Trellis boards...
Adafruit_Trellis     T[8];
Adafruit_UNTZtrument untztrument(&T[0], &T[1], &T[2], &T[3],
                                 &T[4], &T[5], &T[6], &T[7]);
const uint8_t        addr[] = { 0x70, 0x71, 0x72, 0x73,
                                0x74, 0x75, 0x76, 0x77
                              };
#endif // HELLA
//  const byte notePitches[] = {24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55};

#define WIDTH     ((sizeof(T) / sizeof(T[0])) * 2)
#define N_BUTTONS ((sizeof(T) / sizeof(T[0])) * 8)

//byte noteON = 144;  //144 = 10010000 in binary, note on command
//byte noteOFF = 128; //128 = 10000000 in binary, note off command

// Encoder on pins 4,5 sets tempo.  Optional, not present in
// standard UNTZtrument, but won't affect things if unconnected.

enc e(4, 5);  // SLOT4
enc e2(2, 3); // BPM
enc e3(6, 7); // SLOT3
enc e4(8, 9); // SLOT2
enc e5(10, 11); //SLOT1

uint8_t       grid[WIDTH];                 // Sequencer state
uint8_t       heart        = 0,            // Heartbeat LED counter
              col          = WIDTH - 1;    // Current column
unsigned int  bpm          = 240;          // Tempo
uint8_t  slot1;
uint8_t  slot2;
uint8_t  slot3;
uint8_t  slot4;

unsigned long beatInterval = 60000L / bpm, // ms/beat
              prevBeatTime = 0L,           // Column step timer
              prevReadTime = 0L;           // Keypad polling timer
int nON = 144;//144 = 10010000 in binary, note on command
int nOFF = 128;//128 = 10000000 in binary, note off command

// The note[] and channel[] tables are the MIDI note and channel numbers
// for to each row (top to bottom); they're specific to this application.
// bitmask[] is for efficient reading/writing bits to the grid[] array.
//int note[4]    = {  slot1, slot2, slot3, slot4  };
static const uint8_t PROGMEM
channel[4] = {   1,  1,  1,  1  },
             note[4]    = {  slot1, slot2, slot3, slot4  },
                          bitmask[4] = {   1,  2,  4,  8  };

void setup() {
  pinMode(LED, OUTPUT);
  //Serial.begin(31250);  //MIDI
  Serial.begin(115200);   //TEST
  Serial.println("MIDI-seq Demo-v0.4");

#ifndef HELLA
  untztrument.begin(addr[0], addr[1] );
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
  TWBR = 12; // 400 KHz I2C on 16 MHz AVR
#endif
  untztrument.clear();
  untztrument.writeDisplay();
  memset(grid, 0, sizeof(grid));

  enc::begin();                     // Initialize all encoder pins
  e2.setBounds(60 * 4, 480 * 4 + 3); // Set tempo limits
  e2.setValue(bpm * 4);              // *4's for encoder detents
  
  e.setBounds(25 * 4, 66 * 4 + 3); 
  e.setValue(30 * 4);              // *4's for encoder detents
  e3.setBounds(25 * 4, 66 * 4 + 3); 
  e3.setValue(31 * 4);              // *4's for encoder detents
  e4.setBounds(25 * 4, 66 * 4 + 3);
  e4.setValue(32 * 4);              // *4's for encoder detents
  e5.setBounds(25 * 4, 66 * 4 + 3);
  e5.setValue(33 * 4);              // *4's for encoder detents
}

// /////////////////// END SETUP \\\\\\\\\\\\\\\\\\

// Turn on (or off) one column of the display
void line(uint8_t x, boolean set) {
  for (uint8_t mask = 1, y = 0; y < 4; y++, mask <<= 1) {
    uint8_t i = untztrument.xy2i(x, y);
    if (set || (grid[x] & mask)) untztrument.setLED(i);
    else                        untztrument.clrLED(i);
  }
}

void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.println("-------");
  Serial.write(command);          //send note on or note off command
  Serial.println("-------");
  Serial.write(MIDInote);         //send pitch data
  Serial.println("-------");
  Serial.write(MIDIvelocity);     //send velocity data
  Serial.println("-------");
  Serial.println(MIDInote);
}

// /////////////////// LOOP \\\\\\\\\\\\\\\\\\

void loop() {
  uint8_t       mask;
  boolean       refresh = false;
  unsigned long t       = millis();
  enc::poll(); // Read encoder(s)

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
            MIDImessage(nOFF, pgm_read_byte(&note[y]), 127); //turn note off
            //noteOff(pgm_read_byte(&channel[y]), pgm_read_byte(&note[y]), 127);
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
    for (uint8_t row = 0, mask = 1; row < 4; row++, mask <<= 1) {
      if (grid[col] & mask) {
        //noteOff(pgm_read_byte(&channel[row]), pgm_read_byte(&note[row]), 127);
        MIDImessage(nOFF, pgm_read_byte(&note[row]), 127); //turn note off
      }
    }
    // Advance column counter, wrap around
    if (++col >= WIDTH) col = 0;
    // Turn on new column
    line(col, true);
    for (uint8_t row = 0, mask = 1; row < 4; row++, mask <<= 1) {
      if (grid[col] & mask) {
        //noteOn(pgm_read_byte(&channel[row]), pgm_read_byte(&note[row]), 127);
        MIDImessage(nON, pgm_read_byte(&note[row]), 127);
      }
    }


    prevBeatTime = t;
    refresh      = true;
    slot4          = e.getValue()  / 4;  // Div for encoder detents
    bpm            = e2.getValue() / 4; // Div for encoder detents
    slot3          = e3.getValue() / 4; // Div for encoder detents
    slot2          = e4.getValue() / 4; // Div for encoder detents
    slot1          = e5.getValue() / 4; // Div for encoder detents
    beatInterval = 60000L / bpm;

    Serial.print("--SLOT 4--");  // note 4
    Serial.println(slot4);
    Serial.print("--ENCODER2--");  // BPM
    Serial.println(bpm);
    Serial.print("--SLOT 3--");  // note 3
    Serial.println(slot3);
    Serial.print("--SLOT 2--");  // note 2
    Serial.println(slot2);
    Serial.print("--SLOT 1--");  // note 1
    Serial.println(slot1);
    Serial.println(" ");
    //delay(5000);
  }

  if (refresh) untztrument.writeDisplay();
}
