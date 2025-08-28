// DMX Receiver/Fixture Example for WS2812B RGB LEDs
// Target Hardware:  Arduino Uno
//
// Receives individual RGB levels for 60 WS2812B LEDs (60*3 = 180 DMX channels total)
// over an RS485 link in DMX packet format, to be decoded and sent out to a 60 LED strip.
// The test pattern is an LED chaser where two different color bars run end to end
// along the strip at different speeds, crossing over each other when they meet.
//
// The first DMX channel is 1 so the 60 LED RGB values are sent sequentially in
// channels 1 to 180, eg:
// Ch   1: LED  0 RED   value [0-255]
// Ch   2: LED  0 GREEN value [0-255]
// Ch   3: LED  0 BLUE  value [0-255]
// Ch 178: LED 59 RED   value [0-255]
// Ch 179: LED 59 GREEN value [0-255]
// Ch 180: LED 59 BLUE  value [0-255]
//
// Required library:
//    DMXSerial        install from library manager or https://github.com/mathertel/DMXSerial
//
// Gadget Reboot
// https://www.youtube.com/gadgetreboot

#include <DMXSerial.h>
#include "ws2812.h"                // a specific LED controller that disables interrupts to work better

#define NUM_LEDS 60               // number of RGB LEDs on strip
#define DMXSTART 1                 // first DMX channel
#define DMXLENGTH (NUM_LEDS*3)     // number of DMX channels used (3*60 LEDs)

void setup () {

 DMXSerial.init(DMXProbe);        // initialize DMX bus in manual access mode
 DMXSerial.maxChannel(DMXLENGTH); // "onUpdate" will be called when all new ch data has arrived

 setupNeopixel();                 // setup the LED output hardcoded to pin 12 in ws2812.h

}

void loop() {
  uint8_t _dmxData;

  uint8_t *ptrOut;

  // wait for an incomming DMX packet and write
  // the RGB data for 60 LEDs on the strip
  if (DMXSerial.receive()) {

    // Assign the pointer to the input buffer.
    ptrOut = DMXSerial.getBuffer();

    // Map the second set of 8 LED's
    ptrOut[24] = ptrOut[0];
    ptrOut[25] = ptrOut[1];
    ptrOut[26] = ptrOut[2];
    ptrOut[27] = ptrOut[3];
    ptrOut[28] = ptrOut[4];
    ptrOut[29] = ptrOut[5];
    ptrOut[30] = ptrOut[6];
    ptrOut[31] = ptrOut[7];
    ptrOut[32] = ptrOut[8];
    ptrOut[33] = ptrOut[9];
    ptrOut[34] = ptrOut[10];
    ptrOut[35] = ptrOut[11];
    ptrOut[36] = ptrOut[12];
    ptrOut[37] = ptrOut[13];
    ptrOut[38] = ptrOut[14];
    ptrOut[39] = ptrOut[15];
    ptrOut[40] = ptrOut[16];
    ptrOut[41] = ptrOut[17];
    ptrOut[42] = ptrOut[18];
    ptrOut[43] = ptrOut[19];
    ptrOut[44] = ptrOut[20];
    ptrOut[45] = ptrOut[21];
    ptrOut[46] = ptrOut[22];
    ptrOut[47] = ptrOut[23];
 
    updateNeopixel( ptrOut + DMXSTART, NUM_LEDS);
 }
}

