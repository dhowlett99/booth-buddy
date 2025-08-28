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

    // Map the third set of 8 LED's
    ptrOut[48] = ptrOut[0];
    ptrOut[49] = ptrOut[1];
    ptrOut[50] = ptrOut[2];
    ptrOut[51] = ptrOut[3];
    ptrOut[52] = ptrOut[4];
    ptrOut[53] = ptrOut[5];
    ptrOut[54] = ptrOut[6];
    ptrOut[55] = ptrOut[7];
    ptrOut[56] = ptrOut[8];
    ptrOut[57] = ptrOut[9];
    ptrOut[58] = ptrOut[10];
    ptrOut[59] = ptrOut[11];
    ptrOut[60] = ptrOut[12];
    ptrOut[61] = ptrOut[13];
    ptrOut[62] = ptrOut[14];
    ptrOut[63] = ptrOut[15];
    ptrOut[64] = ptrOut[16];
    ptrOut[65] = ptrOut[17];
    ptrOut[66] = ptrOut[18];
    ptrOut[67] = ptrOut[19];
    ptrOut[68] = ptrOut[20];
    ptrOut[69] = ptrOut[21];
    ptrOut[70] = ptrOut[22];
    ptrOut[71] = ptrOut[23];
 
    // Map the fourth set of 8 LED's
    ptrOut[72] = ptrOut[0];
    ptrOut[73] = ptrOut[1];
    ptrOut[74] = ptrOut[2];
    ptrOut[75] = ptrOut[3];
    ptrOut[76] = ptrOut[4];
    ptrOut[77] = ptrOut[5];
    ptrOut[78] = ptrOut[6];
    ptrOut[79] = ptrOut[7];
    ptrOut[80] = ptrOut[8];
    ptrOut[81] = ptrOut[9];
    ptrOut[82] = ptrOut[10];
    ptrOut[83] = ptrOut[11];
    ptrOut[84] = ptrOut[12];
    ptrOut[85] = ptrOut[13];
    ptrOut[86] = ptrOut[14];
    ptrOut[87] = ptrOut[15];
    ptrOut[88] = ptrOut[16];
    ptrOut[89] = ptrOut[17];
    ptrOut[90] = ptrOut[18];
    ptrOut[91] = ptrOut[19];
    ptrOut[92] = ptrOut[20];
    ptrOut[93] = ptrOut[21];
    ptrOut[94] = ptrOut[22];
    ptrOut[95] = ptrOut[23];

    // Map the fifth set of 8 LED's
    ptrOut[96] = ptrOut[0];
    ptrOut[97] = ptrOut[1];
    ptrOut[98] = ptrOut[2];
    ptrOut[99] = ptrOut[3];
    ptrOut[100] = ptrOut[4];
    ptrOut[101] = ptrOut[5];
    ptrOut[102] = ptrOut[6];
    ptrOut[103] = ptrOut[7];
    ptrOut[104] = ptrOut[8];
    ptrOut[105] = ptrOut[9];
    ptrOut[106] = ptrOut[10];
    ptrOut[107] = ptrOut[11];
    ptrOut[108] = ptrOut[12];
    ptrOut[109] = ptrOut[13];
    ptrOut[110] = ptrOut[14];
    ptrOut[111] = ptrOut[15];
    ptrOut[112] = ptrOut[16];
    ptrOut[113] = ptrOut[17];
    ptrOut[114] = ptrOut[18];
    ptrOut[115] = ptrOut[19];
    ptrOut[116] = ptrOut[20];
    ptrOut[117] = ptrOut[21];
    ptrOut[118] = ptrOut[22];
    ptrOut[119] = ptrOut[23];

    // Map the sixth set of 8 LED's
    ptrOut[120] = ptrOut[0];
    ptrOut[121] = ptrOut[1];
    ptrOut[122] = ptrOut[2];
    ptrOut[123] = ptrOut[3];
    ptrOut[124] = ptrOut[4];
    ptrOut[125] = ptrOut[5];
    ptrOut[126] = ptrOut[6];
    ptrOut[127] = ptrOut[7];
    ptrOut[128] = ptrOut[8];
    ptrOut[129] = ptrOut[9];
    ptrOut[130] = ptrOut[10];
    ptrOut[131] = ptrOut[11];
    ptrOut[132] = ptrOut[12];
    ptrOut[133] = ptrOut[13];
    ptrOut[134] = ptrOut[14];
    ptrOut[135] = ptrOut[15];
    ptrOut[136] = ptrOut[16];
    ptrOut[137] = ptrOut[17];
    ptrOut[138] = ptrOut[18];
    ptrOut[139] = ptrOut[19];
    ptrOut[140] = ptrOut[20];
    ptrOut[141] = ptrOut[21];
    ptrOut[142] = ptrOut[22];
    ptrOut[143] = ptrOut[23];

    // Map the seventh set of 8 LED's
    ptrOut[144] = ptrOut[0];
    ptrOut[145] = ptrOut[1];
    ptrOut[146] = ptrOut[2];
    ptrOut[147] = ptrOut[3];
    ptrOut[148] = ptrOut[4];
    ptrOut[149] = ptrOut[5];
    ptrOut[150] = ptrOut[6];
    ptrOut[151] = ptrOut[7];
    ptrOut[152] = ptrOut[8];
    ptrOut[153] = ptrOut[9];
    ptrOut[154] = ptrOut[10];
    ptrOut[155] = ptrOut[11];
    ptrOut[156] = ptrOut[12];
    ptrOut[157] = ptrOut[13];
    ptrOut[158] = ptrOut[14];
    ptrOut[159] = ptrOut[15];
    ptrOut[160] = ptrOut[16];
    ptrOut[161] = ptrOut[17];
    ptrOut[162] = ptrOut[18];
    ptrOut[163] = ptrOut[19];
    ptrOut[164] = ptrOut[20];
    ptrOut[165] = ptrOut[21];
    ptrOut[166] = ptrOut[22];
    ptrOut[167] = ptrOut[23];

    // Map the remaining 4 Leds at the end
    ptrOut[168] = ptrOut[0];
    ptrOut[169] = ptrOut[1];
    ptrOut[170] = ptrOut[2];
    ptrOut[171] = ptrOut[3];
    ptrOut[172] = ptrOut[4];
    ptrOut[173] = ptrOut[5];
    ptrOut[174] = ptrOut[6];
    ptrOut[175] = ptrOut[7];
    ptrOut[176] = ptrOut[8];
    ptrOut[177] = ptrOut[9];
    ptrOut[178] = ptrOut[10];
    ptrOut[179] = ptrOut[11];
   

    updateNeopixel( ptrOut + DMXSTART, NUM_LEDS);
 }
}

