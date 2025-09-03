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

#define NUM_LEDS 60                // number of RGB LEDs on strip
#define DMXSTART 201               // first DMX channel
#define DMXLENGTH (NUM_LEDS*3)     // number of DMX channels used (3*60 LEDs)

void setup () {

 DMXSerial.init(DMXProbe);        // initialize DMX bus in manual access mode
 DMXSerial.maxChannel(512); // "onUpdate" will be called when all new ch data has arrived

 setupNeopixel();                 // setup the LED output hardcoded to pin 12 in ws2812.h

}

void loop() {
  uint8_t _dmxData;
  uint8_t *ptrIn;
  uint8_t *ptrOut;
  uint8_t Out[DMXLENGTH];

  // wait for an incomming DMX packet and write
  // the RGB data for 60 LEDs on the strip
  if (DMXSerial.receive()) {



    // Assign the pointer to the input buffer.
    ptrIn = DMXSerial.getBuffer();
    ptrOut = Out;

    ptrOut[0] = ptrIn[0+DMXSTART];
    ptrOut[1] = ptrIn[1+DMXSTART];
    ptrOut[2] = ptrIn[2+DMXSTART];
    ptrOut[3] = ptrIn[3+DMXSTART];    
    ptrOut[4] = ptrIn[4+DMXSTART];  
    ptrOut[5] = ptrIn[5+DMXSTART];
    ptrOut[6] = ptrIn[6+DMXSTART];
    ptrOut[7] = ptrIn[7+DMXSTART];
    ptrOut[8] = ptrIn[8+DMXSTART];
    ptrOut[9] = ptrIn[9+DMXSTART];
    ptrOut[10] = ptrIn[10+DMXSTART];
    ptrOut[11] = ptrIn[11+DMXSTART];
    ptrOut[12] = ptrIn[12+DMXSTART];
    ptrOut[13] = ptrIn[13+DMXSTART];
    ptrOut[14] = ptrIn[14+DMXSTART];
    ptrOut[15] = ptrIn[15+DMXSTART];
    ptrOut[16] = ptrIn[16+DMXSTART];
    ptrOut[17] = ptrIn[17+DMXSTART];
    ptrOut[18] = ptrIn[18+DMXSTART];
    ptrOut[19] = ptrIn[19+DMXSTART];
    ptrOut[20] = ptrIn[20+DMXSTART];
    ptrOut[21] = ptrIn[21+DMXSTART];
    ptrOut[22] = ptrIn[22+DMXSTART];
    ptrOut[23] = ptrIn[23+DMXSTART];

    // // Map the second set of 8 LED's
    ptrOut[24] = ptrIn[0+DMXSTART];
    ptrOut[25] = ptrIn[1+DMXSTART];
    ptrOut[26] = ptrIn[2+DMXSTART];
    ptrOut[27] = ptrIn[3+DMXSTART];    
    ptrOut[29] = ptrIn[5+DMXSTART];
    ptrOut[30] = ptrIn[6+DMXSTART];
    ptrOut[31] = ptrIn[7+DMXSTART];
    ptrOut[32] = ptrIn[8+DMXSTART];
    ptrOut[33] = ptrIn[9+DMXSTART];
    ptrOut[34] = ptrIn[10+DMXSTART];
    ptrOut[35] = ptrIn[11+DMXSTART];
    ptrOut[36] = ptrIn[12+DMXSTART];
    ptrOut[37] = ptrIn[13+DMXSTART];
    ptrOut[38] = ptrIn[14+DMXSTART];
    ptrOut[39] = ptrIn[15+DMXSTART];
    ptrOut[40] = ptrIn[16+DMXSTART];
    ptrOut[41] = ptrIn[17+DMXSTART];
    ptrOut[42] = ptrIn[18+DMXSTART];
    ptrOut[43] = ptrIn[19+DMXSTART];
    ptrOut[44] = ptrIn[20+DMXSTART];
    ptrOut[45] = ptrIn[21+DMXSTART];
    ptrOut[46] = ptrIn[22+DMXSTART];
    ptrOut[47] = ptrIn[23+DMXSTART];

    // Map the third set of 8 LED's
    ptrOut[48] = ptrIn[0+DMXSTART];
    ptrOut[49] = ptrIn[1+DMXSTART];
    ptrOut[50] = ptrIn[2+DMXSTART];
    ptrOut[51] = ptrIn[3+DMXSTART];
    ptrOut[52] = ptrIn[4+DMXSTART];
    ptrOut[53] = ptrIn[5+DMXSTART];
    ptrOut[54] = ptrIn[6+DMXSTART];
    ptrOut[55] = ptrIn[7+DMXSTART];
    ptrOut[56] = ptrIn[8+DMXSTART];
    ptrOut[57] = ptrIn[9+DMXSTART];
    ptrOut[58] = ptrIn[10+DMXSTART];
    ptrOut[59] = ptrIn[11+DMXSTART];
    ptrOut[60] = ptrIn[12+DMXSTART];
    ptrOut[61] = ptrIn[13+DMXSTART];
    ptrOut[62] = ptrIn[14+DMXSTART];
    ptrOut[63] = ptrIn[15+DMXSTART];
    ptrOut[64] = ptrIn[16+DMXSTART];
    ptrOut[65] = ptrIn[17+DMXSTART];
    ptrOut[66] = ptrIn[18+DMXSTART];
    ptrOut[67] = ptrIn[19+DMXSTART];
    ptrOut[68] = ptrIn[20+DMXSTART];
    ptrOut[69] = ptrIn[21+DMXSTART];
    ptrOut[70] = ptrIn[22+DMXSTART];
    ptrOut[71] = ptrIn[23+DMXSTART];
 
    // Map the fourth set of 8 LED's
    ptrOut[72] = ptrIn[0+DMXSTART];
    ptrOut[73] = ptrIn[1+DMXSTART];
    ptrOut[74] = ptrIn[2+DMXSTART];
    ptrOut[75] = ptrIn[3+DMXSTART];
    ptrOut[76] = ptrIn[4+DMXSTART];
    ptrOut[77] = ptrIn[5+DMXSTART];
    ptrOut[78] = ptrIn[6+DMXSTART];
    ptrOut[79] = ptrIn[7+DMXSTART];
    ptrOut[80] = ptrIn[8+DMXSTART];
    ptrOut[81] = ptrIn[9+DMXSTART];
    ptrOut[82] = ptrIn[10+DMXSTART];
    ptrOut[83] = ptrIn[11+DMXSTART];
    ptrOut[84] = ptrIn[12+DMXSTART];
    ptrOut[85] = ptrIn[13+DMXSTART];
    ptrOut[86] = ptrIn[14+DMXSTART];
    ptrOut[87] = ptrIn[15+DMXSTART];
    ptrOut[88] = ptrIn[16+DMXSTART];
    ptrOut[89] = ptrIn[17+DMXSTART];
    ptrOut[90] = ptrIn[18+DMXSTART];
    ptrOut[91] = ptrIn[19+DMXSTART];
    ptrOut[92] = ptrIn[20+DMXSTART];
    ptrOut[93] = ptrIn[21+DMXSTART];
    ptrOut[94] = ptrIn[22+DMXSTART];
    ptrOut[95] = ptrIn[23+DMXSTART];

    // Map the fifth set of 8 LED's
    ptrOut[96] = ptrIn[0+DMXSTART];
    ptrOut[97] = ptrIn[1+DMXSTART];
    ptrOut[98] = ptrIn[2+DMXSTART];
    ptrOut[99] = ptrIn[3+DMXSTART];
    ptrOut[100] = ptrIn[4+DMXSTART];
    ptrOut[101] = ptrIn[5+DMXSTART];
    ptrOut[102] = ptrIn[6+DMXSTART];
    ptrOut[103] = ptrIn[7+DMXSTART];
    ptrOut[104] = ptrIn[8+DMXSTART];
    ptrOut[105] = ptrIn[9+DMXSTART];
    ptrOut[106] = ptrIn[10+DMXSTART];
    ptrOut[107] = ptrIn[11+DMXSTART];
    ptrOut[108] = ptrIn[12+DMXSTART];
    ptrOut[109] = ptrIn[13+DMXSTART];
    ptrOut[110] = ptrIn[14+DMXSTART];
    ptrOut[111] = ptrIn[15+DMXSTART];
    ptrOut[112] = ptrIn[16+DMXSTART];
    ptrOut[113] = ptrIn[17+DMXSTART];
    ptrOut[114] = ptrIn[18+DMXSTART];
    ptrOut[115] = ptrIn[19+DMXSTART];
    ptrOut[116] = ptrIn[20+DMXSTART];
    ptrOut[117] = ptrIn[21+DMXSTART];
    ptrOut[118] = ptrIn[22+DMXSTART];
    ptrOut[119] = ptrIn[23+DMXSTART];

    // Map the sixth set of 8 LED's
    ptrOut[120] = ptrIn[0+DMXSTART];
    ptrOut[121] = ptrIn[1+DMXSTART];
    ptrOut[122] = ptrIn[2+DMXSTART];
    ptrOut[123] = ptrIn[3+DMXSTART];
    ptrOut[124] = ptrIn[4+DMXSTART];
    ptrOut[125] = ptrIn[5+DMXSTART];
    ptrOut[126] = ptrIn[6+DMXSTART];
    ptrOut[127] = ptrIn[7+DMXSTART];
    ptrOut[128] = ptrIn[8+DMXSTART];
    ptrOut[129] = ptrIn[9+DMXSTART];
    ptrOut[130] = ptrIn[10+DMXSTART];
    ptrOut[131] = ptrIn[11+DMXSTART];
    ptrOut[132] = ptrIn[12+DMXSTART];
    ptrOut[133] = ptrIn[13+DMXSTART];
    ptrOut[134] = ptrIn[14+DMXSTART];
    ptrOut[135] = ptrIn[15+DMXSTART];
    ptrOut[136] = ptrIn[16+DMXSTART];
    ptrOut[137] = ptrIn[17+DMXSTART];
    ptrOut[138] = ptrIn[18+DMXSTART];
    ptrOut[139] = ptrIn[19+DMXSTART];
    ptrOut[140] = ptrIn[20+DMXSTART];
    ptrOut[141] = ptrIn[21+DMXSTART];
    ptrOut[142] = ptrIn[22+DMXSTART];
    ptrOut[143] = ptrIn[23+DMXSTART];

    // Map the seventh set of 8 LED's
    ptrOut[144] = ptrIn[0+DMXSTART];
    ptrOut[145] = ptrIn[1+DMXSTART];
    ptrOut[146] = ptrIn[2+DMXSTART];
    ptrOut[147] = ptrIn[3+DMXSTART];
    ptrOut[148] = ptrIn[4+DMXSTART];
    ptrOut[149] = ptrIn[5+DMXSTART];
    ptrOut[150] = ptrIn[6+DMXSTART];
    ptrOut[151] = ptrIn[7+DMXSTART];
    ptrOut[152] = ptrIn[8+DMXSTART];
    ptrOut[153] = ptrIn[9+DMXSTART];
    ptrOut[154] = ptrIn[10+DMXSTART];
    ptrOut[155] = ptrIn[11+DMXSTART];
    ptrOut[156] = ptrIn[12+DMXSTART];
    ptrOut[157] = ptrIn[13+DMXSTART];
    ptrOut[158] = ptrIn[14+DMXSTART];
    ptrOut[159] = ptrIn[15+DMXSTART];
    ptrOut[160] = ptrIn[16+DMXSTART];
    ptrOut[161] = ptrIn[17+DMXSTART];
    ptrOut[162] = ptrIn[18+DMXSTART];
    ptrOut[163] = ptrIn[19+DMXSTART];
    ptrOut[164] = ptrIn[20+DMXSTART];
    ptrOut[165] = ptrIn[21+DMXSTART];
    ptrOut[166] = ptrIn[22+DMXSTART];
    ptrOut[167] = ptrIn[23+DMXSTART];

    // Map the remaining 4 Leds at the end
    ptrOut[168] = ptrIn[0+DMXSTART];
    ptrOut[169] = ptrIn[1+DMXSTART];
    ptrOut[170] = ptrIn[2+DMXSTART];
    ptrOut[171] = ptrIn[3+DMXSTART];
    ptrOut[172] = ptrIn[4+DMXSTART];
    ptrOut[173] = ptrIn[5+DMXSTART];
    ptrOut[174] = ptrIn[6+DMXSTART];
    ptrOut[175] = ptrIn[7+DMXSTART];
    ptrOut[176] = ptrIn[8+DMXSTART];
    ptrOut[177] = ptrIn[9+DMXSTART];
    ptrOut[178] = ptrIn[10+DMXSTART];
    ptrOut[179] = ptrIn[11+DMXSTART];
   

    updateNeopixel(ptrOut, NUM_LEDS);
 }
}

