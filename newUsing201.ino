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
#define DMXSTART 201                 // first DMX channel
#define DMXLENGTH (NUM_LEDS*3)     // number of DMX channels used (3*60 LEDs)

void setup () {

 DMXSerial.init(DMXProbe);        // initialize DMX bus in manual access mode
 DMXSerial.maxChannel(DMXLENGTH+DMXSTART+1); // "onUpdate" will be called when all new ch data has arrived

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
    ptrOut[24+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[25+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[26+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[27+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[28+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[29+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[30+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[31+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[32+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[33+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[34+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[35+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[36+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[37+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[38+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[39+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[40+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[41+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[42+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[43+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[44+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[45+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[46+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[47+DMXSTART]= ptrOut[23+DMXSTART];

    // Map the third set of 8 LED's
    ptrOut[48+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[49+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[50+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[51+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[52+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[53+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[54+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[55+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[56+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[57+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[58+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[59+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[60+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[61+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[62+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[63+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[64+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[65+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[66+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[67+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[68+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[69+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[70+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[71+DMXSTART]= ptrOut[23+DMXSTART];
 
    // Map the fourth set of 8 LED's
    ptrOut[72+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[73+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[74+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[75+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[76+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[77+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[78+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[79+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[80+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[81+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[82+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[83+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[84+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[85+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[86+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[87+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[88+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[89+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[90+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[91+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[92+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[93+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[94+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[95+DMXSTART]= ptrOut[23+DMXSTART];

    // Map the fifth set of 8 LED's
    ptrOut[96+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[97+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[98+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[99+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[100+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[101+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[102+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[103+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[104+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[105+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[106+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[107+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[108+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[109+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[110+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[111+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[112+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[113+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[114+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[115+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[116+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[117+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[118+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[119+DMXSTART]= ptrOut[23+DMXSTART];

    // Map the sixth set of 8 LED's
    ptrOut[120+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[121+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[122+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[123+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[124+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[125+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[126+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[127+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[128+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[129+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[130+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[131+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[132+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[133+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[134+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[135+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[136+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[137+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[138+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[139+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[140+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[141+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[142+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[143+DMXSTART]= ptrOut[23+DMXSTART];

    // Map the seventh set of 8 LED's
    ptrOut[144+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[145+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[146+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[147+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[148+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[149+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[150+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[151+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[152+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[153+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[154+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[155+DMXSTART]= ptrOut[11+DMXSTART];
    ptrOut[156+DMXSTART]= ptrOut[12+DMXSTART];
    ptrOut[157+DMXSTART]= ptrOut[13+DMXSTART];
    ptrOut[158+DMXSTART]= ptrOut[14+DMXSTART];
    ptrOut[159+DMXSTART]= ptrOut[15+DMXSTART];
    ptrOut[160+DMXSTART]= ptrOut[16+DMXSTART];
    ptrOut[161+DMXSTART]= ptrOut[17+DMXSTART];
    ptrOut[162+DMXSTART]= ptrOut[18+DMXSTART];
    ptrOut[163+DMXSTART]= ptrOut[19+DMXSTART];
    ptrOut[164+DMXSTART]= ptrOut[20+DMXSTART];
    ptrOut[165+DMXSTART]= ptrOut[21+DMXSTART];
    ptrOut[166+DMXSTART]= ptrOut[22+DMXSTART];
    ptrOut[167+DMXSTART]= ptrOut[23+DMXSTART];

    // Map the remaining 4 Leds at the end
    ptrOut[168+DMXSTART]= ptrOut[0+DMXSTART];
    ptrOut[169+DMXSTART]= ptrOut[1+DMXSTART];
    ptrOut[170+DMXSTART]= ptrOut[2+DMXSTART];
    ptrOut[171+DMXSTART]= ptrOut[3+DMXSTART];
    ptrOut[172+DMXSTART]= ptrOut[4+DMXSTART];
    ptrOut[173+DMXSTART]= ptrOut[5+DMXSTART];
    ptrOut[174+DMXSTART]= ptrOut[6+DMXSTART];
    ptrOut[175+DMXSTART]= ptrOut[7+DMXSTART];
    ptrOut[176+DMXSTART]= ptrOut[8+DMXSTART];
    ptrOut[177+DMXSTART]= ptrOut[9+DMXSTART];
    ptrOut[178+DMXSTART]= ptrOut[10+DMXSTART];
    ptrOut[179+DMXSTART]= ptrOut[11+DMXSTART];
   

    updateNeopixel( ptrOut + DMXSTART, NUM_LEDS);
 }
}

