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

//#include <DMXSerial.h>
//#include "ws2812.h"                // a specific LED controller that disables interrupts to work better

#define NUM_LEDS 56               // number of RGB LEDs on strip
#define DMXSTART 1                 // first DMX channel
#define DMXLENGTH (NUM_LEDS*3)     // number of DMX channels used (3*60 LEDs)

void setup () {

 //DMXSerial.init(DMXProbe);        // initialize DMX bus in manual access mode
 //DMXSerial.maxChannel(DMXLENGTH); // "onUpdate" will be called when all new ch data has arrived

 //setupNeopixel();                 // setup the LED output hardcoded to pin 12 in ws2812.h

 Serial.begin(9600);

}

void loop() {
  uint8_t _dmxData;

  uint8_t p;

  uint8_t In[513] ;
  uint8_t *ptrIn;

  uint8_t Out[513] ;
  uint8_t *ptrOut;

  uint8_t First8[24] ;
  uint8_t first8Counter;
  uint8_t *ptrFirst8;

  Serial.flush();
  Serial.println("Welcome to Booth Buddy");
  Serial.flush();

  // wait for an incomming DMX packet and write
  // the RGB data for 60 LEDs on the strip
  //if (DMXSerial.receive()) {
  //ptr = DMXSerial.getBuffer();

  for (int in=0; in<512 ; in++){
    In[in]=0;
  }

  for (int in=0; in<24 ; in++){
    In[in]=127;
  }

  Serial.println("Start With");
    
  for (int sw=0; sw<512; sw++){
    Serial.print(sw);
    Serial.print(":");
    Serial.print(In[sw]);
    Serial.print(",");
  } 
  Serial.println("");

  Serial.flush();

 
  // Assign the pointer to the input buffer.
  ptrIn = In;

  // Load the data for the first 24 channels (8 LEDs into a buffer)
  for (int p = 0; p <  24 ; p++ ) {
    First8[p] =*ptrIn;
    ptrIn++;
  }

  // Assign the pointer to the first8 buffer.
  ptrFirst8 = First8;
  
  Serial.println("Fisrt8 Set to");
  for (int sw=0; sw<24; sw++){
    Serial.print(sw);
    Serial.print(":");
    Serial.print(ptrFirst8[sw]);
    Serial.print(",");
  } 
  Serial.println("");
  
  
  // Initialise output buffer.
  for (int out = 0; out <  512 ; out++ ) {
     Out[out] = 0;
  }


  // Now use that data to create 7 copy's of the data spread across 56 LEDs
  for (int index=0; index < 512; index ++){
    
      Serial.print("Index ");
      Serial.print(index);
      Serial.println("");
    
      Out[index] =First8[first8Counter];
      first8Counter++;
    
    if (first8Counter > 24) {
      first8Counter=0;
    }

  }

  // Assign the pointer to the output buffer.
  ptrOut = Out;
  
  Serial.println("End With");
  for (int sw=0; sw<512; sw++){
    Serial.print(sw);
    Serial.print(":");
    Serial.print(ptrOut[sw]);
    Serial.print(",");
  } 
  Serial.println("");

  delay(1000);

  Serial.println("");
  Serial.println("");
  Serial.println("");

  exit(0);
  //updateNeopixel( *ptrOut+ DMXSTART, NUM_LEDS);
 }

