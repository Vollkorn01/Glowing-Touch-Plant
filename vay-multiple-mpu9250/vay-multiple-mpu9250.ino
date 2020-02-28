// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

//=============================================================================
//                            LED STUFF
//=============================================================================
//
#include <FastLED.h>

// How many leds do you want to activate in your strip?
#define NUM_LEDS 225
#define ACTIVE_LEDS 15
#define BRIGHTNESS 128


// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 5
#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];

//=============================================================================
//                            DEFINING LOGIC VARS
//=============================================================================
//

// define 2 times since we have two multiplexer boards
const int sensorNumber = 16;
const int startingSensorNumber = 0; //starts at 0
int darkness[sensorNumber];
int plantTouched[sensorNumber];
int fadeAmount[sensorNumber];  // Set the amount to fade I usually do 5, 10, 15, 20, 25 etc even up to 255.
int colorStep = 1;
int invert = 1;
int count = 0;
int shift = 0; // since not every led strip has same length, we need to shift


//=============================================================================
//                            DEFINING OUTPUTS
//=============================================================================
//

// define Serial Output
//#define SerialPrintSetup  // uncomment this to not print in serial monitor
//#define SerialPrintSensor// uncomment this to not print in serial monitor
//#define SerialPrintDebug // uncomment this to not print in serial monitor
//#define SerialPrintLED// uncomment this to not print in serial monitor
//#define SerialPrintSound //sends string to raspberry over serial, where sound is played
// Labeling Initialization
#define LED_PIN 13


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

int16_t ax, ay, az, aTot[sensorNumber];
int16_t aTotCalibrated[sensorNumber];
int16_t aTotCalibrationValue[sensorNumber];

// i2xmux init
#define MPU_addr 0x68
byte TCAADDR1 = 112; //multiplexer 1
byte TCAADDR2 = 113; //multiplexer 2
byte activeMultiplexer = TCAADDR1; // inactive and active will switch in code
byte inActiveMultiplexer = TCAADDR2;
int activeSensor;

#define LED_PIN 13
bool blinkState = false;

uint8_t i=0; // reset for SD Card logging


//=============================================================================
//                                   SETUP
//=============================================================================

void setup() {
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );
  
  // initialize sensor variables
  
  for(int x = startingSensorNumber; x < sensorNumber; x++)
  {
    darkness[x] = 255;
    plantTouched[x] = 0;
    fadeAmount[x] = 20; 
  }

  Serial.begin(38400);
  delay(300);                                                            

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    #ifdef SerialPrintSetup
      Serial.println("wire began");
    #endif

    byte activeMultiplexer = TCAADDR1;
    byte inActiveMultiplexer = TCAADDR2;
    
    for(int x = startingSensorNumber; x < sensorNumber; x++)
    {
      activeSensor = x;
      if (x > 7) {
        activeMultiplexer = TCAADDR2;
        inActiveMultiplexer = TCAADDR1;
        activeSensor = activeSensor - 8; // so we can start at 1 again for 2nd multiplexer board
      } else {
      activeMultiplexer = TCAADDR1;
      inActiveMultiplexer = TCAADDR2;
    }
    
    #ifdef SerialPrintSetup
      Serial.println("Wire.beginTransmission inActiveMultiplexer");
    #endif
    
    Wire.beginTransmission(inActiveMultiplexer);
    Wire.write(0);
    Wire.endTransmission(); 
    
    Wire.beginTransmission(activeMultiplexer);
    Wire.write(1 << activeSensor);
    Wire.endTransmission();       
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    #ifdef SerialPrintSetup
      Serial.println("accelGyroMag.enableMag");
    #endif
    accelGyroMag.enableMag();
    #ifdef SerialPrintSetup
    // initialize device
    Serial.println("Initializing I2C devices...");
      accelGyroMag.initialize();
      // verify connection
      Serial.println("Testing device connections...");
      Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
    #endif
  }

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, true);
  delay(1000);
  digitalWrite(LED_PIN, false);
}

void loop() {
  
  count ++;

 // reads sensor data and turns on leds from sensors in multiplexer 1
 for (int t = startingSensorNumber; t < sensorNumber; t++)
   {
    if (count == 100) {
      aTotCalibrationValue[t] = aTot[t];
      if (t == sensorNumber-1) {
      count = 0;
      }
    }

    activeSensor = t;
    if (t > 7) {
      activeMultiplexer = TCAADDR2;
      inActiveMultiplexer = TCAADDR1;
      activeSensor = activeSensor - 8; // so we can start at 1 again for 2nd multiplexer board
    } else {
      activeMultiplexer = TCAADDR1;
      inActiveMultiplexer = TCAADDR2;
    }
    
    Wire.beginTransmission(inActiveMultiplexer); // disable multiplexer board 2, so no address conflict
    Wire.write(0);
    Wire.endTransmission();   
    
    Wire.beginTransmission(activeMultiplexer); // enable multiplexer board 1
    Wire.write(1 << activeSensor);
    Wire.endTransmission();    
    // read raw accel/gyro/mag measurements from device
    accelGyroMag.getAcceleration(&ax, &ay, &az);
    aTot[t] = (ax + ay + az)/3;
    aTotCalibrated[t] = aTot[t] - aTotCalibrationValue[t];
    //these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
    
    #ifdef SerialPrintSensor
        // display tab-separated accel/gyro/mag x/y/z values
        //Serial.print("count: ");
        //Serial.print(count); Serial.print("\t");
        Serial.print("aTotCalibrated: ");
        Serial.print(aTotCalibrated[t]); Serial.print("\t");
        //Serial.print("aTot: ");
        //Serial.print(aTot[t]); Serial.print("\t");
        /*
        Serial.print("ay: ");
        Serial.print(ay); Serial.print("\t");
       
        Serial.print("ax: ");
        Serial.print(ax); Serial.print("\t");
        Serial.print("ay: ");
        Serial.print(ay); Serial.print("\t");
        Serial.print("az: ");
        Serial.print(az); Serial.print("\t");
        Serial.print("tot: ");
        Serial.print(ax+ay+az); Serial.print("\t");
        */
        //Serial.print("gx: ");
        //Serial.print(gx); Serial.print("\t");
        //Serial.print("gy: ");
        //Serial.print(gy); Serial.print("\t");
        //Serial.print(gz); Serial.print("\t");
        //Serial.print(int(mx)); Serial.print("\t");
        //Serial.print(int(my)); Serial.print("\t");
        //Serial.print(int(mz)); Serial.print("\t");
        
      #endif
    
    if (aTotCalibrated[t] > 700) {
      #ifdef SerialPrintDebug
        Serial.print("sensorNumber ");
        Serial.println(t);
      #endif
      plantTouched[t] = 1;
    }
    if (plantTouched[t] == 1) {
      //play sound on raspberry
      #ifdef SerialPrintSound
        Serial.println("starsound_c");
      #endif
    
      darkness[t] = darkness[t] - fadeAmount[t];
      // reverse the direction of the fading at the ends of the fade:
      if( darkness[t] <= 0) {
        fadeAmount[t] = -fadeAmount[t] ;
        darkness[t] = 0;
      }
      if (darkness[t] > 255) {
        plantTouched[t] = 0;
        darkness[t] = 255;
        fadeAmount[t] = -fadeAmount[t];
      }
      int startingLedNumber = 0;
      int activeLeds = 0;
      switch (t) {
        case 0:
          startingLedNumber = 0;
          activeLeds = 15;
          break;
         case 1:
          startingLedNumber = 15;
          activeLeds = 15;
          break;
        case 2:
          startingLedNumber = 30;
          activeLeds = 15;
          break;
        case 3:
          startingLedNumber = 45;
          activeLeds = 15;
          break;
        case 4:
          startingLedNumber = 60;
          activeLeds = 15;
          break;
        case 5:
          startingLedNumber = 75;
          activeLeds = 15;
          break;
        case 6:
          startingLedNumber = 90;
          activeLeds = 14;
          break;
        case 7:
          startingLedNumber = 105;
          activeLeds = 14;
          break;
        case 8:
          startingLedNumber = 119;
          activeLeds = 15;
          break;
        case 9:
          startingLedNumber = 134;
          activeLeds = 14;
          break;
        case 10:
          startingLedNumber = 148;
          activeLeds = 15;
          break;
        case 11:
          startingLedNumber = 163;
          activeLeds = 14;
          break;
        case 12:
          startingLedNumber = 177;
          activeLeds = 15;
          break;
        case 13:
          startingLedNumber = 192;
          activeLeds = 13;
          break;
        case 14:
          startingLedNumber = 205;
          activeLeds = 15;
          break;
      };
     
       for(int i = 0; i < activeLeds; i++ )
       {
       //smoothly change colors 
       leds[i+startingLedNumber].setRGB(0,int(colorStep),int((255-colorStep)));
       if (colorStep >= 240) {
        invert *= -1;
       } else if (colorStep < 1) {
        invert *= -1;
       }
       
       leds[i+startingLedNumber].fadeLightBy(darkness[t]);
       //shift = 0;
       #ifdef SerialPrintLED
          Serial.print("LEDS: ");
          Serial.print(i+startingLedNumber); Serial.print("\t");
       #endif
      }
      FastLED.show();
  }
  colorStep += invert;
}

#ifdef SerialPrintSensor
  Serial.println();
#endif
    #ifdef SerialPrintLED
  Serial.println();
#endif


// blink LED to indicate activity
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);
}
