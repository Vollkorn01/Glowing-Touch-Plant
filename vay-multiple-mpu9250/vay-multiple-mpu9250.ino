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

// How many leds to you want to activate in your strip?
#define NUM_LEDS 45
#define ACTIVE_LEDS 15
#define BRIGHTNESS 64


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

const int sensorNumber = 3;
const int startingSensorNumber = 0; //starts at 0
int darkness[sensorNumber];
int plantTouched[sensorNumber];
int fadeAmount[sensorNumber];  // Set the amount to fade I usually do 5, 10, 15, 20, 25 etc even up to 255.
int count = 0;


//=============================================================================
//                            DEFINING OUTPUTS
//=============================================================================
//

// README:
// uncomment #define BluetoothTransmit, make bool readDate = true, change baudrate of serial1 to 38400
// and change module to hc-05 in order to stream data to smartphone

// define bluetooth output
//#define BluetoothTransmit // uncomment this to not transmit via bluetooth

// define Serial Output
#define SerialPrint  // uncomment this to not print in serial monitor

// define SD Card Logger
//#define Adalogger  // uncomment this to not print on sd card

// starts logging / streaming when receiving start signal from App
  bool startStream = true;

// Labeling Initialization
int exercise = 99;
#define LED_PIN 13
int userNumber = 100;

// Timing init
int startTime;
int endTime;


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

int16_t ax, ay[sensorNumber], az;
int16_t ayCalibrated[sensorNumber];
int16_t ayCalibrationValue[sensorNumber];
int16_t gx, gy, gz;
int16_t mx, my, mz;

// i2xmux init
#define MPU_addr 0x68
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}
#define LED_PIN 13
bool blinkState = false;

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
  
  
  delay(200);
  
  // SD Card Logger Setup
  //----------------------------------------------------------------------------------

  Serial.begin(38400);
  //Serial.println("test with init");
  //Serial1.begin(38400);   
  delay(300);                                                            
  //Serial.println("\r\nAnalog logger test");
  pinMode(13, OUTPUT);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    //Serial.println("wire began");
    for(int x = startingSensorNumber; x < sensorNumber; x++)
    {
      tcaselect(x); // make a loop for all sensors here!
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      accelGyroMag.enableMag();
    }
  
    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

}

uint8_t i=0; // reset for SD Card logging


//=============================================================================
//                                   LOOP
//=============================================================================

void loop() {
  startTime = millis();
  count ++;


 for (int t = startingSensorNumber; t < sensorNumber; t++)
   {
    if (count == 100) {
      ayCalibrationValue[t] = ay[t];
      if (t == sensorNumber-1) {
      count = 0;
      }
    }
    tcaselect(t);
    // read raw accel/gyro/mag measurements from device
    accelGyroMag.getMotion9(&ax, &ay[t], &az, &gx, &gy, &gz, &mx, &my, &mz);  // or     accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    ayCalibrated[t] = ay[t] - ayCalibrationValue[t];
    //these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
    
    #ifdef SerialPrint
        // display tab-separated accel/gyro/mag x/y/z values
        //Serial.print("count: ");
        //Serial.print(count); Serial.print("\t");
        Serial.print("ayCalibrated: ");
        Serial.print(ayCalibrated[t]); Serial.print("\t");
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
        //Serial.print(gx); Serial.print("\t");
        //Serial.print(gy); Serial.print("\t");
        //Serial.print(gz); Serial.print("\t");
        //Serial.print(int(mx)); Serial.print("\t");
        //Serial.print(int(my)); Serial.print("\t");
        //Serial.print(int(mz)); Serial.print("\t");
        
      #endif
    
    if (ayCalibrated[t] > 1500) {
      plantTouched[t] = 1;
    }
    
    if (plantTouched[t] == 1) {
    
      darkness[t] = darkness[t] - fadeAmount[t];
      // reverse the direction of the fading at the ends of the fade:
      if( darkness[t] <= 0) {
        fadeAmount[t] = -fadeAmount[t] ;
        darkness[t] = 0;
      }
      if (darkness[t] > 255) {
        plantTouched[t] = 0;
        darkness[t] = 255;
        fadeAmount[t] = -fadeAmount[t] ;
      }
      //leds[t].setRGB(0,255,0);
      //leds[t].fadeLightBy(darkness[t]);
      
       for(int i = 0; i < ACTIVE_LEDS; i++ )
       {
       leds[i+t*ACTIVE_LEDS].setRGB(0,255,0);  // setRGB functions works by setting
                                 // (RED value 0-255, GREEN value 0-255, BLUE value 0-255)
                                 // RED = setRGB(255,0,0)
                                 // GREEN = setRGB(0,255,0)
       leds[i+t*ACTIVE_LEDS].fadeLightBy(darkness[t]);
       FastLED.show();
       
       #ifdef SerialPrint
          Serial.print("LEDS: ");
          Serial.print(i+t*ACTIVE_LEDS); Serial.print("\t");
       #endif
      }
      
      //FastLED.show();

      //Serial.print("i+t*NUM_LEDS: ");
      //Serial.print(i+t*NUM_LEDS); Serial.print("\t");
      //Serial.print("plantTouched: ");
      //Serial.print(plantTouched[t]); Serial.print("\t");
      //Serial.print("darkness: ");
      //Serial.print(darkness[t]); Serial.print("\t");
  }
  if(t == sensorNumber - 1) {
    #ifdef SerialPrint
      Serial.println();
    #endif

  }
}

// blink LED to indicate activity
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);

endTime = millis();  // THIS DOESNT NECESSARILY MAKES SENSE -> DATAPOINTS ARENT LINEARLY DISTRIBUTED
if (endTime - startTime < 33)
{
  //delay(33 - (endTime - startTime));
  //delay(5);
}
}
