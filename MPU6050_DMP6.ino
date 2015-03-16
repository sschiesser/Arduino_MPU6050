// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 -xctu fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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

// PinChangeInt: see the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.
#include <PinChangeInt.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

void do_output();
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


// ================================================================
// ===               CUSTOM HARDWARE VARIABLES                  ===
// ================================================================

/* ****************************************************************
   ***                        ARDUINO                           ***
   ****************************************************************/
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
boolean blinkState = false;
boolean debug = false;


/* ****************************************************************
   ***                        SLIDER                            ***
   ****************************************************************/
boolean slider = false;
int sliderPin = 2; // Analog read A2 pin
int sliderVal = 0;
int sliderValMin = 0;
int sliderValMax = 1023;
//float sliderValF = 0.0;
int sliderTransmitPos = 22; // First position of slider values in the transmit array
int sliderVals = 2; // Number of positions occupied by slider in the transmit array


/* ****************************************************************
   ***                    THUMB JOYSTICK                        ***
   ****************************************************************/
// thumbJoy pinout (L to R):
// Xout - 5V - Yout - gnd
boolean thumbJoy = false;
int thumbPinX = 0; // Analog read A0 pin
int thumbPinY = 1; // Analog read A1 pin
int thumbValX = 0;
int thumbValY = 0;
int thumbValXMin = 224;
int thumbValXMax = 890;
int thumbValYMin = 20;
int thumbValYMax = 870;
//float thumbValXF = 0.0;
//float thumbValYF = 0.0;
int thumbTransmitPos = 24; // First position of the thumb joystick values in the transmit array
int thumbVals = 4; // Number of positions occupied by thumbJoy in the transmit array


/* ****************************************************************
   ***                       TRACKBALL                          ***
   ****************************************************************/
boolean trackball = false;
int tbLedBluePin = 11; // Digital out 8
int tbLedRedPin = 10; // Digital out 9
int tbLedGreePinn = 9; // Digital out 10
int tbLedWhitePin = 8; // Digital out 11
int tbWheelUpPin = 3; // Digital in 3 (PCInt)
int tbWheelDownPin = 4; // Digital in 4 (PCInt)
int tbWheelLeftPin = 5; // Digital in 5 (PCInt)
int tbWheelRightPin = 6; // Digital in 6 (PCInt)
int tbButtonPin = 7; // Digital in 7 (PCInt)

int tbTransmitPos = 28; // First position of the trackball values in the transmit array
int tbVals = 4; // Number of positions occupied by trackball in the transmit array

volatile int tbWheelVertCntRaw = 120;
volatile int tbWheelHorizCntRaw = 120;
int tbWheelCntMax = 240;
int tbWheelCntMin = 0;
int tbButtonPressed = 0;



/* ****************************************************************
   ***                        MPU-6050                          ***
   ****************************************************************/
// uncomment "OUTPUT_JASCH_FRAME" if you want to work with jasch xcode app
// "ICST_bracelet_MPU" and get an AHRS view of the MPU
#define OUTPUT_ASK_FRAME

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int mpuIntPin = 2;      // interrupt pin for MPU data ready (digital in 2)

//#ifdef OUTPUT_ASK_FRAME
// Frame format:           {ST, ADD,    quaternions,      accel xyz,   gyro xyz,   slider, thumb joy, trackball,        ...         SP }
    uint8_t transmit[36] = {60, 191, 0,0, 0,0, 0,0, 0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,  0,0,    0,0,0,0,   0,0,0,0,   0x00, 0x00, 0x00, 90 };
//#endif


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
//  digitalWrite(12, HIGH);
  mpuInterrupt = true;
//  digitalWrite(12, LOW);
}

void tbButtonInt() {
  if(digitalRead(tbButtonPin) == LOW) {
    digitalWrite(tbLedWhitePin, HIGH);
    tbButtonPressed = 1;
//    if(debug) Serial.println("DOWN");
  }
  else {
    digitalWrite(tbLedWhitePin, LOW);
    tbButtonPressed = 0;
//    if(debug) Serial.println("UP");
  }
}

void tbWheelUpInt() {
  digitalWrite(tbLedBluePin, HIGH);
  if(tbWheelVertCntRaw < tbWheelCntMax) tbWheelVertCntRaw += 10;
  if(debug) {
//    Serial.print("Vertical: ");
//    Serial.println(tbWheelVertCntRaw);
  }
}

void tbWheelDownInt() {
  digitalWrite(tbLedBluePin, LOW);
  if(tbWheelVertCntRaw > tbWheelCntMin) tbWheelVertCntRaw -= 10;
  if(debug) {
//    Serial.print("Vertical: ");
//    Serial.println(tbWheelVertCntRaw);
  }
}

void tbWheelLeftInt() {
  digitalWrite(tbLedRedPin, HIGH);
  if(tbWheelHorizCntRaw > tbWheelCntMin) tbWheelHorizCntRaw -= 10;
  if(debug) {
//    Serial.print("Horizontal: ");
//    Serial.println(tbWheelHorizCntRaw);
  }
}

void tbWheelRightInt() {
  digitalWrite(tbLedRedPin, LOW);
  if(tbWheelHorizCntRaw < tbWheelCntMax) tbWheelHorizCntRaw += 10;
  if(debug) {
//    Serial.print("Horizontal: ");
//    Serial.println(tbWheelHorizCntRaw);
  }
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    #ifdef OUTPUT_ASK_FRAME
        Serial.begin(115200);
    #else
        Serial.begin(115200);
    #endif
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // Slider setuo
    if(slider) {
      if(debug) Serial.println("Slider enabled!");
    }
    else {
      for(int i = 0; i < sliderVals; i++) {
        transmit[sliderTransmitPos + i] = 100 + i;
      }
    }
    
    // Thumb joystick setup
    if(thumbJoy) {
      if(debug) Serial.println("Thumb joystick enabled!");
    }
    else {
      for(int i = 0; i < thumbVals; i++) {
        transmit[thumbTransmitPos + i] = 120 + i;
      }
    }
    
    // Trackball setup
    if(trackball) {
      if(debug) Serial.println("Trackball enabled!");
      pinMode(tbWheelUpPin, INPUT);
      pinMode(tbWheelDownPin, INPUT);
      pinMode(tbWheelLeftPin, INPUT);
      pinMode(tbWheelRightPin, INPUT);
      pinMode(tbButtonPin, INPUT_PULLUP);
      
      attachPinChangeInterrupt(tbButtonPin, tbButtonInt, CHANGE);
      attachPinChangeInterrupt(tbWheelUpPin, tbWheelUpInt, RISING);
      attachPinChangeInterrupt(tbWheelDownPin, tbWheelDownInt, RISING);
      attachPinChangeInterrupt(tbWheelLeftPin, tbWheelLeftInt, RISING);
      attachPinChangeInterrupt(tbWheelRightPin, tbWheelRightInt, RISING);
    }
    else {
      for(int i = 0; i < tbVals; i++) {
        transmit[tbTransmitPos + i] = 140 + i;
      }
    }

    // initialize MPU
    if(debug) Serial.println(F("Initializing I2C devices..."));
    // I2C:
    // - w -> PWR_MGMT_1 (0x6B)
    // - w -> GYRO_CONFIG (0x1B)
    // - w -> ACCEL_CONFIG (0x1C)
    // - w -> PWR_MGMT_1 (0x6B)
    mpu.initialize();

    // verify connection
    if(debug) Serial.println(F("Testing device connections..."));
    // I2C:
    // - r -> WHO_AM_I (0x75)
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    if(debug) Serial.println(F("Initializing DMP..."));
    // I2C:
    // - w -> PWR_MGMT_1 (0x6B)
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        if(debug) Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        if(debug) Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = mpu.getIntStatus();
        // try with pin change interrupt
//        attachPinChangeInterrupt(mpuIntPin, dmpDataReady, RISING);
        attachInterrupt(0, dmpDataReady, RISING);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        if(debug) Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        if(debug) {
          Serial.print("Packet size: ");
          Serial.println(packetSize);
        }
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        
//        if(debug) {
//          Serial.print("!INT -> FIFO count: ");
//          Serial.println(fifoCount);
//        }
//        do_output();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
//    if(debug) {
//      Serial.print("FIFO count: ");
//      Serial.println(fifoCount);
//    }

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        if(debug) {
          Serial.print("FIFO count: ");
          Serial.println(fifoCount);
        }
//        noInterrupts();
        do_output();
//        interrupts();
    }
}


void do_output() {
    #ifdef OUTPUT_ASK_FRAME
    // send data according to jasch MPU bracelet frame + additional ASK thumb sensors values
    // {ST, ADD,    quaternions,      accel xyz,   gyro xyz,   slider, thumb joy, trackball,        ...         SP }
    // {60, 191, 0,0, 0,0, 0,0, 0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,  0,0,    0,0,0,0,   0,0,0,0,   0x00, 0x00, 0x00, 90 };
    transmit[2] = fifoBuffer[0]; // quaternion values MSB LSB
    transmit[3] = fifoBuffer[1];
    transmit[4] = fifoBuffer[4];
    transmit[5] = fifoBuffer[5];
    transmit[6] = fifoBuffer[8];
    transmit[7] = fifoBuffer[9];
    transmit[8] = fifoBuffer[12];
    transmit[9] = fifoBuffer[13];
  
    transmit[10] = fifoBuffer[28]; // accel xyz  MSB
    transmit[11] = fifoBuffer[29]; // LSB
    transmit[12] = fifoBuffer[32];
    transmit[13] = fifoBuffer[33];
    transmit[14] = fifoBuffer[36];
    transmit[15] = fifoBuffer[37];
    
    transmit[16] = fifoBuffer[16]; // gyro xyz MSB
    transmit[17] = fifoBuffer[17]; // LSB
    transmit[18] = fifoBuffer[20];
    transmit[19] = fifoBuffer[21];
    transmit[20] = fifoBuffer[24];
    transmit[21] = fifoBuffer[25];
  
    if(slider) {
      sliderVal = analogRead(sliderPin); // - sliderValMin;
      transmit[22] = (uint8_t)((sliderVal >> 8) & 0xFF);
      transmit[23] = (uint8_t)(sliderVal & 0xFF);
    }
    if(thumbJoy) {
      thumbValX = analogRead(thumbPinX); // - thumbValXMin;
      thumbValY = analogRead(thumbPinY); // - thumbValYMin;
      if(debug) {
        Serial.print("thumbVal: ");
        Serial.print(thumbValX);
        Serial.print(" ");
        Serial.println(thumbValY);
      }
      transmit[24] = (uint8_t)((thumbValX >> 8) & 0xFF);
      transmit[25] = (uint8_t)(thumbValX & 0xFF);
      transmit[26] = (uint8_t)((thumbValY >> 8) & 0xFF);
      transmit[27] = (uint8_t)(thumbValY & 0xFF);
    }
    if(trackball) {
      transmit[28] = (uint8_t)(tbWheelHorizCntRaw);
      transmit[29] = (uint8_t)(tbWheelVertCntRaw);
      transmit[30] = tbButtonPressed;
    }
  	
      
    if(debug) {
      Serial.print("Slider: ");
      Serial.print(transmit[22]);
      Serial.print(" ");
      Serial.println(transmit[23]);
//      for(int i = 0; i < 36; i++) {
//        Serial.print(transmit[i]);
//        Serial.print("\t");
//      }
//      Serial.print("\n");
    }
    else {
      Serial.write(transmit, 36);
    }
  #endif
     
  
  #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
  #endif
  
  #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);
  #endif
  
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
  #endif
  
  #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
  #endif
  
  #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
  #endif
  
  #ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  #endif
  
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

