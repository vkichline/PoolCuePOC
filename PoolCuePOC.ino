//  Pool Cue Proof of Concept
//  Van Kichline
//  August 2019
//  Adapdted from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6
//  Using the I2Cdev libraries: https://www.i2cdevlib.com/
//  Modified for ESP8266, implemented on:
//    Wemos OLED shield (optional) 64×48 pixels (0.66” Across)
//    Proto shield with GY-521, interrup = D8.
//    Wemos D1 Mini
//    Wemos Battery board
//
//  Plan:
//    Create Access Point (APP_NAME)
//    Create captive DNS website
//    Utilize WebSocket server for data updates
//    Display YPR, tap, stroke, impact, 3d graph, etc.
//
//
//   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//   depends on the MPU-6050's INT pin being connected to the D1 Mini's
//   external interrupt D8 pin.
//
// NOTE: Returning:
//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
//OFFSETS    -1286,    -379,    1410,      40,     -64,      44

#include <I2Cdev.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include "MPU6050_6Axis_MotionApps20.h" // includes MPU6050.h


#define     OLED_RESET        -1
#define     INTERRUPT_PIN     D8
#define     BUTTON_A          D3
#define     BUTTON_B          D4
#define     DISPLAY_MODE_YPR  0
#define     DISPLAY_MODE_QUAT 1
#define     DISPLAY_MODE_EUL  2
#define     DISPLAY_MODE_REAL 3
#define     DISPLAY_MODE_WOR  4
#define     DISPLAY_MODE_TPT  5
#define     DISPLAY_MAX_ROT   DISPLAY_MODE_WOR


MPU6050           mpu(0x68);
Adafruit_SSD1306  display(OLED_RESET);        // Wemos OLED shield
bool              dmpReady          = false;  // set true if DMP init was successful
uint16_t          fifoCount         = 0;      // count of all bytes currently in FIFO
uint16_t          packetSize        = 42;     // expected DMP packet size (default is 42 bytes)
int               displayMode       = DISPLAY_MODE_YPR;
uint8_t           teapotPacket[14]  = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'}; // packet structure for InvenSense teapot demo


//  Interrupt Routine and status var
//
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
ICACHE_RAM_ATTR void dmpDataReady() { mpuInterrupt = true; }


//  Start up the OLED and display initial message
//
void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println(F("Starting.."));
  display.display();
  Serial.println(F("Display initialized."));
}


//  Initialize and calibrate GY-521
//
void initMPU() {
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nPausing 2 seconds before calibrating."));
  display.println(F("Pausing..."));
  display.display();
  delay(2000);

  // load and configure the DMP
  display.println(F("Setting..."));
  Serial.println(F("Initializing DMP..."));
  display.display();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


// Setup routine, called onced by RTOS
//
void setup() {
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Starting."));

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000);

  initDisplay();
  initMPU();
}


//  Display routines

// See the yaw/pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//
void displayYawPitchRoll(uint8_t* fifoBuffer) {
  Quaternion  q;        // [w, x, y, z]         quaternion container
  VectorFloat gravity;  // [x, y, z]            gravity vector
  float       ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(F("   Y/P/R\n\n"));
  display.printf(F("Yaw %6.1f\n"), ypr[0] * 180.0 / M_PI);
  display.printf(F("Pit %6.1f\n"), ypr[1] * 180.0 / M_PI);
  display.printf(F("Rol %6.1f\n"), ypr[2] * 180.0 / M_PI);
  display.display();
}


// See the actual quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//
void displayQuaternion(uint8_t* fifoBuffer) {
  Quaternion q; // [w, x, y, z] quaternion container

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(F("Quaternion\n\n"));
  display.printf(F("w %7.4f\n"), q.w);
  display.printf(F("x %7.4f\n"), q.x);
  display.printf(F("y %7.4f\n"), q.y);
  display.printf(F("z %7.4f\n"), q.z);
  display.display();
}


// See Euler angles (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//
void displayEuler(uint8_t* fifoBuffer) {
  Quaternion  q;        // [w, x, y, z] quaternion container
  float       euler[3]; // [psi, theta, phi]    Euler angle container

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(F("   Euler\n\n"));
  display.printf(F("x %7.4f\n"), euler[0] * 180.0 / M_PI);
  display.printf(F("y %7.4f\n"), euler[1] * 180.0 / M_PI);
  display.printf(F("z %7.4f\n"), euler[2] * 180.0 / M_PI);
  display.display();
}


// See acceleration components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//
void displayRealAccel(uint8_t* fifoBuffer) {
  Quaternion  q;       // [w, x, y, z] quaternion container
  VectorInt16 aa;      // [x, y, z]    accel sensor measurements
  VectorFloat gravity; // [x, y, z]    gravity vector
  VectorInt16 aaReal;  // [x, y, z]    gravity-free accel sensor measurements
  static int maxX, maxY, maxZ = 0;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  
  if(aaReal.x > maxX)  maxX = aaReal.x;
  if(aaReal.x > maxY)  maxY = aaReal.y;
  if(aaReal.x > maxZ)  maxZ = aaReal.z;
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(F("RealAccel (x, y, z)\n\n"));
  display.printf(F("%4d %5d\n"), aaReal.x, maxX);
  display.printf(F("%4d %5d\n"), aaReal.y, maxY);
  display.printf(F("%4d %5d\n"), aaReal.z, maxZ);
  display.display();
}


// See acceleration components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//
void displayWorld(uint8_t* fifoBuffer) {
  Quaternion  q;       // [w, x, y, z] quaternion container
  VectorInt16 aa;      // [x, y, z]    accel sensor measurements
  VectorFloat gravity; // [x, y, z]    gravity vector
  VectorInt16 aaReal;  // [x, y, z]    gravity-free accel sensor measurements
  VectorInt16 aaWorld; // [x, y, z]    world-frame accel sensor measurements
  static int maxX, maxY, maxZ = 0;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
  if(aaWorld.x > maxX)  maxX = aaWorld.x;
  if(aaWorld.x > maxY)  maxY = aaWorld.y;
  if(aaWorld.x > maxZ)  maxZ = aaWorld.z;

  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(F("WorldAccel(x, y, z)\n\n"));
  display.printf(F("%4d %5d\n"), aaWorld.x, maxX);
  display.printf(F("%4d %5d\n"), aaWorld.y, maxY);
  display.printf(F("%4d %5d\n"), aaWorld.z, maxZ);
  display.display();
}


//  Display quaternion values in InvenSense Teapot demo format.
//  This is intended to be text for the serial channel,
//  to interface with a Processing sketch.
//
void displayTeapot(uint8_t* fifoBuffer) {
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
}


//  Error selecting display mode
//
void displayModeError() {
  char buffer[32];

  itoa(displayMode, buffer, 10);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Mode error:");
  display.println(buffer);
  display.display();
}


//  Main program loop, called repeatedly by RTOS
//
void loop()
{
  uint8_t     mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t     fifoBuffer[64]; // FIFO storage buffer

  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    if (mpuInterrupt && fifoCount < packetSize)
    {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    yield();
  }

  // Test Button A for display mode changes
  if(!digitalRead(BUTTON_A)) {
    // Button A is pressed.  Debounce, and wait for button to be released.
    Serial.println(F("Button A pressed."));
    delay(10);
    while(!digitalRead(BUTTON_A)) {
      delay(1);
    }
    // Advance displayMode by one.  If past max, wrap around to zero.
    displayMode++;
    if(displayMode > DISPLAY_MAX_ROT) displayMode = 0;
    Serial.printf(F("Dislay mode set to %d\n"), displayMode);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize)
  {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
  {

    // read a packet from FIFO
    while (fifoCount >= packetSize)
    { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    switch(displayMode) {
      case DISPLAY_MODE_YPR:
        displayYawPitchRoll(fifoBuffer);
        break;
      case DISPLAY_MODE_QUAT:
        displayQuaternion(fifoBuffer);
        break;
      case DISPLAY_MODE_EUL:
        displayEuler(fifoBuffer);
        break;
      case DISPLAY_MODE_REAL:
        displayRealAccel(fifoBuffer);
        break;
      case DISPLAY_MODE_WOR:
        displayWorld(fifoBuffer);
        break;
      case DISPLAY_MODE_TPT:
        displayTeapot(fifoBuffer);
        break;
      default:
        displayModeError();
        break;
    }
  }
}

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
