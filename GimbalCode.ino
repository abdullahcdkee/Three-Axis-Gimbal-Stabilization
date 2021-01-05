/*
   Gimbal Camera Self Stabilizing Platform
   Abdullah Siddiqui. 
*/
// I2Cdev, Arduino Wire, and MPU6050 libraries have been used for the following code
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
// class default I2C address is 0x68
// AD0 low = 0x68 
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); corresponding to AD0 high

// Define the 2 servo motors
Servo servo1;
Servo servo2;
float correct;   
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL //Gyro Output Parameters

#define INTERRUPT_PIN 2  


// ================================================================
// ===               DEFINING PARAMETERS                        ===
// ================================================================

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // True if DMP 
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer...Increase to reduce FIFO error?

// orientation/motion vars. Gyro parameters for posiiton control
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [xs, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int initial1 = 89;
int initial2 = 91;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high. decleared volatile as opposed to static
void dmpDataReady() {                   // Checks if Gyro is giving output 
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus, allow for communication between arduino and gyro
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Default. 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(38400);

  Serial.println("CLEARDATA"); //clears up any data left from previous projects

Serial.println("LABEL,Acolumn,Bcolumn,..."); //always write LABEL, so excel knows the next things will be the names of the columns (instead of Acolumn you could write Time for instance)

Serial.println("RESETTIMER"); //resets timer to 0
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
      mpu.initialize(); //initialize devices
      pinMode(INTERRUPT_PIN, INPUT);
      devStatus = mpu.dmpInitialize();  //check if gyro is sending data
  if (devStatus == 0) { // turn on the DMP
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); //Declare interrupt detection routine
      mpuIntStatus = mpu.getIntStatus();  //Check if interrupt received 
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      } 

  // Define the servo motor pins and the initial position of the motors
  servo1.attach(10); servo1.write(80);
  servo2.attach(9); servo2.write(80);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return;
  // wait for interrupt from Gyro
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();       // try to get out of the infinite loop as soon as interrupt received
    }
  }

  mpuInterrupt = false;   // reset interrupt flag and get interrupt status byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow, system may give error during processing if this is not included. This accounts for sync between hardware and software
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();      // reset gryo readings set
    fifoCount = mpu.getFIFOCount();
    Serial.println(("FIFO overflow!"));

    // check for DMP data ready interrupt
  } 
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();     // wait for correct available data
      mpu.getFIFOBytes(fifoBuffer, packetSize);   // read a packet from FIFO
    fifoCount -= packetSize;       // (this lets us immediately read more without waiting for an interrupt)

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);   //These required by library configuration helper functions
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   
    // Yaw, Pitch, Roll values -> Conversion from Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    
    if (j <= 500) {     // Skip the first 500 readings to establish accuracy (self-calibration process)
      correct = ypr[0]; // Yaw starts at random value, so doesn't matter what value we take
      j++;
    }
    else {      // After 500 readings
      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees

 
    // Map the gyro values to servo readable values
 
      int servo1Value = map(ypr[1], -180, 180, 0, 180);
      int servo2Value = map(ypr[2], -180, 180, 180, 0);

      Serial.print("DATA,TIME,TIMER,"); //writes the time in the first column A and the time since the measurements started in column B

      Serial.print(servo1Value);
      Serial.print(',');
      Serial.print(servo2Value);
      Serial.print(',');
      Serial.print(initial1+2*(initial1-servo1Value)-3);
      Serial.print(',');

      Serial.println(180-(initial2+2*(initial2-servo2Value)));
      
      
      // Position control according to the set point    
      servo1.write(initial1+2*(initial1-servo1Value)-10);           //-10 done to minimize steadys state error
      servo2.write(180-(initial2+2*(initial2-servo2Value))-10);
    }
#endif
  }
}
