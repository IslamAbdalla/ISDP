
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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ==============================================================
// ===                  PROGRAM VARIABLES                     ===
//===============================================================
int readAnglePin = 3;
int flag = 1;
int readAngle;
int refAngle = 0;
int backMotor = 9;
int angle ;
int LED = 1;
int LED2 = 8;

// IR Variables
int IRFrontPin = 5;
int IRFront = 1;
int IRFrontFlag = 0 ;

#define MOTOR_PWM   80

int leftMotorDir = 4;
int leftPin = 5;
int rightPin = 6;
int rightMotorDir = 7;    


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  /******************** Program setup **************************/

  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(readAnglePin, INPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(backMotor, OUTPUT);
  pinMode(IRFrontPin, INPUT);
  pinMode(leftMotorDir, OUTPUT);   
  pinMode(rightMotorDir, OUTPUT);   

  /*************************************************************/

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
  Serial.begin(115200);
  // while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

#define stepPWM 100
#define stepTime 10

void stepLeft() {
  analogWrite(rightPin, 0);
  analogWrite(leftPin, 100);
  delay(10);
  analogWrite(leftPin, 0);
}

void stepRight() {
  analogWrite(leftPin, 0);
  analogWrite(rightPin, 100);
  delay(10);
  analogWrite(rightPin, 0);
}
void loop() {
  // if programming failed, don't try to do anything
 // if (!dmpReady) return;      // commented to test

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    //    for( int x = 0; x < 20 ; x++)
    //      stepRight();

    // delay(10);
    angle = ypr[0] * 180 / M_PI;
    IRFront = digitalRead(IRFrontPin);   // White ==== 0; Black === 1;

    if ( flag) {
      
      readAngle = digitalRead(readAnglePin);

      // Serial.print(readAngle);
      // Serial.print(" ");

      Serial.println(angle);


      digitalWrite(LED, (angle & 0b01) ? HIGH : LOW);

      digitalWrite(LED2, (angle & 0b10) ? HIGH : LOW);

    }
    else {
      readAngle = 0;
      readAngle = digitalRead(readAnglePin);

      // Print ref
      Serial.print("Ref = ");
      Serial.print(refAngle);

      // Print diff
      Serial.print("\tDiff = ");
      Serial.println(refAngle - angle);

    }



    if (readAngle) {
      refAngle = angle;
      flag = 0;
    }


    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

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


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    // blink LED to indicate activity
    blinkState = !blinkState;

    // analogWrite(backMotor, (angle > 50 )? 250 : angle+ 200);

    //analogWrite(backMotor, 255);
    analogWrite(leftPin, 255 );
    analogWrite(rightPin, 255  );

    if (refAngle) {

      analogWrite(backMotor, 255);
      if ((refAngle - angle) < 0) {
        // Rotate left
//        analogWrite(leftPin, 0);
//        analogWrite(rightPin, (angle - refAngle) * 4  + 80);

        // Differential: Turn left
        analogWrite(leftPin, MOTOR_PWM - (angle - refAngle) * MOTOR_PWM/ 90 );
        analogWrite(rightPin, MOTOR_PWM );
        
        //Serial.println("Rotate left");

        digitalWrite(LED, HIGH);
        digitalWrite(LED2, LOW);
      }
      else if ((refAngle - angle) > 0) {


        // Rotate right

//
//        analogWrite(rightPin, 0);
//        analogWrite(leftPin, (refAngle - angle) * 4  + 80);
        //Serial.println("Rotate right");
        
        // Differential: Turn right
        analogWrite(leftPin, MOTOR_PWM );
        analogWrite(rightPin, MOTOR_PWM  - (angle - refAngle) * MOTOR_PWM/ 90  );

        digitalWrite(LED, LOW);
        digitalWrite(LED2, HIGH);

      }
      else {
//        // Stop
//        analogWrite(rightPin, 0);
//        analogWrite(leftPin, 0);


        // Differential: moveforward
        analogWrite(leftPin, MOTOR_PWM );
        analogWrite(rightPin, MOTOR_PWM  );
        

        digitalWrite(LED, HIGH);
        digitalWrite(LED2, HIGH);
        //Serial.println("Straight");

      }

      
//    Serial.print(IRFrontFlag );
//    Serial.print(" " );
//    Serial.print(IRFront );
//    
      // IR reading
      if (IRFrontFlag && !IRFront) {
        refAngle += 90;
        refAngle = (refAngle > 180)? (refAngle - 180) + -180:
                   (refAngle < -180)? (refAngle - -180) + 180:
                   refAngle  ;
        IRFrontFlag = 0;
      } else if (!IRFrontFlag && IRFront) {
        IRFrontFlag = 1;

      }
    }




  }
}
