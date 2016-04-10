
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
int readAnglePin = 11;
int flag = 1;
int readAngle;
int refAngle = 0;
int angle ;
int LED = 10;
int LED2 = 8;

// IR Variables
int IRFrontPin = 12;
int IRFront = 1;
int IRFrontFlag = 0 ;


int IRLeftPin = 9;
int IRRightPin = 3;
int IRLeft;
int IRRight ;
int IRLeftEnable = 1 ;
int IRRightEnable = 1 ;
int IRLeftFlag = 1;
int IRRightFlag = 1;

float rightTurnSpeed = 0;
float leftTurnSpeed = 0;
float turnDelay = 4;

#define MOTOR_PWM  ( 255  )
#define MAX255(X,Y) ( (X-Y)>255?255:X-Y )
#define LIMIT(X) ( X>255?255: (X<0)?0 :X )

int leftMotorDir = 7;
int leftPin = 6;
int leftPinOp = 20;//NOT USED
int rightPin = 5;
int rightMotorDir = 4;

// No. of laps
int lapsNo = 0;
int linesNo = 0;
int lapsNoInput = 1;
int lineFlag = 0;
int endFlag = 0 ;

#define RightForward    digitalWrite(rightMotorDir, LOW)
#define RightBackward    digitalWrite(rightMotorDir, HIGH)
#define LeftForward    digitalWrite(leftMotorDir, LOW)
#define LeftBackward    digitalWrite(leftMotorDir, HIGH)
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  /******************** Program setup **************************/

  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(readAnglePin, INPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(leftPinOp, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(IRLeftPin, INPUT);
  pinMode(IRRightPin, INPUT);
  pinMode(IRFrontPin, INPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);

  analogWrite(leftPin, 0 );
  analogWrite(leftPinOp, 0 );
  analogWrite(rightPin, 0  );

  // For switches
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  RightForward;
  LeftForward;

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

/**
   Returns the difference from angle to reference.
   If diff is positve: Turn left.
   If diff is negative: Turn right.
   If 0. On track.
*/
int absDiff(int angle, int ref) {
  angle = angle + 180;
  ref = ref + 180;

  int diff = angle - ref;
  if (diff > 0 && diff < 180) {
    return diff;
  } else if (diff > 180) {
    return -(360 - diff);
  } else if (diff < 0 && diff > -180) {
    return diff;
  } else if (diff < -180) {
    return (360 - -diff);
  } else return 0;

}
#define WHITE 0
#define BLACK 1
long int delayLong = 0;
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
    IRLeft = digitalRead(IRLeftPin);
    IRRight = digitalRead(IRRightPin);

    //    if (delayLong++ > 30000 ) { flag = 0;
    //      }
    if ( flag) {

      readAngle = digitalRead(readAnglePin);

      //  Serial.print(delayLong/300);
      // Serial.print(" ");

      Serial.println(angle);


      digitalWrite(LED, (angle & 0b01) ? HIGH : LOW);

      digitalWrite(LED2, (angle & 0b10) ? HIGH : LOW);
      // refAngle = angle;

    }
    else {
      readAngle = 0;
      readAngle = digitalRead(readAnglePin);

      // Print ref
      //            Serial.print("Ref = ");
      //            Serial.print(refAngle);
      //
      //            // Print diff
      //            Serial.print("\tDiff = ");
      //            Serial.println(refAngle - angle);

    }



    if (readAngle == 0) {
      refAngle = angle;
      flag = 0;

      lapsNo = 0;
      linesNo = 0;
      //lapsNoInput = 1;
      lineFlag = 0;
      endFlag = 0 ;
      IRFrontFlag = 0;
      lapsNoInput = digitalRead(A3) * 1 +
                    digitalRead(A2) * 2 +
                    digitalRead(A1) * 4 +
                    digitalRead(A0) * 8 ;
      Serial.print("\t\t\t ");
      Serial.println(lapsNoInput);
      analogWrite(leftPinOp, 0 );
      RightForward;
      LeftForward;
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
    if (endFlag == 1) {
      if (delayLong++ < 300) {
        //Serial.println(delayLong);
        // return;
      } else {
        analogWrite(leftPin, 0 );
        analogWrite(rightPin, 0  );
        return;
      }
    }

    // -------------------- Left and Right IR -------- //
    if ( refAngle) {
      if (IRLeft == WHITE /* && IRLeftFlag == 1 */) {
        if ( IRLeftEnable == 1 ) {
          rightTurnSpeed = -250;
        }
        IRLeftFlag = 0;
      }
      if ( IRLeft == BLACK  && IRLeftFlag == 0 ) {
        IRLeftFlag = 1;
        IRLeftEnable = 1;

        refAngle += 2;            // Add 3 to adjust the gyro offset
        refAngle = ( refAngle < -179 ) ?  (refAngle - -180) + 180 :
                   (refAngle > 180) ? (refAngle - 180) + -180 :
                   refAngle;
      }


      if (IRRight == WHITE /* && IRRightFlag == 1*/) {
        if ( IRRightEnable == 1 ) {
          leftTurnSpeed = -250;
        }
        IRRightFlag = 0;
      }
      if (IRRight == BLACK && IRRightFlag == 0) {
        IRRightFlag = 1;
        IRRightEnable = 1   ;

        refAngle -= 2;            // Add 3 to adjust the gyro offset
        refAngle = ( refAngle < -179 ) ?  (refAngle - -180) + 180 :
                   (refAngle > 180) ? (refAngle - 180) + -180 :
                   refAngle;
      }
    }
    if (rightTurnSpeed < 0) rightTurnSpeed += turnDelay; else rightTurnSpeed = 0;
    if (leftTurnSpeed < 0) leftTurnSpeed += turnDelay; else leftTurnSpeed = 0;
    //
    //        Serial.print(IRLeftEnable);
    //        Serial.print("   ");
    //        Serial.print(IRLeftFlag);
    //        Serial.print("  \t ");
    //        Serial.print(IRRightFlag);
    //        Serial.print("   ");
    //        Serial.println(IRRightEnable);

    Serial.print(rightTurnSpeed);
    Serial.print("   ");
    Serial.println(leftTurnSpeed);
    // ------------------------------------------------------------------//

    int angleDiff = absDiff(angle, refAngle) ;
    if ( refAngle && !IRFrontFlag) {

      // First, left and right IR sensors:
      if (leftTurnSpeed < 0 || rightTurnSpeed < 0 ) {

        analogWrite(leftPin, MOTOR_PWM + leftTurnSpeed );
        analogWrite(rightPin, MOTOR_PWM + rightTurnSpeed );

      } else
        // If there are no problem:

        if (angleDiff > 0) {
          // Turn left

          if (angleDiff < 34) {
            LeftForward;
            analogWrite(leftPin, LIMIT((MOTOR_PWM - angleDiff * MOTOR_PWM / 34 ))    );
          }
          else {
            LeftBackward;
            analogWrite(leftPin, LIMIT((  (angleDiff - 34 ) * (angleDiff - 34) )) );
          }

          RightForward;
          analogWrite(rightPin, MOTOR_PWM );

          digitalWrite(LED, HIGH);
          digitalWrite(LED2, LOW);

        }
        else if (angleDiff < 0) {
          // Turn right
          LeftForward;
          analogWrite(leftPin, MOTOR_PWM);

          if ( angleDiff > -34 ) {
            RightForward;
            analogWrite(rightPin, LIMIT((MOTOR_PWM - -angleDiff *  MOTOR_PWM / 34))   );
          }
          else {
            RightBackward;
            analogWrite(rightPin, LIMIT((  (-angleDiff - 34 ) * (-angleDiff - 34) ))  );
          }

          //
          //          if ( angleDiff > -34 )
          //            analogWrite(rightPin, ((MOTOR_PWM  - (-angleDiff) * MOTOR_PWM / 34))   );
          //          else
          //            analogWrite(rightPin, 0 );

          digitalWrite(LED, LOW);
          digitalWrite(LED2, HIGH);

        }
        else {

          // Differential: moveforward
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, MOTOR_PWM );


          digitalWrite(LED, HIGH);
          digitalWrite(LED2, HIGH);
          //Serial.println("Straight");

        }

      // IR reading
      if (!IRFront) {
        IRFrontFlag = 1;
        lineFlag = 1;
        linesNo++;

        Serial.println(linesNo);
        if (linesNo % 6 != 0 && linesNo % 6 != 1)
        {
          Serial.println("Rotating" );


          refAngle -= 90;
          refAngle = ( refAngle < -179 ) ?  (refAngle - -180) + 180 :
                     (refAngle > 180) ? (refAngle - 180) + -180 :
                     refAngle;
          if (linesNo % 6 == 5) {

            lapsNo++;
//            refAngle += 3;              // Add 3 to adjust the gyro offset
//            refAngle = ( refAngle < -179 ) ?  (refAngle - -180) + 180 :
//                       (refAngle > 180) ? (refAngle - 180) + -180 :
//                       refAngle;

          }

        }
      }



    } else if ( IRFrontFlag ) {

      // Either turn, or skip (stop)
      if ( linesNo % 6 != 0 && linesNo % 6 != 1 ) {
        // Turn
        // Serial.println("Rotating" );
        if (angleDiff > 30 && linesNo % 6  != 5) {
          // Turning Left
          LeftBackward;
          RightForward;
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, MOTOR_PWM  );
          digitalWrite(LED, HIGH);
          digitalWrite(LED2, LOW);
        }

        else if ( angleDiff < -30 && linesNo % 6  != 5) {
          // Turn Right

          LeftForward;
          RightBackward;
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, MOTOR_PWM  );
          digitalWrite(LED, LOW);
          digitalWrite(LED2, HIGH);

        }

        // HARDCODING FOR THE LAST TURN
        else if (angleDiff > 10 && linesNo % 6  == 5) {
          // Turning Left

          if(angleDiff < 25){
            RightForward;
            LeftForward;
          analogWrite(leftPin, 120 );
          analogWrite(rightPin, MOTOR_PWM  );
          
          }
          else {
          LeftBackward;
          RightForward;
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, MOTOR_PWM  );
          digitalWrite(LED, HIGH);
          digitalWrite(LED2, LOW);
          }
        }

        else if ( angleDiff < -10 && linesNo % 6  == 5) {
          // Turn Right

          if(angleDiff < 25){
            RightForward;
            LeftForward;
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, 120  );
          
          }
          else {
          LeftForward;
          RightBackward;
          analogWrite(leftPin, MOTOR_PWM );
          analogWrite(rightPin, MOTOR_PWM  );
          digitalWrite(LED, LOW);
          digitalWrite(LED2, HIGH);
          }
        }
          //  END OF HARDCODING FOR THE LAST TURN



          else  {

            LeftForward;
            RightForward;
            //          analogWrite(leftPin, 0 );
            //          analogWrite(leftPinOp, 0 );
            //          analogWrite(rightPin, 0  );
            IRFrontFlag = 0;

            // So the car won't continue turning;
            leftTurnSpeed = 0;
            rightTurnSpeed = 0;
          }

          if (linesNo % 6 == 5) {
            // Completed a lap
            //lapsNo++;
          }

        } else  if (lineFlag == 1) {
          // Skip or Stop
          lineFlag = 2;

          if (linesNo % 6 == 1) {
            IRRightEnable = 0;
            IRLeftEnable = 0;
            //  Skip this white ine
            return;
          } else if ( linesNo % 6 == 0 ) {
            // Stop line

            IRRightEnable = 0;
            IRLeftEnable = 0;


            if ( lapsNo >= lapsNoInput ) {
              analogWrite(leftPin, MOTOR_PWM );
              analogWrite(rightPin, MOTOR_PWM  );
              endFlag = 1;
              delayLong = 0;
              return;
            }
            else {
              return;
            }
          }



        } else {
          if (IRFront) IRFrontFlag = 0;
        }



      } else {

        analogWrite(leftPin, 0 );
        analogWrite(rightPin, 0  );
      }




    }
  }
