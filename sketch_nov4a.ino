/*
BISO ELECTRIC VEHICLE 2025-2026

MAIN VEHICLE CODE
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <RobojaxBTS7960.h>  //Download this library from the instructions page
#include <Servo.h>
#include <math.h>


//LCD SETUP------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD I2C address


//MOTOR DRIVER PINS+SETUP------------------------------------
#define RPWM 10  // define pin for RPWM pin (output)
#define R_EN 11  // define pin for R_EN pin (input)
#define R_IS 13  // define pin for R_IS pin (output)

#define LPWM 9   // define pin for LPWM pin (output)
#define L_EN 8   // define pin for L_EN pin (input)
#define L_IS 12  // define pin for L_IS pin (output)

#define CW 1   //defines CW motor movement
#define CCW 0  //defines CCW motor movement

#define debug 1  //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

RobojaxBTS7960 motor(R_EN, RPWM, R_IS, L_EN, LPWM, L_IS, debug);


//ENCODER PINS------------------------------------
#define ENCA 2  //Vehicle encoder pin
#define ENCB 3  //Vehicle encoder pin

//LCD Encoder Pins
#define DialCLK 5  //increase/decrease time/dist
#define DialDT 4   //increase/decrease time/dist


//PUSH BUTTON PINS------------------------------------
#define StartButtonPin 6  //Start vehicle
#define SetButtonPin 7    //Set target dist
#define DialButtonPin A0  //Change adjustment interval


//ENCODER VARIABLES------------------------------------
volatile unsigned long counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile unsigned long temp;


//LCD VARIABLES------------------------------------
int pos = 2;    //increment position in rotation
float inc = 1;  //increment value

int set = 0;


//MOVEMENT VARIABLES------------------------------------
double TargetDistance = 10.000;                  //Target Distance in m
double TargetDistanceCM = TargetDistance * 100;  //Target Distance in cm
double ArcLength;                                //Actual travel distance
double TargetTime = 20.00;                       //Target Time in s
double wheelDiameter = 7.3025;                   //Wheel Diameter is 2.875in = 7.3025 cm
double wheelCircumfrence = wheelDiameter * 3.14159;
double slowDownDistance;     //Distance that initial slowing is initiated
double snailDistance;        //Distance that secondary slowing is initiated
double pulsesPerRev = 1200;  //Encoder PPR

bool SBpressed = false;
bool moved = false;
bool slowed = false;
bool snail = false;
bool reachedTargetDistance = false;

double targetEncoderValue;
double slowDownEncoderValue;
double maxEncoderValue;
double encoderChange;
double finalDist;


//TIMER VARIABLES------------------------------------
long startTime;
long currentTime;
long endTime;

double timeDiff;
double milisTime;
double runTime;


void setup() {
  
  Serial.begin(9600);
  Serial.println("Entered Setup");


  //MOTOR SETUP----------------------------------------------------
  motor.begin();
  Serial.println("Motor Setup Complete");


  //LCD SETUP----------------------------------------------------
  set = 0;
  pos = 2;
  inc = 1;

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Tgt Dist:      m");
  lcd.setCursor(9, 0);
  lcd.print(TargetDistance);
  lcd.setCursor(0, 1);
  lcd.print("Increment: ");
  lcd.print(inc);

  Serial.println("LCD Setup Complete");


  //LCD BUTTON SETUP----------------------------------------------------
  pinMode(SetButtonPin, INPUT);
  digitalWrite(SetButtonPin, HIGH);  //enable pullups to make pin high

  Serial.println("LCD Button Setup Complete");


  //BUTTON SETUP----------------------------------------------------
  pinMode(StartButtonPin, INPUT);
  digitalWrite(StartButtonPin, HIGH);  //enable pullups to make pin high

  Serial.println("Button Setup Complete");


  //LCD ENCODER SETUP----------------------------------------------------
  pinMode(DialCLK, INPUT);  // set pin to input
  pinMode(DialDT, INPUT);   // set pin to input

  digitalWrite(DialCLK, HIGH);  // turn on pullup resistors
  digitalWrite(DialDT, HIGH);   // turn on pullup resistors


  //ENCODER SETUP----------------------------------------------------
  pinMode(ENCA, INPUT);  // set pin to input
  pinMode(ENCB, INPUT);  // set pin to input

  digitalWrite(ENCA, HIGH);  // turn on pullup resistors
  digitalWrite(ENCB, HIGH);  // turn on pullup resistors

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on most Arduino.
  attachInterrupt(0, ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on most Arduino.
  attachInterrupt(1, ai1, RISING);

  Serial.println("Encoder Setup Complete");


  //TARGET ENCODER VALUES SETUP----------------------------------------------------
  targetEncoderValue = getEncoderValue(TargetDistance, wheelDiameter);
  maxEncoderValue = targetEncoderValue * 1.5;

  Serial.println(targetEncoderValue);
  Serial.println(maxEncoderValue);

  Serial.println("Target Values Set");


  //VARIABLE SETUP----------------------------------------------------
  SBpressed = false;
  moved = false;
  slowed = false;
  snail = false;
  reachedTargetDistance = false;

  Serial.println("Variable Reset Complete");

  Serial.println("Setup Complete");
}

void loop() {

  if (set == 0) {
    Serial.println(analogRead(DialButtonPin));

    if (digitalRead(SetButtonPin) == LOW) {
      Serial.println("SET pressed - Dist Set");

      TargetDistanceCM = TargetDistance * 100;
      ArcLength = getArcLength(TargetDistanceCM);
      slowDownDistance = ArcLength - 100;

      Serial.println(ArcLength);

      targetEncoderValue = getEncoderValue(ArcLength, wheelDiameter);
      slowDownEncoderValue = getEncoderValue(slowDownDistance, wheelDiameter);
      maxEncoderValue = getEncoderValue(ArcLength * 2, wheelDiameter);

      Serial.println(targetEncoderValue);
      Serial.println(slowDownEncoderValue);
      Serial.println(maxEncoderValue);

      lcd.setCursor(0, 0);
      lcd.print("Tgt Time:      s");
      lcd.setCursor(9, 0);
      lcd.print(TargetTime, 3);
      inc = 5;
      pos = 2;
      lcd.setCursor(11, 1);
      lcd.print("     ");
      lcd.setCursor(11, 1);
      lcd.print(inc);

      set = 1;

      delay(300);
    }


    if (analogRead(DialButtonPin) == 0) {
      if (pos == 1) {
        inc = 1;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 2) {
        inc = 0.5;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 3) {
        inc = 0.1;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 4) {
        inc = 0.01;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 5) {
        inc = 0.001;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos = 1;
      }
    }


    if (digitalRead(DialDT) == LOW) {
      TargetDistance -= inc;
      lcd.setCursor(9, 0);
      lcd.print("      ");
      lcd.setCursor(9, 0);
      lcd.print(TargetDistance, 3);
      delay(250);
    }


    if (digitalRead(DialCLK) == LOW) {
      TargetDistance += inc;
      lcd.setCursor(9, 0);
      lcd.print("      ");
      lcd.setCursor(9, 0);
      lcd.print(TargetDistance, 3);
      delay(250);
    }
  }


  if (set == 1) {
    Serial.println(analogRead(DialButtonPin));

    if (digitalRead(SetButtonPin) == LOW) {
      Serial.println("SET pressed - Time Set");

      lcd.setCursor(0, 0);
      lcd.print("Tgt Dist:      m");
      lcd.setCursor(9, 0);
      lcd.print(TargetDistance, 3);
      lcd.setCursor(0, 1);
      lcd.print("Tgt Time:      s");
      lcd.setCursor(9, 1);
      lcd.print(TargetTime, 3);

      set = 2;

      delay(300);
    }

    if (analogRead(DialButtonPin) == 0) {
      if (pos == 1) {
        inc = 5;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 2) {
        inc = 1;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 3) {
        inc = 0.5;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 4) {
        inc = 0.1;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos++;
      } else if (pos == 5) {
        inc = 0.01;
        lcd.setCursor(11, 1);
        lcd.print("    ");
        lcd.setCursor(11, 1);
        lcd.print(inc, 3);
        delay(400);
        pos = 1;
      }
    }


    if (digitalRead(DialDT) == LOW) {
      TargetTime -= inc;
      lcd.setCursor(9, 0);
      lcd.print("      ");
      lcd.setCursor(9, 0);
      lcd.print(TargetTime, 3);
      delay(250);
    }


    if (digitalRead(DialCLK) == LOW) {
      TargetTime += inc;
      lcd.setCursor(9, 0);
      lcd.print("      ");
      lcd.setCursor(9, 0);
      lcd.print(TargetTime, 3);
      delay(250);
    }
  }


  if (set == 2) {

    if (digitalRead(SetButtonPin) == LOW) {
      Serial.println("SET pressed - Set Dist/Time");

      inc = 1;
      pos = 2;

      lcd.setCursor(0, 0);
      lcd.print("Tgt Dist:      m");
      lcd.setCursor(9, 0);
      lcd.print(TargetDistance);
      lcd.setCursor(0, 1);
      lcd.print("Increment: ");
      lcd.print(inc);

      set = 0;

      delay(300);
    }

    if (digitalRead(StartButtonPin) == LOW) {
      //This section of code resets the movement bools and counter and records the starting time
      Serial.println("Start pressed");
      moved = false;
      slowed = false;
      snail = false;
      reachedTargetDistance = false;
      counter = 0;
      startTime = millis();
      Serial.println(startTime);
      SBpressed = true;
    }

    if (SBpressed) {
      //This section of code turns the motor on
      motor.rotate(50, CW);  // run motor with 30% speed in CW direction
      moved = true;
      SBpressed = false;
    }

    if (moved) {
      //This section of code keeps the motor running until the vehicle reaches the slow down dist
      if (counter >= slowDownEncoderValue && counter < maxEncoderValue) {
        Serial.println("Reached Decel Distance");
        //This section of code stops the motor and checks run time
        motor.stop();  // stop the motor
        motor.rotate(30, CCW);
        delay(100);
        motor.stop();
        delay(500);

        currentTime = millis();
        Serial.println(currentTime);
        milisTime = currentTime - startTime;
        runTime = milisTime / 1000.00;
        timeDiff = TargetTime - runTime;

        Serial.println(timeDiff);

        slowed = true;
        moved = false;
      }
    }

    if (slowed) {
      /*
      This section of code initiates a set of small pulses.
      These pulses are averaged and used to determine the total number needed to reach the target dist.
      By knowing the amount of pulses needed, the interval between pulses can be adjusted to achieve the target time.
      */
      encoderChange = 0;
      double initalEncoderVal;
      double finalEncoderVal;

      //Performs 5 pulses at 0.4 secs per pulse
      for (int i = 0; i < 5; i++) {
        initalEncoderVal = counter;
        motor.rotate(10, CW);
        delay(200);
        motor.stop();
        delay(200);
        finalEncoderVal = counter;

        encoderChange += (finalEncoderVal - initalEncoderVal);
      }

      //average distance change per pulse
      encoderChange = encoderChange / 5;

      //check run time
      currentTime = millis();
      Serial.println(currentTime);
      milisTime = currentTime - startTime;
      runTime = milisTime / 1000.00;
      timeDiff = TargetTime - runTime;

      Serial.println(timeDiff);

      snail = true;
      slowed = false;
    }


    if (snail) {
      //This sections of code determines number of pulses needed to reach TD
      //It also determines the interval length needed to reach the target time
      double remainingDist = targetEncoderValue - counter;
      double pulses = remainingDist / encoderChange;
      double pulseTime = pulses * 0.2;
      double nonPulseTime = timeDiff - pulseTime;
      if (nonPulseTime < 0) {
        nonPulseTime = 0;
      }
      double pulseDelay = 1000 * (nonPulseTime / (pulses - 1));  //delay in ms

      //initiate pulses
      for (int i = 0; i < pulses; i++) {
        motor.rotate(10, CW);
        delay(200);
        motor.stop();
        delay(pulseDelay);
      }

      //check run time
      endTime = millis();
      Serial.println(endTime);
      milisTime = endTime - startTime - pulseDelay;
      runTime = milisTime / 1000.00;
      Serial.println(runTime);

      snail = false;
      reachedTargetDistance = true;
    }

    if (reachedTargetDistance) {
      //This section of code records actual dist and time for debugging purposes

      delay(1000);  //wait 1 sec to make sure all movement has stopped

      Serial.println("Reached Target Distance Loop");
      Serial.println(counter);

      //determine distance error from run
      finalDist = TargetDistance * (counter / targetEncoderValue);

      //print time/dist on LCD
      lcd.setCursor(0, 0);
      lcd.print("Fin Dist:      m");
      lcd.setCursor(9, 0);
      lcd.print(finalDist, 3);
      lcd.setCursor(0, 1);
      lcd.print("Fin Time:      s");
      lcd.setCursor(9, 1);
      lcd.print(runTime, 3);

      //reset all bools + counter
      SBpressed = false;
      moved = false;
      slowed = false;
      snail = false;
      reachedTargetDistance = false;
      counter = 0;
    }
  }
}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}

double getEncoderValue(double TD, double WD) {
  double PPR = 1200;
  double WheelCircumfrence = 3.14159 * WD;
  double tgtENCval = (TD / WheelCircumfrence) * PPR;
  return tgtENCval;
}

double getArcLength(double TargetDist) {
  double vehicleWidth = 13.00;
  double canBonusError = 2.00;
  double arcHeight = 100.00 - (vehicleWidth + canBonusError) / 2;
  double arcRadius = ((arcHeight / 2) + (sq(TargetDist) / (8 * arcHeight)));
  double arcAngle = 2 * asin(TargetDist / (2 * arcRadius));
  double arcLength = arcRadius * arcAngle;
  return arcLength;
}
