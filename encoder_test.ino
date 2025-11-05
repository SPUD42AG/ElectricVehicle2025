/*
BISO ELECTRIC VEHICLE CODE 2025 - 2026

CODE TO TEST ENCODER
*/

// Encoder connect to digitalpin 2 and 3 on the Arduino.

volatile unsigned int counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile unsigned int temp;

void setup() {
  Serial.begin (9600);
 
  pinMode(2, INPUT);           // set pin to input
  pinMode(3, INPUT);           // set pin to input
  
  digitalWrite(2, HIGH);       // turn on pullup resistors
  digitalWrite(3, HIGH);       // turn on pullup resistors
 
 
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on most Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on most Arduino.
  attachInterrupt(1, ai1, RISING);
}

void loop() {
  // Send the value of counter
  
  if (counter != temp){
    Serial.println (counter);
    temp = counter;
  }
  
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}
			
