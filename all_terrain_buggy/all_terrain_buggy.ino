/*
 * All-Terrain Buggy
 * M5 Demonstration 
 * Firmware for interfacing with buggy and remote control 
 * Authors: Akshat Sahay, Chris Caron, Sebastian Armstrong 
 */

#define CHANNEL_1 5 // channel 1 of the controller
#define CHANNEL_2 6 // channel 2 of the controller

/* driver pins for TB6612*/
#define AIN1 2
#define AIN2 3
#define PWMA A1

#define BIN1 9
#define BIN2 10
#define PWMB A2

#define DEBUG 0 // print signal values

int sig1 = 0; // CHANNEL_1 value
int sig2 = 0; // CHANNEL_2 value
int a_speed = 0; // speed of A side
int b_speed = 0; // speed of B side
int offset = 0;

int getState(int sig1, int sig2); 

void setup() {
  pinMode(CHANNEL_1, INPUT); // set channel 1 to INPUT
  pinMode(CHANNEL_2, INPUT); // set channel 2 to INPUT

  if(DEBUG == 1) {
    // print signal values
    Serial.print(sig1);
    Serial.print(" ");
    Serial.println(sig2);    
  }

  Serial.begin(9600); // start serial port 
}

void loop() {
  sig1 = pulseIn(CHANNEL_1, HIGH);
  sig2 = pulseIn(CHANNEL_2, HIGH);

  // states: OFF = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4 
  if(((sig1 < 1560) && (sig1 > 1500)) && ((sig2 < 1560) && (sig2 > 1500))) {
    // both signals are 0, state: OFF
    Serial.println("OFF"); 

    // set driver to STOP mode 
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); 
  }

  else if((sig1 > 1560) && ((sig2 < 1560) && (sig2 > 1500))) {
    // sig1 is 1, sig2 is 0, state: FORWARD
    Serial.println("FORWARD"); 

    // set driver to CLOCKWISE mode
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW); 
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW); 

    // limit sig1 values for speed
    if(sig1 > 2000) sig1 = 2000; 
    if(sig1 < 1000) sig1 = 1000;
    
    a_speed = map(sig1, 1000, 2000, 0, 255); 
    analogWrite(PWMA, a_speed);
    analogWrite(PWMB, a_speed); 
  }

  else if((sig1 < 1500) && ((sig2 < 1560) && (sig2 > 1500))) {
    // sig1 is -1, sig2 is 0, state: BACKWARD
    Serial.println("BACKWARD"); 

    // set driver to COUNTER-CLOCKWISE mode
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH); 

    // limit sig1 values for speed
    if(sig1 > 2000) sig1 = 2000; 
    if(sig1 < 1000) sig1 = 1000;

    a_speed = map(sig1, 1000, 2000, 0, 255); 
    analogWrite(PWMA, a_speed);
    analogWrite(PWMB, a_speed);       
  }

  else if(sig2 > 1500) {
    // sig1 is X, sig2 is 1, state: RIGHT
    Serial.println("RIGHT"); 

    if(sig1 < 1500) {
      // NORTHEAST
      // set driver to COUNTER-CLOCKWISE mode
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW); 
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW); 
  
      // limit sig1 values for speed
      if(sig1 > 2000) sig1 = 2000; 
      if(sig1 < 1000) sig1 = 1000;

      // limit sig1 values for speed
      if(sig2 > 2000) sig2 = 2000; 
      if(sig2 < 1000) sig2 = 1000;
  
      b_speed = map(sig1, 1000, 2000, 0, 255); 
      offset = map(sig2, 1000, 2000, 0, 100); 
      a_speed = a_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }

    if(sig1 > 1560) {
      // NORTHEAST
      // set driver to COUNTER-CLOCKWISE mode
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH); 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH); 
  
      // limit sig1 values for speed
      if(sig1 > 2000) sig1 = 2000; 
      if(sig1 < 1000) sig1 = 1000;

      // limit sig1 values for speed
      if(sig2 > 2000) sig2 = 2000; 
      if(sig2 < 1000) sig2 = 1000;
  
      b_speed = map(sig1, 1000, 2000, 0, 255); 
      offset = map(sig2, 1000, 2000, 0, 100); 
      a_speed = a_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }
  }

  else if(sig2 < 1560) {
    // sig1 is X, sig2 is -1, state: LEFT
    Serial.println("LEFT"); 
    
  }

  else if(sig1 == 0 || sig2 == 0) {
    Serial.println("ERROR"); 
  }
}
