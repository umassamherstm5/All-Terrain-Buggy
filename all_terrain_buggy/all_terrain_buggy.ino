/*
 * All-Terrain Buggy
 * M5 Demonstration 
 * Firmware for interfacing with buggy and remote control 
 * Authors: Akshat Sahay, Chris Caron, Sebastian Armstrong 
 */

#define CHANNEL_1 A1 // channel 1 of the controller
#define CHANNEL_2 A2 // channel 2 of the controller

/* driver pins for TB6612, A is right and B is left */
#define AIN1 2
#define AIN2 3
#define PWMA 10

#define BIN1 5
#define BIN2 6
#define PWMB 9

/* thresholds for signal values, used in loop */
#define SIG_HIGH_ACC 1490
#define SIG_LOW_ACC 1410

#define SIG_HIGH_STEER 1515
#define SIG_LOW_STEER 1435

#define DEBUG 1  // print signal values

int sig1 = 0; // CHANNEL_1 value
int sig2 = 0; // CHANNEL_2 value

int a_speed = 0; // speed of A side
int b_speed = 0; // speed of B side
int offset  = 0; // speed difference for turning

void setup() {
  // set RC channels to INPUT
  pinMode(CHANNEL_1, INPUT); 
  pinMode(CHANNEL_2, INPUT); 

  // set driver pins to OUTPUT
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT); 
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT); 
  pinMode(PWMA, OUTPUT); 
  pinMode(PWMB, OUTPUT); 

  Serial.begin(9600); // start serial port 
}

void loop() {
  // get signal values from receiver 
  sig1 = pulseIn(CHANNEL_1, HIGH);
  sig2 = pulseIn(CHANNEL_2, HIGH);

  if(DEBUG == 1) {
    // print signal values
    Serial.print(sig1);
    Serial.print(" ");
    Serial.println(sig2);    
  }

  // states: OFF = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4 
  if(((sig1 < SIG_HIGH_ACC) && (sig1 > SIG_LOW_ACC)) && ((sig2 < SIG_HIGH_STEER) && (sig2 > SIG_LOW_STEER))) {
    // both signals are 0, state: OFF
    Serial.println("OFF"); 

    // set driver to STOP mode 
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); 
  }

  else if((sig1 > SIG_HIGH_ACC) && ((sig2 < SIG_HIGH_STEER) && (sig2 > SIG_LOW_STEER))) {
    // sig1 is 1, sig2 is 0, state: FORWARD
    Serial.println("FORWARD"); 

    // set driver to CLOCKWISE mode
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW); 
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW); 

    // limit sig1 values for speed
    if(sig1 > 1850) sig1 = 1850; 

    // get speed proportional to sig1 in FORWARD state
    // the higher the sig1 val, the higher the speed
    a_speed = map(sig1, SIG_HIGH_ACC, 1850, 0, 255); 
    analogWrite(PWMA, a_speed);
    analogWrite(PWMB, a_speed); 
  }

  else if((sig1 < SIG_LOW_ACC) && ((sig2 < SIG_HIGH_STEER) && (sig2 > SIG_LOW_STEER))) {
    // sig1 is -1, sig2 is 0, state: BACKWARD
    Serial.println("BACKWARD"); 

    // set driver to COUNTER-CLOCKWISE mode
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH); 

    // limit sig1 values for speed
    if(sig1 < 1000) sig1 = 1000;

    // get speed proportional to sig1 in BACKWARD state
    // the lower the sig1 val, the higher the speed
    a_speed = map(sig1, SIG_LOW_ACC, 1000, 0, 255); 
    analogWrite(PWMA, a_speed);
    analogWrite(PWMB, a_speed);       
  }

  else if((sig2 < SIG_LOW_STEER) && sig2 > 0) {
    // sig1 is X, sig2 is 1, state: RIGHT
    Serial.print("RIGHT "); 

    if(sig1 > SIG_HIGH_ACC) {
      Serial.println("FORWARD");  
      
      // set driver to CLOCKWISE mode
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW); 
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW); 
  
      // limit sig1 values for speed
      if(sig1 > 1850) sig1 = 1850; 
      // limit sig2 values for speed
      if(sig2 < 1000) sig2 = 1000;
  
      b_speed = map(sig1, SIG_HIGH_ACC, 1850, 0, 255); 
      offset  = map(sig2, SIG_LOW_STEER, 1000, 0, b_speed/2); 
      // to go RIGHT, left side faster than right side 
      a_speed = b_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }

    else if(sig1 < SIG_LOW_ACC) {
      Serial.println("BACKWARD"); 
      
      // set driver to COUNTER-CLOCKWISE mode
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH); 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH); 
  
      // limit sig1 values for speed
      if(sig1 < 1000) sig1 = 1000;
      // limit sig2 values for speed
      if(sig2 < 1000) sig2 = 1000;
  
      b_speed = map(sig1, SIG_LOW_ACC, 1000, 0, 255); 
      offset  = map(sig2, SIG_LOW_STEER, 1000, 0, b_speed/2);
      // to go RIGHT, left side faster than right side 
      a_speed = b_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }

    else {
      Serial.println("SPIN");

      // set driver to SPIN, both sides move opposite dirs
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH); 
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);

      // speed dependent solely on sig2 value
      a_speed = map(sig2, SIG_LOW_STEER, 1000, 0, 255);
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, a_speed);   
    }
  }

  else if(sig2 > SIG_HIGH_STEER) {
    // sig1 is X, sig2 is -1, state: LEFT
    Serial.print("LEFT "); 

    if(sig1 > SIG_HIGH_ACC) {
      Serial.println("FORWARD"); 
      // set driver to CLOCKWISE mode
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW); 
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW); 
  
      // limit sig1 values for speed
      if(sig1 > 1850) sig1 = 1850; 
      // limit sig1 values for speed
      if(sig2 > 1950) sig2 = 1950; 
  
      a_speed = map(sig1, SIG_HIGH_ACC, 1850, 0, 255); 
      offset  = map(sig2, SIG_HIGH_STEER, 1950, 0, a_speed/2); 
      // to go LEFT, right side faster than left
      b_speed = a_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }

    else if(sig1 < SIG_LOW_ACC) {
      Serial.println("BACKWARD");  
      // set driver to COUNTER-CLOCKWISE mode
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH); 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH); 
  
      // limit sig1 values for speed
      if(sig1 < 1000) sig1 = 1000;
      // limit sig1 values for speed
      if(sig2 > 1950) sig2 = 1950; 
  
      a_speed = map(sig1, SIG_LOW_ACC, 1000, 0, 255); 
      offset  = map(sig2, SIG_HIGH_STEER, 1950, 0, a_speed/2); 
      // to go LEFT, right side faster than left
      b_speed = a_speed - offset; 
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, b_speed);  
    }

    else {
      Serial.println("SPIN");

      // set driver to SPIN, both sides move opposite dirs
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW); 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);

      // speed dependent solely on sig2 value
      a_speed = map(sig2, SIG_HIGH_STEER, 1950, 0, 255);
      analogWrite(PWMA, a_speed);
      analogWrite(PWMB, a_speed);   
    }
  }

  else {
    Serial.println("ERROR"); 
    // set driver to STOP mode 
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}
