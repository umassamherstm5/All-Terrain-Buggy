/*
 * All-Terrain Buggy
 * M5 Demonstration 
 * Firmware for interfacing with buggy and remote control 
 * Author: Akshat Sahay, Chris Caron, Sebastian Armstrong 
 */

#define CHANNEL_1 5 // channel 1 of the controller
#define CHANNEL_2 6 // channel 2 of the controller 

int sig1 = 0;
int sig2 = 0; 

int getState(int sig1, int sig2); 

void setup() {
  pinMode(CHANNEL_1, INPUT); // set channel 1 to INPUT
  pinMode(CHANNEL_2, INPUT); // set channel 2 to INPUT

  Serial.begin(9600); // start serial port 
}

void loop() {
  sig1 = pulseIn(CHANNEL_1, HIGH, 10000); // read pulse on channel 1 
  sig2 = pulseIn(CHANNEL_2, HIGH, 10000); // read pulse on channel 2

  Serial.print(sig1);
  Serial.print(" ");
  Serial.println(sig2); 

  // states: OFF = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4 
  delay(10); 
}

int getState(int sig1, int sig2) {
  // possible values are 1, 0 and -1 
  if((1520 < sig1 < 1540) && (1520 < sig2 < 1540)) {
    // both signals are 0, state: OFF
    Serial.println("OFF"); 
  }

  else if((sig1 > 1540) && (1520 < sig2 < 1540)) {
    // sig1 is 1, sig2 is 0, state: FORWARD
    Serial.println("FORWARD"); 
  }

  else if((sig1 < 1520) && (1520 < sig2 < 1540)) {
    // sig1 is -1, sig2 is 0, state: BACKWARD
    Serial.println("BACKWARD"); 
  }

  else if(sig2 > 1540) {
    // sig1 is X, sig2 is 1, state: RIGHT
    Serial.println("RIGHT"); 
  }

  else if(sig2 < 1520) {
    // sig1 is X, sig2 is -1, state: LEFT
    Serial.println("LEFT"); 
  }

  return 0;
}
