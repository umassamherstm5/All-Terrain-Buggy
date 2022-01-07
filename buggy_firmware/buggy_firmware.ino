/*
 * All-Terrain Buggy
 * M5 Demonstration 
 * Firmware for interfacing with buggy and remote control 
 * Authors: Akshat Sahay, Chris Caron, Sebastian Armstrong 
 */

/*
 * FlySky FS-GT
 * ------------
 * The FlySky FS-GT is a radio receiver module which pairs to a FlySky hobbyist controller. 
 * 
 * We connect this receiver to our Feather 328P and read the pulses on different channels using the pulseIn function. 
 * CHANNEL_1 refers to the left joystick's vertical motion. 
 * CHANNEL_2 refers to the right joystick's horizontal motion. 
 */

/* pin assignments for FlySky receiver signals */
#define CHANNEL_1 A1 // channel 1 of the controller
#define CHANNEL_2 A2 // channel 2 of the controller

/*
 * DRV8871 Motor Driver 
 * --------------------
 * The DRV8871 is a brushed DC motor driver. Two logic inputs control 4 N-channel MOFSETs for bidirectional control. 
 * The inputs can be either HIGH/LOW or PWM (we use PWM) to control motor speed and direction. 
 * Setting both inputs to LOW activates a low-power deep sleep mode. This will help us conserve battery life. 
 * 
 * For ease of control, we connect each of the 3 motors on either side of the buggy together. 
 * This allows us to use only 4 pins on the Arduino instead of 12 (2 for each motor)
 * This also makes turning easy, as we simply slow down a side to turn in either direction. 
 * In the code, we refer to A as the right side and B as the left side. 
 * 
 * All inputs are PWM-capable pins 
 * AIN1 refers to input 1 for the right side. 
 * AIN2 refers to input 2 for the right side.
 * BIN1 refers to input 1 for the left side. 
 * BIN2 refers to input 2 for the left side. 
 */

/* pin assignments for driver control signals */
#define AIN1 5
#define AIN2 6 
#define BIN1 9
#define BIN2 10 

/*
 * We use pulseIn to read the pulse durations on CHANNEL_1 and CHANNEL_2. 
 * pulseIn detects a change on a pin (LOW->HIGH/HIGH->LOW). 
 * It measures the duration of the change and returns the duration in microseconds. 
 * 
 * The receiver pulses have default durations of about 1450us for CHANNEL_1 and 1475us for CHANNEL_2.  
 * Any shorter or longer pulses mean the joysticks are being used, and the buggy needs to perform an action. 
 * 
 * The threshold values are to check for the default durations: they act as a "deadzone" when the joysticks 
 * are in their central position. The car only performs actions when the durations are more than the 
 * "deadzone" values. 
 * 
 * We organise the buggy's action into 5 modes: STOP, FORWARD, BACKWARD, LEFT and RIGHT. 
 * Speed and vertical direction are controlled by the left joystick (sig1, CHANNEL_1). 
 * Horizontal direction is controlled by the right joystick (sig2, CHANNEL_2). 
 * 
 * STOP, FORWARD and BACKWARD are simple: the car stops, goes forward or backward. 
 * LEFT and RIGHT are a little more complex: in these modes the car goes in the direction specified, 
 * but the forward and backward motion is controlled within these conditions. 
*/

/* thresholds for signal values, used in loop */
#define SIG_HIGH_ACC 1490
#define SIG_LOW_ACC 1410

#define SIG_HIGH_STEER 1515
#define SIG_LOW_STEER 1435

#define DEBUG 0  // print signal values

/* variables for incoming pulse widths from CHANNEL_1 and CHANNEL_2 respectively */
int sig1 = 0; // CHANNEL_1 value
int sig2 = 0; // CHANNEL_2 value

/* variables for speed of either side */
int a_speed = 0; // speed of right side
int b_speed = 0; // speed of left side
int offset  = 0; // speed difference for turning

/*
 * We use status LEDs on our PCB to easily debug problems without using the serial console.
 * SIG_OFF turns on if either signal value is a 0 (means FlySky receiver is NC or OFF). 
 * SIG_ON turns on otherwise, meaning we receive signals from the receiver and everything is fine. 
 */

/* pin assignments for status LEDs */
#define LED_NOSIG 2
#define LED_SIG 3

/*
 * Function: setup
 * ---------------
 * Runs once at the beginning of the code (when the buggy powers up or resets). 
 * Sets all required pin modes and begins the serial module. 
 * 
 * returns: none 
 */
void setup() {
  // set RC channels to INPUT
  pinMode(CHANNEL_1, INPUT); 
  pinMode(CHANNEL_2, INPUT); 

  // set driver pins to OUTPUT
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT); 
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT); 

  Serial.begin(9600); // start serial port 
}

/*
 * Function: loop
 * --------------
 * Runs every clock cycle of the ATMEGA328P (8MHz frequency). 
 * Collects pulse values from the FlySky FS-GT receiver and uses them to drive the buggy. 
 * 
 * returns: none 
 */
void loop() {
  // get signal values from receiver 
  sig1 = pulseIn(CHANNEL_1, HIGH);
  sig2 = pulseIn(CHANNEL_2, HIGH);

  if(sig1 == 0 || sig2 == 0) {
    // error, turn on LED_NOSIG
    digitalWrite(LED_NOSIG, HIGH); 
    digitalWrite(LED_SIG, LOW);
  }

  else {
    // no error, turn on LED_SIG
    digitalWrite(LED_NOSIG, LOW); 
    digitalWrite(LED_SIG, HIGH);
  }

  if(DEBUG == 1) {
    // print signal values
    Serial.print(sig1);
    Serial.print(" ");
    Serial.println(sig2);    
  }
  
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
