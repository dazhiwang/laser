// ME350 Laser Reflection Sketch - W16
// updated 1-5-2016 

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

// Targets:
const int DES_POSITION_1 = 3;   // [encoder counts] Motor position corresponding to first lane (swill store the encoder values from the current iteration (A and B).
// The two bits to the left of those wiluggest to put it as 0)
const int DES_POSITION_2 = 105; // [encoder counts] Motor position corresponding to second lane
const int DES_POSITION_3 = 228; // [encoder counts] Motor position corresponding to third lane
const int DES_POSITION_4 = 445; // [encoder counts] Motor position corresponding to fourt lane
const int LOWER_BOUND    = 0;   // [encoder counts] For calibration. This is the value your mechanism calibrates to in lane 1.
boolean calibrated       = false; // This variable keeps track of whether the mechanism is or is not calibrated.

// Sensor settings:
const int PROX_SWITCH_THRESHOLD = 220; // [0-1023] thershold value after which the proximity switch turns off

// Computation of position and velocity:
volatile long motorPosition = 0;    // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int  encoderStatus = 0;    // [binary] Past and Current A&B values of the encoder 
// The rightmost two bits of encoderStatus l store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity         = 0;    // [encoder counts / seconds] Current motor velocity 
long previousMotorPosition  = 0;    // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime    = 0;    // [microseconds] System clock value the last time a velocity was computed 
const int MIN_VEL_COMP_COUNT = 2;   // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME  = 5000; // [microseconds] Minimal time that must pass between two velocity measurements
int oldTargetPosition = 0;
// Control
int targetPosition    = 0;   // [encoder counts] desired motor position
float positionError   = 0;   // [encoder counts] Position error
float integralError   = 0;   // [encoder counts * seconds] Integrated position error
float velocityError   = 0;   // [encoder counts / seconds] Velocity error
float desiredVoltage  = 0;   // [Volt] Desired motor voltage
int motorCommand      = 0;   // [0-255] PWM signal sent to the motor
const float KP             = 0.006;//0.035;   // [Volt / encoder counts] P-Gain
const float KI             = 0.0001;//0.005;   // [Volt / (encoder counts * seconds)] I-Gain
const float KD             = 0.00035;//0.003;   // [Volt * seconds / encoder counts] D-Gain up to probably 0.0005
const float BASE_CMD       = 1.5* 2.15;   // [Volt] Base voltage that is needed to make the motor just start to rotate
const int   CTRL_DEAD_BAND = 2;      // [encoder counts] "Close enough" range, in which the controller does not send a voltage to the motor
const float SUPPLY_VOLTAGE = 9.0;    // [Volt] Supply voltage at the HBridge
   
// Time keeping:
unsigned long executionDuration = 0;  // [microseconds] Time between this and the previous loop execution
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time

// Pin assignment
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_LANE_1           = 4;
const int PIN_NR_LANE_2           = 6;
const int PIN_NR_LANE_3           = 12;
const int PIN_NR_LANE_4           = 13;
const int PIN_NR_ON_OFF_SWITCH    = 5;
const int PIN_NR_FWD_LIMIT_SWITCH = 7;
const int PIN_NR_PROX_SWITCH_1    = 1;
const int PIN_NR_PWM_OUTPUT       = 9;
const int PIN_NR_PWM_DIRECTION_1  = 10;
const int PIN_NR_PWM_DIRECTION_2  = 11;


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
    // Bitwise shift left by one bit, to make room for a bit of new data:
    encoderStatus <<= 1;   
    // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
    // and put that value into the rightmost bit of encoderStatus:
    encoderStatus |= digitalRead(2);   
    // Bitwise shift left by one bit, to make room for a bit of new data:
    encoderStatus <<= 1;
    // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
    // and put that value into the rightmost bit of encoderStatus:
    encoderStatus |= digitalRead(3);
    // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
    // bitwise AND operator on mstatus and 15(=1111):
    encoderStatus &= 15;
    if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) 
    {   // the encoder status matches a bit pattern that requires counting up by one
        motorPosition++;         // increase the encoder count by one
    }
    else
    {   // the encoder status does not match a bit pattern that requires counting up by one.  
        // Since this function is only called if something has changed, we have to count downwards
        motorPosition--;         // decrease the encoder count by one
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // Declare which digital pins are inputs and which are outputs:
    pinMode(PIN_NR_ENCODER_A,        INPUT);
    pinMode(PIN_NR_ENCODER_B,        INPUT);
    pinMode(PIN_NR_LANE_1,           INPUT);
    pinMode(PIN_NR_LANE_2,           INPUT);
    pinMode(PIN_NR_LANE_3,           INPUT);
    pinMode(PIN_NR_LANE_4,           INPUT);
    pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
    pinMode(PIN_NR_FWD_LIMIT_SWITCH, INPUT);
    pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
    pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
    pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);

    // Turn on the pullup resistors on the encoder channels
    // (the other sensors already have physical resistors on the breadboard)
    digitalWrite(PIN_NR_ENCODER_A, HIGH);  
    digitalWrite(PIN_NR_ENCODER_B, HIGH);

    // Activate interrupt for encoder pins.
    // If either of the two pins changes, the function 'updateMotorPosition' is called:
    attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
    attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

    // Set initial output to the motor to 0
    analogWrite(PIN_NR_PWM_OUTPUT, 0);

    // Begin serial communication for monitoring.
    Serial.begin(115200);
    Serial.println("start");
}


////////////////////////////////////////////////////////////////////////////////////////////////
// After creating a setup() function, which initializes and sets the initial values,          //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to change and respond. Use it to actively control the Arduino board. //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
    // Determine the duration it took to execute the last loop. This time is used 
    // for integration and for monitoring the loop time via the serial monitor.
    executionDuration = micros() - lastExecutionTime;
    lastExecutionTime = micros();
  
    // Read sensor signals from pins
    int proxSwitch1        = analogRead(PIN_NR_PROX_SWITCH_1);
    int limitSwitchForward = digitalRead(PIN_NR_FWD_LIMIT_SWITCH);
    int onOffSwitch        = digitalRead(PIN_NR_ON_OFF_SWITCH);
    int lane1              = digitalRead(PIN_NR_LANE_1);
    int lane2              = digitalRead(PIN_NR_LANE_2);
    int lane3              = digitalRead(PIN_NR_LANE_3);
    int lane4              = digitalRead(PIN_NR_LANE_4);
    
    // Speed Computation:
    if (abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT ||
        (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME)       
    {   // If at least a minimum time interval has elapsed or
        // the motor has travelled through at least a minimum angle ... 
        // .. compute a new value for speed:
        // (speed = delta angle [encoder counts] divided by delta time [seconds])
        motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                                (micros() - previousVelCompTime);
        // Remember this encoder count and time for the next iteration:
        previousMotorPosition = motorPosition;
        previousVelCompTime   = micros();
    }

    // Control:
    if (onOffSwitch==HIGH && proxSwitch1<PROX_SWITCH_THRESHOLD)
    {   // If the toggle switch is on and there is nothing in the proximity sensor range...
        // .. run the controller:
    
        // Determine which lane has a laser on; set that lane's encoder count as the desired position
        if (lane1==HIGH) {
            targetPosition = DES_POSITION_1;
        } else if (lane2==HIGH) {
            targetPosition = DES_POSITION_2;
        } else if (lane3==HIGH) {
            targetPosition = DES_POSITION_3;
        } else if (lane4==HIGH) {
            targetPosition = DES_POSITION_4;
        } else {
            // no target specified, do nothing and leave value at previous value
        }
        if (targetPosition != oldTargetPosition){
            oldTargetPosition = targetPosition;
            integralError = 0;
        }
        
        // Compute the position error [encoder counts]
        positionError = targetPosition - motorPosition;
        
        // Compute the integral of the position error  [encoder counts * seconds]
        if (abs(positionError) < CTRL_DEAD_BAND)
        {
          // prevent the integral error from continuing to accumulate, if the position is close enough
          integralError = integralError;
        }
        else  
        {   
          integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
        }
        
        
        // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
        velocityError = 0 - motorVelocity;

        // Use a PID controller to send the appropriate signal to the motor
        // This uses the function "PID_Controller" at the end of the sketch
        desiredVoltage = PID_Controller(positionError, integralError, velocityError);
    }
    else
    {   //  Otherwise, either the toggle switch is off or there is something in the proximity sensor range, so...
        // ... do not run the controller, stop the motor:
        desiredVoltage = 0; 
        // .. and reset the integrator of the error:
        integralError = 0;
        // Produce some debugging output:
        if(onOffSwitch==LOW)
        {   // If the toggle switch is off...
            // .. notify the user
            Serial.println("Warning: The toggle switch is off.");
            
            //////////This is where you should set "calibrated" back to false/////////////
            
        }
        else
        {   // Otherwise something is in the way of the proximity sensor, so...
            // ... notify the user
            Serial.println("Warning: There is an object in the way.");
        }
    }
  
  ////////////////////////////////////////////////////////////
  ////////////// Insert Calibration Code Here!!!//////////////
  ////////////////////////////////////////////////////////////
  
  // It is recommended that you use an "if else" statement to trigger your calibration sequence
  // if the variable "calibrated" is false. Upon initialization the code will set "calibrated"
  // equal to false. Then when this point of the loop is reached, if "calibrated" is false your
  // mechanism should should SLOWLY move to the first lane then once the limit switch is triggered
  // and the motor velocity is 0 you can set your motor position equal to the lane one value (zero).
  // It might also be helpful to ADD A DELAY after the limit switch is triggered and the motor
  // velocity is zero. If you put a small delay of ~500 ms, any potentially bouncing off the hard
  // stop will cease before you set your calibration position. Finally, after you have very accurately
  // calibrated your mechanism you can set "calibrated" equal to true and on the next loop your
  // mechanism be ready to respond to laser inputs.
 if (calibrated == false && onOffSwitch ==  HIGH && proxSwitch1<PROX_SWITCH_THRESHOLD ){
  desiredVoltage = -3;
  if (limitSwitchForward == HIGH && motorVelocity == 0){
    delay(1000);
    calibrated = true;
    //encode//!!!!!!
    motorPosition = LOWER_BOUND;
    desiredVoltage = 0;
  }
 }
if (onOffSwitch == LOW){
  calibrated = false;
}
  // Your mechanism should reset "calibrated" back to false when the toggle switch is flipped to "off".
  // This will allow you to easily re-calibrate at any point during testing if you feel the need. 
  
    // Send the command (either result of control, or 0) to the H-bridge
    // Convert from voltage to PWM cycle:
    motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
    // Clip values larger than 255
    if (motorCommand > 255)
    {
        motorCommand = 255;
        Serial.println("Warning: Exceeded maximal desired voltage.  Anti-Windup might be needed.");
    }
    
    analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
    // Determine rotation direction
    if (desiredVoltage >= 0)
    {   // If voltage is positive ...
        // ... turn forward
        digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
        digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
    }
    else
    {   // ... otherwise turn backward:
        digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
        digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
    }
    
    // Send a status of the controller to the serial monitor.  
    // Each character will take 85 microseconds to send, so be
    // selective in what you write out:
//    if (printCounter == FREQUENCY_OF_PRINTING)
//    {
  
      //Serial.print("Power switch [on/off]: ");
      Serial.print("PWR: ");
      Serial.print(onOffSwitch);
      //Serial.print("      Motor Position [encoder counts]: ");
      Serial.print("  MP: ");
      Serial.print(motorPosition);
      //Serial.print("      Motor Velocity [encoder counts / seconds]: ");
      Serial.print("  MV: ");
      Serial.print(motorVelocity);
      //Serial.print("      Encoder Status [4 bit value]: ");
      Serial.print("  ES: ");
      Serial.print(encoderStatus);
      //Serial.print("      Target Position [encoder counts]: ");
      Serial.print("  TP: ");
      Serial.print(targetPosition);
      //Serial.print("      Position Error [encoder counts]: ");
      Serial.print("  PE: ");
      Serial.print(positionError);
      //Serial.print("      Integrated Error [encoder counts * seconds]: ");
      Serial.print("  IE: ");
      Serial.print(integralError);
      //Serial.print("      Velocity Error [encoder counts / seconds]: ");
      Serial.print("  VE: ");
      Serial.print(velocityError);
      //Serial.print("      Desired Output Voltage [Volt]: ");
      Serial.print("  DV: ");
      Serial.print(desiredVoltage);
      //Serial.print("      Motor Command [0-255]: ");
      Serial.print("  MC: ");
      Serial.print(motorCommand);
      //Serial.print("      Execution Duration [microseconds]: ");
      Serial.print("  ED: ");
      Serial.print(executionDuration);
      Serial.println(""); // new line
      
      
}  // End of main loop


//////////////////////////////////////////////////////////////////////////////////////////
// This is the actual controller function that uses the error in position and velocity  //
// and the integrated error and computes a desired voltage that should be sent to the   //
// motor.  In addition to the standard PID controller, a feedforward voltage is also    //
// computed additively to compensate for friction.                                      //
//////////////////////////////////////////////////////////////////////////////////////////
float PID_Controller(int positionError_, int integralError_, float velocityError_ )
{
    float  desiredVoltage_; // [Volt] Desired motor voltage
    
//    //limit the error, to reduce overshoot if travelling through a large angle
//    if(positionError_ > 120)
//    {
//         positionError_ = 120;
//    }
//    if(positionError_ < -120)
//    {
//         positionError_ = -120;
//    }
    
    // PID control equation:  
    desiredVoltage_ = KP * positionError_ +  
                      KI * integralError_ +
                      KD * velocityError_;

    // Add a fixed value in the direction of desired motion to overcome friction etc.    
    if (desiredVoltage_ < 0)
      {
      desiredVoltage_ = desiredVoltage_ - BASE_CMD;
      }

    else
      {
      desiredVoltage_ = desiredVoltage_ + BASE_CMD;
      }
    

    // If we are close enough to the desired target, turn motor off:  
    if (abs(positionError_) < CTRL_DEAD_BAND)
    {
         desiredVoltage_ = 0;
    }
    
    // Return the desired voltage to the loop-function
    return desiredVoltage_;
}
