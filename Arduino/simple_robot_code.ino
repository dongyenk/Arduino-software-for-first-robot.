/* The robot has functions to go forward, backwards, left right.
* It has a supersonic transceiver, that transmits and receives super sonic signals.
* It can stop before it crashes into an obstacle.
*
* The robot consists of
*   - a chassis
*   - motors
*   - a motor controller
*   - an arduino
*   - a super sonic transceiver
*   - Many wires connecting everything.
*/

#include <Servo.h> // preprocessor directive. Preprocessor inserts contents of header file into program

Servo my_servo; 

byte right_forward_pin = 5;
byte right_reverse_pin = 6;
byte left_forward_pin = 9; // Digital Output pins, with pulse width modulation, that connect to the motor controller.
byte left_reverse_pin = 10;

byte servo_motor_pin = 11;

byte trig = 3; // trigger sonic signal - out
byte echo = 4; // echo detection -> INPUT

void robot_forward(){
  //digitalWrite( right_forward_pin, HIGH );
  //digitalWrite( left_forward_pin, HIGH ); // this code no good. the left motor goes faster than right motor, so it turns.  
  analogWrite( right_forward_pin, 255 );
  analogWrite( left_forward_pin, 160  );  // this seems to work.
  
  digitalWrite( right_reverse_pin, LOW );  
  digitalWrite( left_reverse_pin, LOW );
}
void robot_reverse(){
  digitalWrite( right_forward_pin, LOW );
  digitalWrite( right_reverse_pin, HIGH );
  
  digitalWrite( left_forward_pin, LOW );
  digitalWrite( left_reverse_pin, HIGH );
}
void robot_right(){
  digitalWrite( right_forward_pin, LOW );
  digitalWrite( right_reverse_pin, HIGH );

  digitalWrite( left_forward_pin, HIGH );
  digitalWrite( left_reverse_pin, LOW );
}
void robot_left(){
  digitalWrite( right_forward_pin, HIGH );
  digitalWrite( right_reverse_pin, LOW );

  digitalWrite( left_forward_pin, LOW );
  digitalWrite( left_reverse_pin, HIGH );
}
void robot_stop(){
  digitalWrite( right_forward_pin, LOW );
  digitalWrite( right_reverse_pin, LOW );
  
  digitalWrite( left_forward_pin, LOW );
  digitalWrite( left_reverse_pin, LOW );
}

unsigned long cm_distance_to_microseconds_duration( int distance = 0 ){
  // An obstacle 15 cm away causes 880 microseconds pulse duration on the supersonic transceiver. 880 / 15 approximately equals 58. // Will use (cm * 58) to get the microseconds duration for the distance argumetn
  return distance * 58; // converts the cm distance to the pulse duration, for an obstacle at that distance.
}

// to do : find out duration of pulse for obstacle 10cm away. Make system for converting supersonic pulse duration, to distance.
void setup(){
  Serial.begin(9600);

  pinMode( left_forward_pin, OUTPUT );
  pinMode( left_reverse_pin, OUTPUT );
  pinMode( right_forward_pin, OUTPUT );
  pinMode( right_reverse_pin, OUTPUT );

  
  pinMode( servo_motor_pin, OUTPUT ); // I keep forgetting these, then wonder why a thing isn't working

  pinMode( echo, INPUT ); // these 2 for Super sonic transceiver
  pinMode( trig, OUTPUT );

  
  robot_stop();
  delay(2000);
  robot_forward();
  
 
}
void loop(){
  long duration = 0;
  
  digitalWrite( trig, HIGH ); 
  delayMicroseconds( 10 );
  digitalWrite( trig, LOW ); // quickly outputs a supersonic signal pulse
  delayMicroseconds(10);
  
  duration = pulseIn( echo, HIGH ); // reads HIGH voltage pulse on echo pin; returns pulse length in microseconds
      // waits for echo pin to go from LOW to HIGH -> starts timing. waits for HIGH to LOW ->, stops timing
      // halts program while measuring pulse
  Serial.print("Pulse Microseconds: "); Serial.println( duration );
  // The closer an obstacle is to the supersonic sensor, the shorter the pulses.
  unsigned long stop_at_this_distance = cm_distance_to_microseconds_duration( 6 );
  
  if( duration <= stop_at_this_distance ){ // Once robot is very close to an obstacle, get the robot to turn, until it finds a new good direction.    
    robot_stop();
  }
}
