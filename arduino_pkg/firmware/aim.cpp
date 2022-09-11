#include <Arduino.h>
#include <Servo.h>
 

// Interrupt pins
// PIN MEGA   Interrupt pim
//  2             0
//  3             1
//  21            2
//  20            3
//  19            4
const int int_id_chan_1 = 1;  // i.e. pin 3
const int int_id_chan_2 = 0;  // i.e. pin 2
const int int_id_chan_3 = 2;  // i.e. pin 21

const int THROTTLE_OUTPUT_PIN = 6;
const int STEERING_OUTPUT_PIN = 7;

const float STEERING_IDLE = 79;
const float STEERING_MAX = 98;
const float STEERING_MIN = 63;

float THROTTLE_IDLE = 90;

unsigned long CHANNEL_1_IDLE_MIN = 1486;
unsigned long CHANNEL_1_IDLE_MAX = 1521;
unsigned long CHANNEL_1_MAX = 2039;
unsigned long CHANNEL_1_MIN = 1061;

// Definitions

float throttle_angle = THROTTLE_IDLE;
float steering_angle = STEERING_IDLE;

// CHANNEL 1 = STEERING
volatile unsigned long tmp_pulse_width_1 = 0;
volatile unsigned long pulse_width_1 = 0;
volatile unsigned long prev_time_1 = 0;

// CHANNEL 2 = THROTTLE
volatile unsigned long tmp_pulse_width_2 = 0;
volatile unsigned long pulse_width_2 = 0;
volatile unsigned long prev_time_2 = 0;

// CHANNEL 3 = SWITCH MANUAL / AUTONOMOUS
volatile unsigned long pwm_value_3_change = 0;
volatile unsigned long pwm_value_3 = 0;
volatile unsigned long prev_time_3 = 0;

// servo object
Servo throttle_servo;
Servo steering_servo;

// Functions

void steering_callback(){
  tmp_pulse_width_1 = micros() - prev_time_1; 
  prev_time_1 = micros();

  if(tmp_pulse_width_1 < 3500){
    pulse_width_1 = tmp_pulse_width_1;
    // Serial.println(pulse_width_1);
   } 
}

void throttle_callback(){
  tmp_pulse_width_2 = micros() - prev_time_2; 
  prev_time_2 = micros();

  if(tmp_pulse_width_2 < 3500){
    pulse_width_2 = tmp_pulse_width_2;
    // Serial.println(pulse_width_2);

   }
}

void setup() {
  
  Serial.begin(57600);
  attachInterrupt(int_id_chan_1, steering_callback, CHANGE);
  attachInterrupt(int_id_chan_2, throttle_callback, CHANGE);
  //attachInterrupt(int_id_chan_3, change_3, CHANGE);
  
  throttle_servo.attach(THROTTLE_OUTPUT_PIN);
  throttle_servo.write(THROTTLE_IDLE);
  
  steering_servo.attach(STEERING_OUTPUT_PIN);
  steering_servo.write(STEERING_IDLE);
  
  delay(5000);
  Serial.println("End setup");

}
 
void loop() {

  // Steering
  if(pulse_width_1 < CHANNEL_1_IDLE_MAX && pulse_width_1 > CHANNEL_1_IDLE_MIN){
    steering_angle = STEERING_IDLE;
  } else if (pulse_width_1 >= CHANNEL_1_IDLE_MAX) {
    steering_angle = int(STEERING_IDLE + (pulse_width_1 - CHANNEL_1_IDLE_MAX) * ((STEERING_MAX - STEERING_IDLE) / (CHANNEL_1_MAX - CHANNEL_1_IDLE_MAX)));
  } else if (pulse_width_1 <= CHANNEL_1_IDLE_MIN) {
    steering_angle = int(STEERING_IDLE - (CHANNEL_1_IDLE_MIN - pulse_width_1) * ((STEERING_IDLE - STEERING_MIN) / (CHANNEL_1_IDLE_MIN - CHANNEL_1_MIN)));  
  }
  else {
  }

  if (steering_angle > STEERING_MAX) {
    steering_servo.write(STEERING_MAX);
  }
  else if (steering_angle < STEERING_MIN) {
    steering_servo.write(STEERING_MIN);
  }
  else {
    steering_servo.write(steering_angle);
  }


  Serial.print("steering: ");
  Serial.println(steering_angle);


//   // Throttle
//   // throttle_angle = THROTTLE_IDLE + int((pwm_value_2 - 1500) / 33);
//   // throttle_servo.write(throttle_angle);
  

  
// ;
//   // Serial.print("throttle: ");
//   // Serial.println(throttle_angle);
  
  delay(1); 

}
 


