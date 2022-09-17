#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <self_racing_car_msgs/VehicleCommand.h>

// ------ CONSTANTS ------

// Interrupt pins
// PIN MEGA   Interrupt pim
//  2             0
//  3             1
//  21            2
//  20            3
//  19            4
const int CHANNEL_1_PIN_INT_ID = 1; // i.e. pin 3
const int CHANNEL_2_PIN_INT_ID = 0; // i.e. pin 2
const int CHANNEL_3_PIN_INT_ID = 2; // i.e. pin 21

const int STEERING_OUTPUT_PIN = 6;
const int THROTTLE_OUTPUT_PIN = 7;

const int LED_PIN = 13;

const float STEERING_IDLE = 79;
const float STEERING_MAX = 98;
const float STEERING_MIN = 63;

const float THROTTLE_IDLE = 90;
const float THROTTLE_MAX = 105;
const float THROTTLE_MIN = 75;

const unsigned long PULSE_WIDTH_THRESHOLD = 3500;

const unsigned long CHANNEL_1_IDLE_MIN = 1486;
const unsigned long CHANNEL_1_IDLE_MAX = 1521;
const unsigned long CHANNEL_1_MAX = 2039;
const unsigned long CHANNEL_1_MIN = 1061;

const unsigned long CHANNEL_2_IDLE_MIN = 1462;
const unsigned long CHANNEL_2_IDLE_MAX = 1498;
const unsigned long CHANNEL_2_MAX = 1660;
const unsigned long CHANNEL_2_MIN = 1319;

const bool ROS_MODE = true;

// ------ VARIABLES ------

// commands that will be sent by PWM
float steering_angle_rx = STEERING_IDLE;
float throttle_angle_rx = THROTTLE_IDLE;
volatile float steering_angle_pi = STEERING_IDLE;
volatile float throttle_angle_pi = THROTTLE_IDLE;
float steering_angle_final = STEERING_IDLE;
float throttle_angle_final = THROTTLE_IDLE;

// channel 1 = steering
volatile unsigned long tmp_pulse_width_1 = 0;
volatile unsigned long pulse_width_1 = 0;
volatile unsigned long prev_time_1 = 0;
// channel 2 = throttle
volatile unsigned long tmp_pulse_width_2 = 0;
volatile unsigned long pulse_width_2 = 0;
volatile unsigned long prev_time_2 = 0;
// channel 3 = engagement switch
volatile unsigned long tmp_pulse_width_3 = 0;
volatile bool engaged_mode = false;
volatile unsigned long prev_time_3 = 0;

// Servo objects
Servo throttle_servo;
Servo steering_servo;

// ROS stuff
ros::NodeHandle nh;
void vehicle_command_callback(const self_racing_car_msgs::VehicleCommand &msg) {
  digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN)); // blink the led
  steering_angle_pi = msg.steering_angle;
  throttle_angle_pi = msg.throttle_angle;
}
ros::Subscriber<self_racing_car_msgs::VehicleCommand> sub("vehicle_command", &vehicle_command_callback);

// ------ FUNCTIONS ------

void steering_callback() {
  tmp_pulse_width_1 = micros() - prev_time_1;
  prev_time_1 = micros();

  if (tmp_pulse_width_1 < PULSE_WIDTH_THRESHOLD) {
    pulse_width_1 = tmp_pulse_width_1;
  }
}

void throttle_callback() {
  tmp_pulse_width_2 = micros() - prev_time_2;
  prev_time_2 = micros();

  if (tmp_pulse_width_2 < PULSE_WIDTH_THRESHOLD) {
    pulse_width_2 = tmp_pulse_width_2;
  }
}

void display_value(const char description[], float value) {
  if (ROS_MODE) {
    nh.loginfo(description);
    char result[8];
    dtostrf(value, 6, 2, result);
    nh.loginfo(result);
  } else {
    Serial.print(description);
    Serial.println(value);
  }
}

// ------ SETUP ------

void setup() {

  // Setting up connection etc
  if (ROS_MODE) {
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    nh.loginfo("In the setup");
  } else {
    Serial.begin(57600);
    Serial.println("In the setup");
  }

  // Setting up pins
  attachInterrupt(CHANNEL_1_PIN_INT_ID, steering_callback, CHANGE);
  attachInterrupt(CHANNEL_2_PIN_INT_ID, throttle_callback, CHANGE);
  pinMode(LED_PIN, OUTPUT);
  steering_servo.attach(STEERING_OUTPUT_PIN);
  throttle_servo.attach(THROTTLE_OUTPUT_PIN);

  // Doing the ESC calibration
  steering_servo.write(STEERING_IDLE);
  throttle_servo.write(THROTTLE_IDLE);
  delay(5000);

  if (ROS_MODE) {
    nh.loginfo("End setup");
  } else {
    Serial.println("End setup");
  }
}

// ------ LOOP ------

void loop() {

  // Compute steering from the Rx
  if (pulse_width_1 < CHANNEL_1_IDLE_MAX && pulse_width_1 > CHANNEL_1_IDLE_MIN) {
    steering_angle_rx = STEERING_IDLE;
  } else if (pulse_width_1 >= CHANNEL_1_IDLE_MAX) {
    steering_angle_rx = int(STEERING_IDLE + (pulse_width_1 - CHANNEL_1_IDLE_MAX) * ((STEERING_MAX - STEERING_IDLE) / (CHANNEL_1_MAX - CHANNEL_1_IDLE_MAX)));
  } else if (pulse_width_1 <= CHANNEL_1_IDLE_MIN) {
    steering_angle_rx = int(STEERING_IDLE - (CHANNEL_1_IDLE_MIN - pulse_width_1) * ((STEERING_IDLE - STEERING_MIN) / (CHANNEL_1_IDLE_MIN - CHANNEL_1_MIN)));
  } else {
  }

  // Compute throttle from the Rx
  if (pulse_width_2 < CHANNEL_2_IDLE_MAX && pulse_width_2 > CHANNEL_2_IDLE_MIN) {
    throttle_angle_rx = THROTTLE_IDLE;
  } else if (pulse_width_2 >= CHANNEL_2_IDLE_MAX) {
    throttle_angle_rx = int(THROTTLE_IDLE + (pulse_width_2 - CHANNEL_2_IDLE_MAX) * ((THROTTLE_MAX - THROTTLE_IDLE) / (CHANNEL_2_MAX - CHANNEL_2_IDLE_MAX)));
  } else if (pulse_width_2 <= CHANNEL_2_IDLE_MIN) {
    throttle_angle_rx = int(THROTTLE_IDLE - (CHANNEL_2_IDLE_MIN - pulse_width_2) * ((THROTTLE_IDLE - THROTTLE_MIN) / (CHANNEL_2_IDLE_MIN - CHANNEL_2_MIN)));
  } else {
  }

  // Displaying the values
  display_value("steering Rx: ", steering_angle_rx);
  display_value("throttle Rx: ", throttle_angle_rx);
  display_value("steering Pi: ", steering_angle_pi);
  display_value("throttle Pi: ", throttle_angle_pi);

  // Deciding which command to send
  if (engaged_mode) {
    // TODO:
    // - the logic here can be improved, for instance if the values from the Rx are outside of a certain region, we send them anyway
    // - we can also try to do some smoothing
    steering_angle_final = steering_angle_pi;
    throttle_angle_final = throttle_angle_pi;
  } else {
    steering_angle_final = steering_angle_rx;
    throttle_angle_final = throttle_angle_rx;
  }

  // Sending the commands
  if (steering_angle_final > STEERING_MAX) {
    steering_servo.write(STEERING_MAX);
  } else if (steering_angle_final < STEERING_MIN) {
    steering_servo.write(STEERING_MIN);
  } else {
    steering_servo.write(steering_angle_final);
  }

  if (throttle_angle_final > THROTTLE_MAX) {
    throttle_servo.write(THROTTLE_MAX);
  } else if (throttle_angle_final < THROTTLE_MIN) {
    throttle_servo.write(THROTTLE_MIN);
  } else {
    throttle_servo.write(throttle_angle_final);
  }

  if (ROS_MODE) {
    nh.spinOnce();
  }
  delay(1);
}
