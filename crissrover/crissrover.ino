
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define NUM_MOTORS 4

// Motor pin definitions
const int PWM_PINS[NUM_MOTORS] = {11, 10, 9, 8};
const int DIR_PINS[NUM_MOTORS] = {34, 36, 38, 40};
const int ENCA_PINS[NUM_MOTORS] = {22, 24, 50, 52};
const int ENCB_PINS[NUM_MOTORS] = {23, 25, 51, 53};

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__Int32MultiArray pwm_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile int posi[NUM_MOTORS] = {0};
volatile int pwm_values[NUM_MOTORS] = {0};

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Function pointer array for encoder reading
void (*encoderInterruptHandlers[NUM_MOTORS])() = {
  []() { readEncoder(0); },
  []() { readEncoder(1); },
  []() { readEncoder(2); },
  []() { readEncoder(3); }
};

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Copy encoder positions to message
    for (int i = 0; i < NUM_MOTORS; i++) {
      encoder_msg.data.data[i] = posi[i];
    }
    RCSOFTCHECK(rcl_publish(&publisher, &encoder_msg, NULL));
  }
}

volatile uint32_t last_message_time = 0;

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = 
    (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // Copy received PWM values
  for (int i = 0; i < NUM_MOTORS; i++) {
    pwm_values[i] = msg->data.data[i];
  }

  last_message_time = millis();
}

void setup() {
  set_microros_transports();
  
  // Setup pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(ENCA_PINS[i], INPUT);
    pinMode(ENCB_PINS[i], INPUT);
    
    // Attach interrupts using function pointers
    attachInterrupt(digitalPinToInterrupt(ENCA_PINS[i]), 
                    encoderInterruptHandlers[i], CHANGE);
  }
  
  delay(2000);
  allocator = rcl_get_default_allocator();
  
  // Micro-ROS initialization
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_motor_node", "", &support));
  
  // Allocate memory for multiarray messages
  encoder_msg.data.capacity = NUM_MOTORS;
  encoder_msg.data.size = NUM_MOTORS;
  encoder_msg.data.data = (int32_t*)malloc(NUM_MOTORS * sizeof(int32_t));
  
  pwm_msg.data.capacity = NUM_MOTORS;
  pwm_msg.data.size = NUM_MOTORS;
  pwm_msg.data.data = (int32_t*)malloc(NUM_MOTORS * sizeof(int32_t));
  
  // Create publisher and subscriber
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data"));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "pwm_command"));
  
  // Timer setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  
  uint32_t current_time = millis();

  // Stop motors if no message received in the last 100ms
  if (current_time - last_message_time > 100) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      pwm_values[i] = 0;
    }
  }
  
  // Run motors with current PWM values
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Implement motor control logic
    digitalWrite(DIR_PINS[i], pwm_values[i] > 0 ? HIGH : LOW);
    analogWrite(PWM_PINS[i], abs(pwm_values[i]));
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // time delay to check change
}

void readEncoder(int motorIndex) {
  static int lastA[NUM_MOTORS] = {0};
  static int lastB[NUM_MOTORS] = {0};
  
  int a = digitalRead(ENCA_PINS[motorIndex]);
  int b = digitalRead(ENCB_PINS[motorIndex]);
  
  if (a != lastA[motorIndex] || b != lastB[motorIndex]) {
    if (a != lastA[motorIndex]) {
      posi[motorIndex] += (b == a) ? 1 : -1;
    } else {
      posi[motorIndex] += (a == b) ? 1 : -1;
    }
    
    lastA[motorIndex] = a;
    lastB[motorIndex] = b;
  }
}
