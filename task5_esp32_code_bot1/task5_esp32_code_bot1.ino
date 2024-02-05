#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h> 

#include <Servo.h>

static const int servoPin1 = 13;
static const int servoPin2 = 12;
static const int servoPin3 = 14;
static const int servoPin4 = 27;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t publisher;
std_msgs__msg__Bool bool_msg;
rcl_subscription_t sub_pen_down_2;
std_msgs__msg__Bool bool_sub_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void subscription_callback_pen(const void * msgin)
{  
   const std_msgs__msg__Bool *received_msg = (const std_msgs__msg__Bool *)msgin;
}

void pen2_down_publish(bool value) {
  bool_msg.data = value;
  RCSOFTCHECK(rcl_publish(&publisher, &bool_msg, NULL));
}


void setup() {
  set_microros_wifi_transports("OPPO Reno8 5G", "k23xdrxw", "192.168.235.8", 8888);
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot2_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/bot2_vel"));

  RCCHECK(rclc_subscription_init_default(
    &sub_pen_down_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pen2_down"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pen2_down"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_pen_down_2, &bool_sub_msg, &subscription_callback_pen, ON_NEW_DATA));

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo4.write(120);
  pen2_down_publish(false);
  

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  if (bool_sub_msg.data == true){
    servo4.write(90);
  }
  else{
    servo4.write(120);
  }

  Serial.print("subscribing data :");
  Serial.println(msg.linear.x);
  servo1.write(msg.linear.x);
  servo2.write(msg.linear.y);
  servo3.write(msg.linear.z);
}
