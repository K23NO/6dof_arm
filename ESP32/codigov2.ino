#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <ESP32Servo.h>

// ---------------- Configuración ----------------
#define N_SERVOS 6
const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Límites físicos
const int SERVO_MIN_ANGLES[N_SERVOS] = {0, 0, 0, 0, 0, 0};
const int SERVO_MAX_ANGLES[N_SERVOS] = {180, 180, 180, 180, 180, 180};

// Posición física de home
const int SERVO_HOME_ANGLES[N_SERVOS] = {90, 90, 90, 0, 90, 0};

// --------------- Infra micro-ROS ---------------
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg_in;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static int32_t data_buffer[N_SERVOS];
int current_angles[N_SERVOS] = {0, 0, 0, 0, 0, 0};

Servo servos[N_SERVOS];

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ---------------- Utilidades ------------------
void error_loop() {
  pinMode(2, OUTPUT);
  while (1) {
    digitalWrite(2, !digitalRead(2));
    delay(300);
  }
}

// ---------------- Callbacks -------------------
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32MultiArray * m =
    (const std_msgs__msg__Int32MultiArray *) msgin;

  size_t n = m->data.size;
  if (n > N_SERVOS) n = N_SERVOS;

  for (size_t i = 0; i < n; i++) {
    int angle = (int)m->data.data[i];
    if (angle < SERVO_MIN_ANGLES[i]) angle = SERVO_MIN_ANGLES[i];
    if (angle > SERVO_MAX_ANGLES[i]) angle = SERVO_MAX_ANGLES[i];

    current_angles[i] = angle;
    servos[i].write(current_angles[i]);  // mover servo físico
  }
}

// ---------------- Setup -------------------
void setup() {
  set_microros_transports();

  std_msgs__msg__Int32MultiArray__init(&msg_in);
  msg_in.data.data = data_buffer;
  msg_in.data.capacity = N_SERVOS;
  msg_in.data.size = N_SERVOS;

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    current_angles[i] = SERVO_HOME_ANGLES[i];
    servos[i].write(current_angles[i]);
  }

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_commands"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in,
                                         &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(5);
}
