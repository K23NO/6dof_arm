#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <ESP32Servo.h>

// ---------------- Configuración ----------------
#define N_SERVOS 6
const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const char* joint_names[N_SERVOS] = {
  "joint_1", "joint_2", "joint_3",
  "joint_4", "joint_5", "joint_6"
};

// Límites físicos
const int SERVO_MIN_ANGLES[N_SERVOS] = {0, 0, 0, 0, 0, 0};
const int SERVO_MAX_ANGLES[N_SERVOS] = {180, 180, 180, 180, 180, 180};

// Posición física de home
const int SERVO_HOME_ANGLES[N_SERVOS] = {90, 90, 90, 0, 90, 30};

// Offset para alinear con RViz
// Si mando [90,90,0,0,0,40] quiero que en RViz sea [0,0,0,0,0,0]
const int SERVO_HOME_OFFSETS[N_SERVOS] = {
  90, 90, 90, 0, 90, 0
};

// --------------- Infra micro-ROS ---------------
rcl_subscription_t subscriber;
rcl_publisher_t joint_state_publisher;
std_msgs__msg__Int32MultiArray msg_in;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

static int32_t data_buffer[N_SERVOS];
static double joint_positions[N_SERVOS];
static rosidl_runtime_c__String joint_name_array[N_SERVOS];
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

double angleToRadians(int angle, int servo_index) {
  int offset = SERVO_HOME_OFFSETS[servo_index];
  return (angle - offset) * (M_PI / 180.0);
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    for (int i = 0; i < N_SERVOS; i++) {
      joint_positions[i] = angleToRadians(current_angles[i], i);
    }

    int64_t time_ns = rmw_uros_epoch_nanos();
    joint_state_msg.header.stamp.sec = time_ns / 1000000000;
    joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
  }
}

// ---------------- Setup -------------------
void setup() {
  set_microros_transports();

  std_msgs__msg__Int32MultiArray__init(&msg_in);
  msg_in.data.data = data_buffer;
  msg_in.data.capacity = N_SERVOS;
  msg_in.data.size = N_SERVOS;

  sensor_msgs__msg__JointState__init(&joint_state_msg);

  for (int i = 0; i < N_SERVOS; i++) {
    rosidl_runtime_c__String__init(&joint_name_array[i]);
    rosidl_runtime_c__String__assign(&joint_name_array[i], joint_names[i]);
    joint_positions[i] = angleToRadians(SERVO_HOME_ANGLES[i], i);
  }

  joint_state_msg.name.data = joint_name_array;
  joint_state_msg.name.capacity = N_SERVOS;
  joint_state_msg.name.size = N_SERVOS;

  joint_state_msg.position.data = joint_positions;
  joint_state_msg.position.capacity = N_SERVOS;
  joint_state_msg.position.size = N_SERVOS;

  rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

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

  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(5);
}
