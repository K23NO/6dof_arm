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
#include <math.h>  // <-- necesario para M_PI

// ---------------- Configuración ----------------
#define N_SERVOS 6
const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int FEEDBACK_PINS[N_SERVOS] = {26, 25, 33, 32, 35, 34}; // Pines ADC NIKOLY
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

// Offset para alinear con RViz u otra referencia (0 rad cuando está en home físico)
const int SERVO_HOME_OFFSETS[N_SERVOS] = {90, 90, 90, 0, 90, 30};

const int FEEDBACK_WINDOW_SIZE = 5;  // tamaño de ventana de media móvil

// --------------- Infra micro-ROS ---------------
rcl_subscription_t subscriber;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t feedback_publisher;
std_msgs__msg__Int32MultiArray msg_in;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Int32MultiArray feedback_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

static int32_t data_buffer[N_SERVOS];
static int32_t feedback_buffer[N_SERVOS];
static double joint_positions[N_SERVOS];
static rosidl_runtime_c__String joint_name_array[N_SERVOS];
static int feedback_samples[N_SERVOS][FEEDBACK_WINDOW_SIZE];
static int feedback_indices[N_SERVOS] = {0};
static long feedback_sums[N_SERVOS] = {0};
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

// Convierte lectura ADC (15–3780) a ángulo (0–180)
int adcToAngle(int adc_value) {
  int angle = map(adc_value, 15, 3780, 0, 180);
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return angle;
}

int applyMovingAverage(int servo_idx, int sample_deg) {
  int idx = feedback_indices[servo_idx];
  feedback_sums[servo_idx] -= feedback_samples[servo_idx][idx];
  feedback_samples[servo_idx][idx] = sample_deg;
  feedback_sums[servo_idx] += sample_deg;
  feedback_indices[servo_idx] = (idx + 1) % FEEDBACK_WINDOW_SIZE;

  // redondeo al entero más cercano
  long sum = feedback_sums[servo_idx];
  return (int)((sum + (FEEDBACK_WINDOW_SIZE / 2)) / FEEDBACK_WINDOW_SIZE);
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

// ---------------- Timer: Publicación -------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    // Leer cada retroalimentación, convertir a ángulo y luego a radianes
    for (int i = 0; i < N_SERVOS; i++) {
      int adc_value = analogRead(FEEDBACK_PINS[i]);
      int real_angle = adcToAngle(adc_value);
      int filtered_angle = applyMovingAverage(i, real_angle);

      // Convertir a radianes respecto al home físico
      joint_positions[i] = (filtered_angle - SERVO_HOME_OFFSETS[i]) * (M_PI / 180.0);
      feedback_buffer[i] = filtered_angle;
    }

    // Publicar
    int64_t time_ns = rmw_uros_epoch_nanos();
    joint_state_msg.header.stamp.sec = time_ns / 1000000000;
    joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;
    joint_state_msg.position.size = N_SERVOS;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

    feedback_msg.data.size = N_SERVOS;
    RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
  }
}

// ---------------- Setup -------------------
void setup() {
  set_microros_transports();

  std_msgs__msg__Int32MultiArray__init(&msg_in);
  msg_in.data.data = data_buffer;
  msg_in.data.capacity = N_SERVOS;
  msg_in.data.size = 0;

  sensor_msgs__msg__JointState__init(&joint_state_msg);
  std_msgs__msg__Int32MultiArray__init(&feedback_msg);
  feedback_msg.data.data = feedback_buffer;
  feedback_msg.data.capacity = N_SERVOS;
  feedback_msg.data.size = N_SERVOS;

  analogReadResolution(12);  // Lecturas ADC consistentes (0-4095)

  for (int i = 0; i < N_SERVOS; i++) {
    rosidl_runtime_c__String__init(&joint_name_array[i]);
    rosidl_runtime_c__String__assign(&joint_name_array[i], joint_names[i]);
    joint_positions[i] = (SERVO_HOME_ANGLES[i] - SERVO_HOME_OFFSETS[i]) * (M_PI / 180.0);
    feedback_buffer[i] = SERVO_HOME_ANGLES[i];
    feedback_indices[i] = 0;
    feedback_sums[i] = SERVO_HOME_ANGLES[i] * FEEDBACK_WINDOW_SIZE;
    for (int j = 0; j < FEEDBACK_WINDOW_SIZE; j++) {
      feedback_samples[i][j] = SERVO_HOME_ANGLES[i];
    }
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

  // Inicializar servos y pines ADC
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    current_angles[i] = SERVO_HOME_ANGLES[i];
    servos[i].write(current_angles[i]);
    pinMode(FEEDBACK_PINS[i], INPUT);
  }

  // Suscripción y publicación
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

  RCCHECK(rclc_publisher_init_default(
    &feedback_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_feedback"));

  // Timer (100 ms)
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ---------------- Loop -------------------
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(5);
}