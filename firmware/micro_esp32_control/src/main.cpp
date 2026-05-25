/**
 * main.cpp — Robot ESP32 DevKit v1   [VERSION DEFINITIVA]
 * ══════════════════════════════════════════════════════════════════
 * Cambios respecto a la version anterior:
 *   1. IMU TOLERANTE A FALLOS: si el MPU6050 no responde, el robot
 *      arranca igual y la teleoperacion funciona. El imu_task solo
 *      se lanza si imu_ok == true.
 *   2. I2C ROBUSTO: 100 kHz (mas estable que 400 kHz con cableado
 *      real) + Wire.setTimeOut(25) para que una lectura fallida
 *      nunca cuelgue el task ni dispare el watchdog.
 *   3. YAW REAL: el giroscopio (gz) se integra en un angulo de yaw
 *      acumulado. El quaternion publicado en /robot_imu ya NO tiene
 *      yaw=0; lleva la rotacion real en Z, que es la que el nodo de
 *      odometria necesita para estimar la trayectoria del robot.
 *
 * Hardware (segun esquematico):
 *   - TB6612FNG: PWMA→D32 AI1→D25 AI2→D26 | PWMB→D13 BI1→D14 BI2→D27
 *   - MPU6050:   SDA→D21  SCL→D22
 *   - micro-ROS via WiFi UDP
 *
 * Topicos ROS2:
 *   Publica  /robot_imu   (sensor_msgs/Imu)  — con yaw real
 *   Suscribe /cmd_vel     (geometry_msgs/Twist)
 * ══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <micro_ros_platformio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// ════════════════════════════════════════════════════════════════
//  Pines I2C — MPU6050
// ════════════════════════════════════════════════════════════════
#define I2C_SDA 21
#define I2C_SCL 22

#define CH_A 0
#define CH_B 1

// ════════════════════════════════════════════════════════════════
//  Pines TB6612FNG (Motor A = Rueda_der | Motor B = Rueda_izq)
// ════════════════════════════════════════════════════════════════
#define PWMA_PIN    32
#define PWMB_PIN    13
#define MOTORA_IN1  26
#define MOTORA_IN2  25
#define MOTORB_IN1  14
#define MOTORB_IN2  27

#define LEDC_FREQ  5000
#define LEDC_RES   8

// ════════════════════════════════════════════════════════════════
//  Geometria del robot (del URDF)
// ════════════════════════════════════════════════════════════════
#define WHEEL_SEPARATION  0.0808f
#define MAX_LINEAR_VEL    0.3f
#define STEERING_EFF      1.0f
#define MIN_PWM_THRESHOLD 0.10f

// ════════════════════════════════════════════════════════════════
//  Escalas MPU6050 (rango por defecto: ±2g, ±250°/s)
// ════════════════════════════════════════════════════════════════
static constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
static constexpr float GYRO_SCALE  = (3.14159265f / 180.0f) / 131.0f;

// Umbral anti-deriva del giroscopio en reposo: por debajo de este
// valor (rad/s) se considera ruido y no se integra al yaw. Esto
// evita que el yaw se desvie solo con el robot quieto.
#define GYRO_Z_DEADBAND   0.012f

#define RCSOFTCHECK(fn) { (void)(fn); }

// ════════════════════════════════════════════════════════════════
//  Objetos micro-ROS
// ════════════════════════════════════════════════════════════════
static rcl_publisher_t            pub_imu;
static rcl_subscription_t         sub_cmd_vel;
static rcl_timer_t                timer;
static rclc_executor_t            executor;
static rclc_support_t             support;
static rcl_allocator_t            allocator;
static rcl_node_t                 node;
static sensor_msgs__msg__Imu      imu_msg;
static geometry_msgs__msg__Twist  cmd_vel_msg;

// ════════════════════════════════════════════════════════════════
//  IMU
// ════════════════════════════════════════════════════════════════
static MPU6050 mpu;

// Bandera global: true solo si el MPU6050 respondio en el arranque.
// Si es false, imu_task NO se lanza y el robot funciona sin IMU.
static bool imu_ok = false;

static SemaphoreHandle_t imu_mutex;
static struct {
  float ax, ay, az;
  float gx, gy, gz;
  float qw, qx, qy, qz;   // quaternion con yaw real
} imu_data = {0,0,0,0,0,0, 1,0,0,0};

// Estado del filtro: pitch/roll por filtro complementario,
// yaw por integracion directa del giroscopio Z.
static float pitch_est = 0.0f;
static float roll_est  = 0.0f;
static float yaw_est   = 0.0f;
static unsigned long last_imu_us = 0;

// ════════════════════════════════════════════════════════════════
//  Motores
// ════════════════════════════════════════════════════════════════
static SemaphoreHandle_t cmd_mutex;
static float g_linear_x  = 0.0f;
static float g_angular_z = 0.0f;


// ════════════════════════════════════════════════════════════════
//  Control de motores
// ════════════════════════════════════════════════════════════════
void motors_init()
{
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);

  ledcSetup(CH_A, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PWMA_PIN, CH_A);
  ledcSetup(CH_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PWMB_PIN, CH_B);

  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
  digitalWrite(MOTORA_IN1, LOW);
  digitalWrite(MOTORA_IN2, LOW);
  digitalWrite(MOTORB_IN1, LOW);
  digitalWrite(MOTORB_IN2, LOW);
}

void set_motor_a(float speed)
{
  if (fabsf(speed) < MIN_PWM_THRESHOLD) {
    digitalWrite(MOTORA_IN1, LOW); digitalWrite(MOTORA_IN2, LOW);
    ledcWrite(CH_A, 0); return;
  }
  int pwm = constrain((int)(fabsf(speed) * 255.0f), 0, 255);
  digitalWrite(MOTORA_IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(MOTORA_IN2, speed > 0 ? LOW  : HIGH);
  ledcWrite(CH_A, pwm);
}

void set_motor_b(float speed)
{
  if (fabsf(speed) < MIN_PWM_THRESHOLD) {
    digitalWrite(MOTORB_IN1, LOW); digitalWrite(MOTORB_IN2, LOW);
    ledcWrite(CH_B, 0); return;
  }
  int pwm = constrain((int)(fabsf(speed) * 255.0f), 0, 255);
  digitalWrite(MOTORB_IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(MOTORB_IN2, speed > 0 ? LOW  : HIGH);
  ledcWrite(CH_B, pwm);
}

void apply_cmd_vel(float linear_x, float angular_z)
{
  float half_sep = (WHEEL_SEPARATION / 2.0f) * STEERING_EFF;
  float v_right  = linear_x + (angular_z * half_sep);
  float v_left   = linear_x - (angular_z * half_sep);

  float sr = v_right / MAX_LINEAR_VEL;
  float sl = v_left  / MAX_LINEAR_VEL;

  float max_s = max(fabsf(sr), fabsf(sl));
  if (max_s > 1.0f) { sr /= max_s; sl /= max_s; }

  set_motor_a(sr);
  set_motor_b(sl);
}


// ════════════════════════════════════════════════════════════════
//  Filtro de orientacion
//    pitch/roll → filtro complementario (giroscopio + acelerometro)
//    yaw        → integracion directa del giroscopio Z
//
//  El MPU6050 no tiene magnetometro, asi que el yaw no se puede
//  corregir con un sensor absoluto: se integra gz. Tiene deriva
//  lenta, aceptable para una sustentacion y para alimentar la
//  odometria a corto/medio plazo. La banda muerta GYRO_Z_DEADBAND
//  evita que el yaw se mueva con el robot en reposo.
// ════════════════════════════════════════════════════════════════
void update_orientation(float ax, float ay, float az,
                        float gx, float gy, float gz, float dt)
{
  float accel_pitch = atan2f(ay, az);
  float accel_roll  = atan2f(-ax, sqrtf(ay*ay + az*az));

  pitch_est = 0.98f * (pitch_est + gy * dt) + 0.02f * accel_pitch;
  roll_est  = 0.98f * (roll_est  + gx * dt) + 0.02f * accel_roll;

  // Yaw: integracion del giroscopio Z con banda muerta anti-deriva.
  if (fabsf(gz) > GYRO_Z_DEADBAND) {
    yaw_est += gz * dt;
  }
  // Normalizar yaw a (-pi, pi]
  if (yaw_est >  3.14159265f) yaw_est -= 2.0f * 3.14159265f;
  if (yaw_est < -3.14159265f) yaw_est += 2.0f * 3.14159265f;

  // Euler (roll, pitch, yaw) → quaternion
  float cy = cosf(yaw_est   * 0.5f), sy = sinf(yaw_est   * 0.5f);
  float cp = cosf(pitch_est * 0.5f), sp = sinf(pitch_est * 0.5f);
  float cr = cosf(roll_est  * 0.5f), sr = sinf(roll_est  * 0.5f);

  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    imu_data.qw = cr*cp*cy + sr*sp*sy;
    imu_data.qx = sr*cp*cy - cr*sp*sy;
    imu_data.qy = cr*sp*cy + sr*cp*sy;
    imu_data.qz = cr*cp*sy - sr*sp*cy;
    imu_data.ax = ax; imu_data.ay = ay; imu_data.az = az;
    imu_data.gx = gx; imu_data.gy = gy; imu_data.gz = gz;
    xSemaphoreGive(imu_mutex);
  }
}


// ════════════════════════════════════════════════════════════════
//  Callback cmd_vel (Core 0 — micro-ROS executor)
// ════════════════════════════════════════════════════════════════
void cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msg_in;
  if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    g_linear_x  = (float)msg->linear.x;
    g_angular_z = (float)msg->angular.z;
    xSemaphoreGive(cmd_mutex);
  }
}


// ════════════════════════════════════════════════════════════════
//  Callback timer — publica IMU a 50 Hz
//  Si el IMU no arranco (imu_ok == false), imu_data conserva el
//  quaternion identidad (qw=1) y aceleraciones en 0: el mensaje
//  sigue siendo valido, solo que sin datos reales de sensor.
// ════════════════════════════════════════════════════════════════
void timer_callback(rcl_timer_t * timer, int64_t)
{
  if (!timer) return;
  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    imu_msg.orientation.w          = imu_data.qw;
    imu_msg.orientation.x          = imu_data.qx;
    imu_msg.orientation.y          = imu_data.qy;
    imu_msg.orientation.z          = imu_data.qz;
    imu_msg.linear_acceleration.x  = imu_data.ax;
    imu_msg.linear_acceleration.y  = imu_data.ay;
    imu_msg.linear_acceleration.z  = imu_data.az;
    imu_msg.angular_velocity.x     = imu_data.gx;
    imu_msg.angular_velocity.y     = imu_data.gy;
    imu_msg.angular_velocity.z     = imu_data.gz;
    xSemaphoreGive(imu_mutex);
  }
  RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
}


// ════════════════════════════════════════════════════════════════
//  Task motores — Core 1, 50 Hz
// ════════════════════════════════════════════════════════════════
void motors_task(void *)
{
  const TickType_t period      = pdMS_TO_TICKS(20);
  const TickType_t timeout_tks = pdMS_TO_TICKS(500);
  TickType_t last_wake         = xTaskGetTickCount();
  TickType_t last_cmd_time     = xTaskGetTickCount();

  while (true) {
    float lin = 0.0f, ang = 0.0f;
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      lin = g_linear_x; ang = g_angular_z;
      xSemaphoreGive(cmd_mutex);
    }
    if (fabsf(lin) > 0.001f || fabsf(ang) > 0.001f) {
      last_cmd_time = xTaskGetTickCount();
      apply_cmd_vel(lin, ang);
    } else if ((xTaskGetTickCount() - last_cmd_time) > timeout_tks) {
      apply_cmd_vel(0.0f, 0.0f);
    }
    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════════
//  Task IMU — Core 1, 100 Hz
//  Solo se ejecuta si imu_ok == true (ver setup()).
// ════════════════════════════════════════════════════════════════
void imu_task(void *)
{
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last_wake    = xTaskGetTickCount();

  while (true) {
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    float ax = ax_r * ACCEL_SCALE;
    float ay = ay_r * ACCEL_SCALE;
    float az = az_r * ACCEL_SCALE;
    float gx = gx_r * GYRO_SCALE;
    float gy = gy_r * GYRO_SCALE;
    float gz = gz_r * GYRO_SCALE;

    unsigned long now = micros();
    float dt = (last_imu_us > 0) ? (now - last_imu_us) * 1e-6f : 0.01f;
    last_imu_us = now;
    // Proteccion: dt fuera de rango razonable se descarta.
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    update_orientation(ax, ay, az, gx, gy, gz, dt);

    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════════
//  Inicializacion micro-ROS
// ════════════════════════════════════════════════════════════════
bool microros_init()
{
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "robot_node", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "robot_imu") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(&timer, &support,
        RCL_MS_TO_NS(20), timer_callback) != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel,
        &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  imu_msg.orientation_covariance[0] = 0.01f;
  imu_msg.orientation_covariance[4] = 0.01f;
  imu_msg.orientation_covariance[8] = 0.01f;

  return true;
}


// ════════════════════════════════════════════════════════════════
//  Task micro-ROS — Core 0
// ════════════════════════════════════════════════════════════════
void microros_task(void *)
{
  Serial.println("[INFO] Conectando WiFi...");
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(
    (char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);
  Serial.printf("[INFO] SSID: %s | Agente: %s:%d\n",
                WIFI_SSID, AGENT_IP, AGENT_PORT);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[INFO] Conectando al agente...");
    if (microros_init()) {
      Serial.println("[OK] micro-ROS listo — /robot_imu pub | /cmd_vel sub");
      break;
    }
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    Serial.println("[WARN] Reintentando en 3s...");
  }

  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(NULL);
}


// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== ROBOT ESP32 BOOT =====");

  motors_init();
  Serial.println("[OK] TB6612FNG listo");

  imu_mutex = xSemaphoreCreateMutex();
  cmd_mutex = xSemaphoreCreateMutex();

  // ── Inicializacion del IMU con manejo de fallo ────────────────
  //  I2C a 100 kHz (mas estable que 400 kHz con cableado real) y
  //  con timeout: una lectura fallida nunca cuelga el task.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(25);          // ms — evita bloqueos del bus I2C

  mpu.initialize();
  delay(100);                   // dar tiempo a que el MPU se asiente
  imu_ok = mpu.testConnection();

  if (imu_ok) {
    Serial.println("[OK] MPU6050 conectado (addr 0x68) — IMU activo");
    // El imu_task SOLO se lanza si el IMU respondio.
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 6, NULL, 1);
  } else {
    Serial.println("[WARN] MPU6050 no responde — robot SIN IMU.");
    Serial.println("[WARN] La teleoperacion funciona; /robot_imu");
    Serial.println("[WARN] publicara orientacion identidad (qw=1).");
  }

  // motors_task y microros_task se lanzan SIEMPRE: la teleoperacion
  // no depende del IMU.
  xTaskCreatePinnedToCore(motors_task,   "motors",   4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microros_task, "microros", 8192, NULL, 5, NULL, 0);

  Serial.printf("[INFO] Tasks activos (IMU: %s)\n", imu_ok ? "SI" : "NO");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }