/*
 * PROYECTO: Auto-balanceo (versión “agresiva”) + CONTROL DE POTENCIA
 * PLATAFORMA: Arduino UNO
 * DRIVERS:  Motor1 HW-135 (1/8 microstep)  |  Motor2 EasyDriver (1/8 microstep)
 * SENSOR:   GY-521 (MPU6050)
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// ======================= CONFIG GLOBAL ====================
// --- (A) CONFIG. DE MOTORES ---
#define DRIVER_TYPE 1        // AccelStepper: 1 = driver step/dir
// Motor 1 -> HW-135
#define stepPin1 2
#define dirPin1  3
// Motor 2 -> EasyDriver
#define stepPin2 4
#define dirPin2  5

// Valores base “agresivos”:
const int BASE_MAX_SPEED   = 25000;  // [pasos/s]
const int BASE_ACCEL       = 12000;  // [pasos/s^2]

// --- (B) CONFIG. DEL SENSOR (GY-521 / MPU6050) ---
Adafruit_MPU6050 mpu;
float pitch = 0.0f;           // Ángulo estimado (grados)
float gyro_y_cal = 0.0f;      // Offset giroscopio en eje Y
unsigned long t_prev_ms = 0;  // Marca de tiempo para dt
const float alpha = 0.98f;    // Filtro complementario

// --- (C) CONFIG. DEL CONTROL PID (usado como PD) ---
double Setpoint, Input, Output;
// Ganancias originales:
double Kp = 250, Ki = 0, Kd = 30;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- (D) SEGURIDAD ---
const float ANGLE_CUTOFF = 30.0f;   // [deg] si |pitch| supera esto, se detiene
bool fault_latched = false;         // “latch” de seguridad
const float ANGLE_RESET = 10.0f;    // [deg] umbral para salir del fallo

// --- (E) OBJETOS DE MOTOR ---
AccelStepper stepper1(DRIVER_TYPE, stepPin1, dirPin1);
AccelStepper stepper2(DRIVER_TYPE, stepPin2, dirPin2);

// --- (F) TRAZAS (Serial Plotter) ---
const unsigned long PLOT_DT_MS = 20; // 50 Hz de envío
unsigned long last_plot_ms = 0;

// --- (G) CONTROL DE “POTENCIA” ---
const int PIN_POTENCIA = A0;          // Potenciómetro 0–5 V
volatile float power_scale = 1.00f;    // Escala 0.00–1.00 (0–100%)
const float POWER_MIN = 0.30f;         // Evita que el robot quede sin reacción
const float POWER_MAX = 1.00f;         // Tope seguro (100%)
unsigned long last_power_update = 0;
const unsigned long POWER_UPDATE_MS = 100; // Refresco de potencia (10 Hz)

// Efectivos (derivados de power_scale)
int eff_max_speed = BASE_MAX_SPEED;
int eff_accel     = BASE_ACCEL;


// ==========================================================
// ===============   SECCIÓN: SENSORES   ====================
// ==========================================================

void calibrarGiroY(uint16_t n_lecturas = 1000, uint16_t delay_ms = 3) {
  gyro_y_cal = 0;
  for (uint16_t i = 0; i < n_lecturas; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_y_cal += g.gyro.y;      // [rad/s]
    delay(delay_ms);
  }
  gyro_y_cal /= n_lecturas;
}

float leerPitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long t_now = millis();
  float dt = (t_now - t_prev_ms) / 1000.0f;
  t_prev_ms = t_now;

  // Estimación con acelerómetro (inclinación instantánea)
  float pitch_acc = atan2(a.acceleration.x, a.acceleration.z) * 180.0f / PI;

  // Estimación con giroscopio (integración de velocidad angular)
  float gyro_y = g.gyro.y - gyro_y_cal; // [rad/s] corregido
  pitch += gyro_y * dt * 180.0f / PI;   // integrar y pasar a grados

  // Filtro complementario
  pitch = alpha * pitch + (1.0f - alpha) * pitch_acc;

  return pitch;
}


// ==========================================================
// ===============   SECCIÓN: MOTORES    ====================
// ==========================================================

void aplicarLimitesMotores() {
  // Ajusta límites efectivos según power_scale
  eff_max_speed = (int)(BASE_MAX_SPEED * power_scale);
  eff_accel     = (int)(BASE_ACCEL     * power_scale);

  // Asegurar mínimos distintos de cero
  if (eff_max_speed < 200) eff_max_speed = 200;
  if (eff_accel     < 200) eff_accel     = 200;

  stepper1.setMaxSpeed(eff_max_speed);
  stepper1.setAcceleration(eff_accel);

  stepper2.setMaxSpeed(eff_max_speed);
  stepper2.setAcceleration(eff_accel);
}

void configurarMotores() {
  aplicarLimitesMotores();
}

/**
 * @param u  Señal de control (Output del PID), en [pasos/s]
 */
void mandarMotores(double u) {
  // Escalar la orden final también puede ayudar a “sentir” la potencia
  double u_scaled = u * power_scale;

  stepper1.setSpeed(u_scaled);
  stepper2.setSpeed(-u_scaled);

  // AccelStepper en modo “runSpeed” (no bloqueante)
  stepper1.runSpeed();
  stepper2.runSpeed();
}

void pararMotores() {
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper1.runSpeed();
  stepper2.runSpeed();
}


// ==========================================================
// ===============   SECCIÓN: PID        ====================
// ==========================================================

void configurarPID() {
  Setpoint = 0.0;  // objetivo = vertical (por eso en el plot "set:" se ve 0)
  myPID.SetMode(AUTOMATIC);
  // Límites simétricos basados en el valor BASE (no escalado):
  myPID.SetOutputLimits(-BASE_MAX_SPEED, BASE_MAX_SPEED);
}


// ==========================================================
// ===============   SECCIÓN: POTENCIA   ====================
// ==========================================================

// Lee potenciómetro A0 y actualiza power_scale (0.30–1.00)
void leerPotenciaDesdePot() {
  int raw = analogRead(PIN_POTENCIA); // 0..1023
  float p = (float)raw / 1023.0f;     // 0..1
  // Mapear a [POWER_MIN, POWER_MAX]
  p = POWER_MIN + p * (POWER_MAX - POWER_MIN);
  power_scale = p;
}

// Parsea comandos tipo: P=70  (70%)
void leerPotenciaDesdeSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  // Admite "P=70", "p=50", "P 80", etc.
  s.replace(" ", "");
  s.toUpperCase();

  if (s.startsWith("P=") || s.startsWith("P")) {
    int idx = s.indexOf('=');
    String num;
    if (idx >= 0) num = s.substring(idx + 1);
    else          num = s.substring(1);

    int pct = num.toInt(); // 0..100
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    float p = (float)pct / 100.0f;
    // Mapear a [POWER_MIN, POWER_MAX]
    p = POWER_MIN + p * (POWER_MAX - POWER_MIN);
    power_scale = p;

    Serial.print("[Potencia] Fijada por Serial a ");
    Serial.print(pct);
    Serial.println("%");
  }
}

// Refresca potencia cada POWER_UPDATE_MS y aplica nuevos límites
void actualizarPotencia() {
  unsigned long now = millis();
  if (now - last_power_update >= POWER_UPDATE_MS) {
    last_power_update = now;
    leerPotenciaDesdePot();   // perilla A0 (si no hay perilla, queda al mínimo/ruido)
    leerPotenciaDesdeSerial(); // permite sobrescribir por Serial
    aplicarLimitesMotores();
  }
}


// ==========================================================
// ===============   SECCIÓN: TRAZAS     ====================
// ==========================================================

void trazarSerial(double angulo, double set, double u) {
  unsigned long now = millis();
  if (now - last_plot_ms >= PLOT_DT_MS) {
    last_plot_ms = now;
    Serial.print("angulo: ");   Serial.print(angulo, 3);
    Serial.print(", set: ");    Serial.print(set, 3);
    Serial.print(", u: ");      Serial.print(u, 1);
    Serial.print(", pot: ");    Serial.print(power_scale, 2); // para ver la potencia
    Serial.print(", vmax: ");   Serial.print(eff_max_speed);
    Serial.print(", acc: ");    Serial.println(eff_accel);
  }
}

void informarFallo(const char* motivo) {
  Serial.print("[SEGURIDAD] ");
  Serial.println(motivo);
}


// ==========================================================
// =====================   SETUP   ==========================
// ==========================================================
void setup() {
  Serial.begin(115200);

  // --- SENSORES ---
  if (!mpu.begin()) {
    Serial.println("ERROR: No se encontró MPU6050.");
    while (1); // bloqueo intencional
  }
  Serial.println("MPU6050 OK. Calibrando... No mover el robot.");
  calibrarGiroY();
  Serial.print("Offset Giro Y = ");
  Serial.println(gyro_y_cal, 6);
  t_prev_ms = millis();

  // --- MOTORES ---
  configurarMotores();

  // --- PID ---
  configurarPID();

  // --- POTENCIA ---
  pinMode(PIN_POTENCIA, INPUT);
  actualizarPotencia(); // inicial

  Serial.println("Inicio correcto. Comandos: P=## (0..100). Perilla en A0.");
}


// ==========================================================
// ======================   LOOP   ==========================
// ==========================================================
void loop() {
  // ----- ACTUALIZAR POTENCIA (perilla/serial) -----
  actualizarPotencia();

  // ----- SENSORES -----
  float ang = leerPitch(); // (grados)

  // ----- SEGURIDAD -----
  if (!fault_latched && fabs(ang) > ANGLE_CUTOFF) {
    fault_latched = true;
    informarFallo("Corte por ángulo (exceso de inclinación).");
  }
  if (fault_latched && fabs(ang) < ANGLE_RESET) {
    fault_latched = false;
    Serial.println("[SEGURIDAD] Restablecido (ángulo dentro de rango).");
  }

  // ----- PID -----
  Input = ang;
  if (fault_latched) {
    Output = 0;
  } else {
    myPID.Compute(); // Kp,Ki,Kd originales
  }

  // ----- MOTORES -----
  if (fault_latched) {
    pararMotores();
  } else {
    mandarMotores(Output);
  }

  // ----- TRAZAS -----
  trazarSerial(ang, Setpoint, Output);
}
