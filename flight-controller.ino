#include <Wire.h>
#include <MPU6050.h>

// Define pins for motor control
const int motor1Pin = 13;
const int motor2Pin = 12;
const int motor3Pin = 11;
const int motor4Pin = 10;

// MPU6050 object
MPU6050 mpu;

// PID controller structure
struct PID {
  float Kp, Ki, Kd;
  float error, integral, derivative, lastError;
};

// PID controllers for roll, pitch, and yaw
PID rollPID, pitchPID, yawPID;

// Kalman filter structure for one axis
struct KalmanFilter {
  float Q_angle, Q_bias, R_measure;
  float P[2][2], x_hat[2];
  float P_hat[2][2], x_hat_minus[2], K[2];
};

// Kalman filters for roll, pitch, and yaw
KalmanFilter rollKF, pitchKF, yawKF;

// Variables for sensor data and control outputs
float ax, ay, az, gx, gy, gz;
float roll, pitch, yaw;
float desiredRoll, desiredPitch, desiredYaw;
float motor1Speed, motor2Speed, motor3Speed, motor4Speed;

// I2C Pins (RP2040)
const int SDA_PIN = 0;  // SDA (Data)
const int SCL_PIN = 1;  // SCL (Clock)

// Control variables for SBUS data
float xPWM, yPWM, zPWM, yawPWM;  // Store received PWM values (Roll, Pitch, Throttle, Yaw)

void setup() {
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);  // Use custom SDA/SCL pins
  Serial.begin(115200);           // Start serial communication for data input

  // Initialize MPU6050
  mpu.initialize();
  
  // Initialize PID controller parameters
  rollPID.Kp = 1.0;
  rollPID.Ki = 0.01;
  rollPID.Kd = 0.1;
  
  pitchPID.Kp = 1.0;
  pitchPID.Ki = 0.01;
  pitchPID.Kd = 0.1;

  yawPID.Kp = 1.0;
  yawPID.Ki = 0.01;
  yawPID.Kd = 0.1;

  // Initialize Kalman filter parameters
  rollKF.Q_angle = 0.001;
  rollKF.Q_bias = 0.003;
  rollKF.R_measure = 0.03;
  pitchKF.Q_angle = 0.001;
  pitchKF.Q_bias = 0.003;
  pitchKF.R_measure = 0.03;
  yawKF.Q_angle = 0.001;
  yawKF.Q_bias = 0.003;
  yawKF.R_measure = 0.03;

  // Initialize Kalman filter matrices
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      rollKF.P[i][j] = 0;
      pitchKF.P[i][j] = 0;
      yawKF.P[i][j] = 0;
    }
  }
}

// Function to read sensor data and update Kalman filter
void readSensorData() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Update Kalman filter for roll, pitch, and yaw
  roll = kalmanFilterUpdate(rollKF, gx, ax);
  pitch = kalmanFilterUpdate(pitchKF, gy, ay);
  yaw = kalmanFilterUpdate(yawKF, gz, az);
}

// Kalman filter update for one axis
float kalmanFilterUpdate(KalmanFilter &kf, float gyroRate, float accelAngle) {
  // Prediction
  kf.x_hat_minus[0] = kf.x_hat[0] + gyroRate * 0.01;  // Assuming a 10ms loop time
  kf.x_hat_minus[1] = kf.x_hat[1];

  kf.P_hat[0][0] = kf.P[0][0] + kf.Q_angle;
  kf.P_hat[0][1] = kf.P[0][1] + kf.Q_bias;
  kf.P_hat[1][0] = kf.P[1][0] + kf.Q_bias;
  kf.P_hat[1][1] = kf.P[1][1] + kf.R_measure;

  // Measurement update
  kf.K[0] = kf.P_hat[0][0] / kf.P_hat[0][0];
  kf.K[1] = kf.P_hat[1][0] / kf.P_hat[0][0];

  kf.x_hat[0] = kf.x_hat_minus[0] + kf.K[0] * (accelAngle - kf.x_hat_minus[0]);
  kf.x_hat[1] = kf.x_hat_minus[1] + kf.K[1] * (accelAngle - kf.x_hat_minus[0]);

  kf.P[0][0] = kf.P_hat[0][0] - kf.K[0] * kf.P_hat[0][0];
  kf.P[0][1] = kf.P_hat[0][1] - kf.K[0] * kf.P_hat[0][1];
  kf.P[1][0] = kf.P_hat[1][0] - kf.K[1] * kf.P_hat[0][0];
  kf.P[1][1] = kf.P_hat[1][1] - kf.K[1] * kf.P_hat[0][1];

  return kf.x_hat[0];
}

// Function to receive PWM control values via Serial
void receiveControlInput() {
  if (Serial.available() >= 4 * sizeof(float)) {
    // Read PWM values for Roll, Pitch, Throttle (Z), and Yaw
    Serial.readBytes((char *)&xPWM, sizeof(float));
    Serial.readBytes((char *)&yPWM, sizeof(float));
    Serial.readBytes((char *)&zPWM, sizeof(float));
    Serial.readBytes((char *)&yawPWM, sizeof(float));
  }
}

// Function to calculate PID outputs and control motors
void controlMotors() {
  // Calculate PID outputs for roll, pitch, and yaw
  float rollError = desiredRoll - roll;
  rollPID.integral += rollError;
  rollPID.derivative = rollError - rollPID.lastError;
  rollPID.lastError = rollError;
  float rollOutput = rollPID.Kp * rollError + rollPID.Ki * rollPID.integral + rollPID.Kd * rollPID.derivative;

  float pitchError = desiredPitch - pitch;
  pitchPID.integral += pitchError;
  pitchPID.derivative = pitchError - pitchPID.lastError;
  pitchPID.lastError = pitchError;
  float pitchOutput = pitchPID.Kp * pitchError + pitchPID.Ki * pitchPID.integral + pitchPID.Kd * pitchPID.derivative;

  float yawError = desiredYaw - yaw;
  yawPID.integral += yawError;
  yawPID.derivative = yawError - yawPID.lastError;
  yawPID.lastError = yawError;
  float yawOutput = yawPID.Kp * yawError + yawPID.Ki * yawPID.integral + yawPID.Kd * yawPID.derivative;

  // Map PID outputs to motor speeds (motor mixing)
  motor1Speed = constrain(1500 + rollOutput + pitchOutput - yawOutput, 1000, 2000);
  motor2Speed = constrain(1500 - rollOutput + pitchOutput + yawOutput, 1000, 2000);
  motor3Speed = constrain(1500 - rollOutput - pitchOutput - yawOutput, 1000, 2000);
  motor4Speed = constrain(1500 + rollOutput - pitchOutput + yawOutput, 1000, 2000);

  // Set motor speeds using PWM
  analogWrite(motor1Pin, motor1Speed);
  analogWrite(motor2Pin, motor2Speed);
  analogWrite(motor3Pin, motor3Speed);
  analogWrite(motor4Pin, motor4Speed);
}

void loop() {
  receiveControlInput();  // Receive control input (PWM values)
  readSensorData();       // Read sensor data and update Kalman filters
  controlMotors();        // Control motors based on PID outputs
}
