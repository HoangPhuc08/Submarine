#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/quaternion.h> // Required for imu::Quaternion

// BNO055
#define I2C_SDA    42     // GPIO pin for I2C SDA (e.g., ESP32 pin 2)
#define I2C_SCL    41     // GPIO pin for I2C SCL (e.g., ESP32 pin 1)

TwoWire I2CBNO = TwoWire(0); // For ESP32, '0' typically corresponds to the default I2C bus

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);

// Variables for integration - NOW 3D!
float currentVelocityX = 0.0;
float currentVelocityY = 0.0;
float currentVelocityZ = 0.0;

float currentDisplacementX = 0.0;
float currentDisplacementY = 0.0;
float currentDisplacementZ = 0.0;

unsigned long previousMillis = 0; // For delta_t calculation

// Variables for consistent loop timing
unsigned long loopTimer = 0;
const int loopInterval = 500; // Print sensor data every 500ms (2Hz) to avoid overwhelming serial monitor.
                              // You can adjust this for faster updates if needed.

// Threshold for Zero-Velocity Update (ZUPT)
const float ACCEL_NOISE_THRESHOLD = 0.05; // m/s^2. Adjust this based on observed sensor noise
                                          // A value of 0.05 m/s^2 means +/- 0.05 m/s^2 is considered noise.

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect (useful for some boards)

  Serial.println("Initializing BNO055...");

  // Initialize I2C for BNO055
  I2CBNO.begin(I2C_SDA, I2C_SCL);

  if (!bno.begin()) {
    Serial.println("Ooops, No BNO055 detected ... Check your wiring or I2C ADDR!");
    // Removed while(true) here to allow the message to print and potentially continue
    // if you uncomment other parts or just want to see the failure message.
    // For a production system, you might still want an infinite loop here.
    Serial.println("Halting program. Please check wiring.");
    while(true); // Re-added for safety in this context.
  }

  Serial.println("BNO055 initialized successfully!");
  Serial.println("Waiting for BNO055 calibration. Move the sensor slowly to calibrate.");
  Serial.println("Calibration values: Sys=3, Gyro=3, Accel=3, Mag=3 is ideal.");

  // Wait for the sensor to be calibrated
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  // Loop until system calibration is at least 1 (some basic calibration) or higher
  // For best results, wait until system (Sys) is 3.
  while (system < 1) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION STATUS: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
    delay(500); // Give it some time and don't spam serial
  }

  Serial.println("\nBNO055 Calibrated (at least partially)!");
  Serial.println("Ready to read data.");

  // Initialize previousMillis for delta_t calculation
  previousMillis = millis();
}


// Modified to accept a full 3D acceleration vector
void updateSensorData(sensors_vec_t acceleration, float deltaTime) {
  // Calculate the magnitude of the acceleration vector
  float accelMagnitude = sqrt(
    acceleration.x * acceleration.x +
    acceleration.y * acceleration.y +
    acceleration.z * acceleration.z
  );

  // If acceleration magnitude is within the noise threshold, assume stationary (ZUPT)
  if (accelMagnitude < ACCEL_NOISE_THRESHOLD) {
    currentVelocityX = 0.0;
    currentVelocityY = 0.0;
    currentVelocityZ = 0.0;
    // You typically don't reset displacement during ZUPT, as it's an accumulation.
    // Resetting displacement would mean it only tracks movement while actively moving.
    // currentDisplacementX = 0.0; // Only uncomment if you want displacement to reset when stationary
    // currentDisplacementY = 0.0;
    // currentDisplacementZ = 0.0;
  } else {
    // Integrate acceleration to get velocity for each axis
    currentVelocityX += acceleration.x * deltaTime;
    currentVelocityY += acceleration.y * deltaTime;
    currentVelocityZ += acceleration.z * deltaTime;
  }

  // Integrate velocity to get displacement for each axis
  currentDisplacementX += currentVelocityX * deltaTime;
  currentDisplacementY += currentVelocityY * deltaTime;
  currentDisplacementZ += currentVelocityZ * deltaTime;
}


void loop() {
  unsigned long currentMillis = millis();

  // Run the sensor reading and printing loop at a fixed rate
  if (currentMillis - loopTimer >= loopInterval) {
    loopTimer = currentMillis; // Reset the timer for the next interval

    // --- Get all sensor data ---
    sensors_event_t accelData, gyroData, magData, eulerData, linearAccelData, gravityData;
    imu::Quaternion quat;
    int8_t temp;
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;

    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER); // Heading, Roll, Pitch
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); // Acceleration with gravity removed
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);         // Gravity vector
    quat = bno.getQuat();                                                // Quaternions
    temp = bno.getTemp();                                                // Chip temperature
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);    // Current calibration status

    // --- Print all data to Serial ---
    Serial.println("------------------------------------");

    // Calibration Status
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system_cal, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro_cal, DEC);
    Serial.print(" Accel=");
    Serial.print(accel_cal, DEC);
    Serial.print(" Mag=");
    Serial.println(mag_cal, DEC);

    // Temperature
    Serial.print("Temp:         "); Serial.print(temp); Serial.println(" C");

    // Raw Accelerometer data (includes gravity)
    Serial.print("Accel (raw):  X="); Serial.print(accelData.acceleration.x, 3);
    Serial.print(" Y="); Serial.print(accelData.acceleration.y, 3);
    Serial.print(" Z="); Serial.print(accelData.acceleration.z, 3);
    Serial.println(" m/s^2");

    // Linear Acceleration (gravity removed) - this is what you are using for integration
    Serial.print("Linear Accel: X="); Serial.print(linearAccelData.acceleration.x, 3);
    Serial.print(" Y="); Serial.print(linearAccelData.acceleration.y, 3);
    Serial.print(" Z="); Serial.print(linearAccelData.acceleration.z, 3);
    Serial.println(" m/s^2");

    // Gyroscope data
    Serial.print("Gyro:         X="); Serial.print(gyroData.gyro.x, 3);
    Serial.print(" Y="); Serial.print(gyroData.gyro.y, 3);
    Serial.print(" Z="); Serial.print(gyroData.gyro.z, 3);
    Serial.println(" rad/s");

    // Magnetometer data
    Serial.print("Mag:          X="); Serial.print(magData.magnetic.x, 3);
    Serial.print(" Y="); Serial.print(magData.magnetic.y, 3);
    Serial.print(" Z="); Serial.print(magData.magnetic.z, 3);
    Serial.println(" uT");

    // Euler angles (Heading, Roll, Pitch)
    Serial.print("Euler:        H="); Serial.print(eulerData.orientation.x, 3); // Heading (Yaw)
    Serial.print(" R="); Serial.print(eulerData.orientation.y, 3); // Roll
    Serial.print(" P="); Serial.print(eulerData.orientation.z, 3); // Pitch
    Serial.println(" degrees");

    // Quaternions
    Serial.print("Quat:         w="); Serial.print(quat.w(), 3);
    Serial.print(" x="); Serial.print(quat.x(), 3);
    Serial.print(" y="); Serial.print(quat.y(), 3);
    Serial.print(" z="); Serial.print(quat.z(), 3);
    Serial.println("");

    // Gravity Vector
    Serial.print("Gravity:      X="); Serial.print(gravityData.acceleration.x, 3);
    Serial.print(" Y="); Serial.print(gravityData.acceleration.y, 3);
    Serial.print(" Z="); Serial.print(gravityData.acceleration.z, 3);
    Serial.println(" m/s^2");

    // --- Your updated displacement tracking (now using the linear accel in 3D) ---

    // Calculate the time difference since the last integration step
    float delta_t = (currentMillis - previousMillis) / 1000.0f; // Convert ms to seconds
    previousMillis = currentMillis; // Update for the next iteration

    // Update velocity and displacement only if there was a time difference
    if (delta_t > 0) {
      updateSensorData(linearAccelData.acceleration, delta_t); // Pass the entire acceleration vector
    }

    Serial.print("\n--- Integrated Data (3-axis) ---");
    Serial.print("\nDelta T:        "); Serial.print(delta_t, 5); Serial.println(" s");

    // Print Linear Acceleration for all axes
    Serial.print("Linear Accel: X="); Serial.print(linearAccelData.acceleration.x, 3);
    Serial.print(" Y="); Serial.print(linearAccelData.acceleration.y, 3);
    Serial.print(" Z="); Serial.print(linearAccelData.acceleration.z, 3);
    Serial.println(" m/s^2");

    // Print Current Velocity for all axes
    Serial.print("Current Vel:  X="); Serial.print(currentVelocityX, 3);
    Serial.print(" Y="); Serial.print(currentVelocityY, 3);
    Serial.print(" Z="); Serial.print(currentVelocityZ, 3);
    Serial.println(" m/s");

    // Print Current Displacement for all axes
    Serial.print("Current Disp: X="); Serial.print(currentDisplacementX, 3);
    Serial.print(" Y="); Serial.print(currentDisplacementY, 3);
    Serial.print(" Z="); Serial.print(currentDisplacementZ, 3);
    Serial.println(" m");

    Serial.println("------------------------------------\n");
  }

  // No delay here, as the loop is controlled by the `loopInterval` check.
  // This allows other non-blocking tasks to run if added later.
}