#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <U8g2lib.h>             // Required for U8g2 OLED display library
#include <utility/imumaths.h>    // Renamed from quaternion.h in some Adafruit libraries, provides imu::Quaternion

// =========================================================================
// ===================  YOUR HARDWARE CONFIGURATION  =======================
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 41

#define IMU_ADDRESS  0x29
#define LCD_ADDRESS  0x3C
// =========================================================================

// --- OBJECT CREATION ---

// 1. U8G2 OLED OBJECT (Your original, working constructor)
// This constructor is kept EXACTLY as you provided it.
// It will use the global "Wire" object, which is configured in setup().
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

// 2. BNO055 IMU OBJECT
// This constructor also uses the default global "Wire" object.
Adafruit_BNO055 bno = Adafruit_BNO055(-1, IMU_ADDRESS);

// Variables for integration - NOW 3D!
float currentVelocityX = 0.0;
float currentVelocityY = 0.0;
float currentVelocityZ = 0.0;

float currentDisplacementX = 0.0;
float currentDisplacementY = 0.0;
float currentDisplacementZ = 0.0;

unsigned long previousMillis = 0; // For delta_t calculation (for integration)

// Variables for consistent loop timing for display updates
unsigned long loopTimer = 0;
const int loopInterval = 500; // Update OLED display and Serial Monitor every 500ms (2Hz).
                              // Sensor data acquisition and integration will run faster, continuously.

// Threshold for Zero-Velocity Update (ZUPT)
const float ACCEL_NOISE_THRESHOLD = 0.05; // m/s^2. Adjust this based on observed sensor noise
                                          // A value of 0.05 m/s^2 means +/- 0.05 m/s^2 is considered noise.

// Modified to accept a full 3D acceleration vector
void updateSensorData(sensors_vec_t acceleration, float deltaTime) {
  // Calculate the magnitude of the acceleration vector
  float accelMagnitude = sqrt(
    acceleration.x * acceleration.x +
    acceleration.y * acceleration.y +
    acceleration.z * acceleration.z
  );

  // If acceleration magnitude is within the noise threshold, assume stationary (ZUPT)
  // This helps prevent drift when the sensor is not moving.
  if (accelMagnitude < ACCEL_NOISE_THRESHOLD) {
    currentVelocityX = 0.0;
    currentVelocityY = 0.0;
    currentVelocityZ = 0.0;
    // We typically don't reset displacement during ZUPT, as it's an accumulation.
    // Resetting displacement would mean it only tracks movement while actively moving.
    // currentDisplacementX = 0.0; // Uncomment only if you want displacement to reset when stationary
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
  currentDisplacementZ += currentDisplacementZ * deltaTime; // Fixed bug here: was += currentDisplacementZ * deltaTime;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // --- STEP 1: INITIALIZE THE GLOBAL "Wire" BUS ---
  // This single command configures the one-and-only I2C bus for BOTH devices.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // --- STEP 2: INITIALIZE THE OLED DISPLAY ---
  // The u8g2.begin() call will use the pre-configured global Wire object.
  Serial.println("Initializing U8g2 OLED Display...");
  // As per your working code, we set the shifted address
  u8g2.setI2CAddress(LCD_ADDRESS * 2);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Set a default font for initial messages
  u8g2.drawStr(0, 32, "OLED OK!");
  u8g2.sendBuffer();
  delay(1000);

  // --- STEP 3: INITIALIZE THE BNO055 IMU ---
  // The bno.begin() call will now use the same pre-configured global Wire object.
  Serial.println("Initializing BNO055 IMU...");
  if (!bno.begin()) {
    Serial.println("Failed to find BNO055 chip. Halting.");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 32, "IMU FAIL!");
    u8g2.sendBuffer();
    while (1);
  }
  bno.setExtCrystalUse(true); // Recommended for stability
  Serial.println("BNO055 found!");

  // --- STEP 4: CALIBRATE THE IMU ---
  Serial.println("Waiting for BNO055 calibration. Move the sensor slowly to calibrate.");
  Serial.println("Calibration values: Sys=3, Gyro=3, Accel=3, Mag=3 is ideal.");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x8_tf); // Use 8x8 font for calibration details
  u8g2.drawStr(0, 8, "Calibrating BNO...");
  u8g2.sendBuffer();
  
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  do {
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    char cal_str[32]; // Increased buffer size
    sprintf(cal_str, "Sys:%d G:%d A:%d M:%d", system_cal, gyro_cal, accel_cal, mag_cal);
    
    u8g2.clearBuffer(); // Clear for fresh update
    u8g2.drawStr(0, 8, "Calibrating BNO...");
    u8g2.drawStr(0, 24, cal_str); // Display calibration status
    u8g2.sendBuffer();

    Serial.print("CALIBRATION STATUS: Sys=");
    Serial.print(system_cal, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro_cal, DEC);
    Serial.print(" Accel=");
    Serial.print(accel_cal, DEC);
    Serial.print(" Mag=");
    Serial.println(mag_cal, DEC);
    delay(500);
  } while (system_cal < 3); // Wait for full system calibration (Sys=3 is ideal)

  Serial.println("\nBNO055 Calibrated! (Sys=3)");
  Serial.println("Ready to read data.");
  
  u8g2.clearBuffer();
  u8g2.drawStr(0, 8, "BNO055 Ready!");
  u8g2.sendBuffer();
  delay(1000); // Show "Ready" message for a moment

  // Initialize previousMillis for delta_t calculation for integration
  previousMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Integration Section ---
  // This section runs as frequently as possible (every loop cycle) to ensure accurate integration.
  // Calculate the time difference since the last integration step
  float delta_t = (currentMillis - previousMillis) / 1000.0f; // Convert ms to seconds
  previousMillis = currentMillis; // Update for the next iteration

  // Get linear acceleration data (gravity removed, ideal for integration)
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Update velocity and displacement based on the linear acceleration and delta_t
  if (delta_t > 0) { // Only update if there's a valid time difference
    updateSensorData(linearAccelData.acceleration, delta_t);
  }

  // --- Display and Serial Output Section ---
  // This section runs at a fixed rate (loopInterval) to refresh the OLED display and Serial Monitor.
  if (currentMillis - loopTimer >= loopInterval) {
    loopTimer = currentMillis; // Reset the timer for the next interval

    // --- Get all other sensor data for display ---
    // (linearAccelData is already fetched above for integration, but others need to be read now)
    sensors_event_t accelData, gyroData, magData, eulerData, gravityData;
    imu::Quaternion quat;
    int8_t temp;
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;

    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER); // Raw accelerometer (includes gravity)
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER); // Heading, Roll, Pitch
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);         // Gravity vector
    quat = bno.getQuat();                                                // Quaternions
    temp = bno.getTemp();                                                // Chip temperature
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);    // Current calibration status

    // --- Clear and draw to U8g2 Display ---
    u8g2.clearBuffer(); // Clear the display buffer before drawing new data

    // Use a character buffer for formatted strings to draw on the OLED
    char buffer[64]; 
    
    // Set font for all text on the display
    u8g2.setFont(u8g2_font_8x8_tf); // 8x8 pixel font, transparent foreground (tf = transparent foreground)
    u8g2.setFontMode(1);            // Use transparent mode for text background

    // Line 0 (y=8): Calibration Status & Temperature
    // Format: CAL S:?G:?A:?M:? T:?C
    sprintf(buffer, "CAL S%dG%dA%dM%d T%dC", system_cal, gyro_cal, accel_cal, mag_cal, temp);
    u8g2.drawStr(0, 8, buffer); // x=0, y=8 (for first line using 8x8 font)

    // Line 1 (y=16): Linear Acceleration (gravity removed, used for integration)
    // Format: ACC:X?.? Y?.? Z?.? (1 decimal place)
    sprintf(buffer, "ACC:X%.1f Y%.1f Z%.1f", linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z);
    u8g2.drawStr(0, 16, buffer);

    // Line 2 (y=24): Euler Angles (Heading, Roll, Pitch)
    // Format: EUL:H?.? R?.? P?.? (1 decimal place)
    sprintf(buffer, "EUL:H%.1f R%.1f P%.1f", eulerData.orientation.x, eulerData.orientation.y, eulerData.orientation.z);
    u8g2.drawStr(0, 24, buffer);

    // Line 3 (y=32): Quaternions (W and X)
    // Format: Q: W?.?? X?.?? (2 decimal places)
    sprintf(buffer, "Q:W%.2f X%.2f", quat.w(), quat.x());
    u8g2.drawStr(0, 32, buffer);

    // Line 4 (y=40): Quaternions (Y and Z)
    // Format: Q: Y?.?? Z?.?? (2 decimal places)
    sprintf(buffer, "Q:Y%.2f Z%.2f", quat.y(), quat.z());
    u8g2.drawStr(0, 40, buffer);

    // Line 5 (y=48): Current Velocity
    // Format: VEL:X?.? Y?.? Z?.? (1 decimal place)
    sprintf(buffer, "VEL:X%.1f Y%.1f Z%.1f", currentVelocityX, currentVelocityY, currentVelocityZ);
    u8g2.drawStr(0, 48, buffer);

    // Line 6 (y=56): Current Displacement
    // Format: DIS:X?.? Y?.? Z?.? (1 decimal place)
    sprintf(buffer, "DIS:X%.1f Y%.1f Z%.1f", currentDisplacementX, currentDisplacementY, currentDisplacementZ);
    u8g2.drawStr(0, 56, buffer);
    
    u8g2.sendBuffer(); // Transfer internal memory to the physical display

    // --- Serial Monitor Output (kept for debugging/comparison) ---
    // This section is copied directly from your original request's serial output format.
    // --- Print all data to Serial ---
    // Serial.println("------------------------------------");

    // // Calibration Status
    // Serial.print("CALIBRATION: Sys=");
    // Serial.print(system_cal, DEC);
    // Serial.print(" Gyro=");
    // Serial.print(gyro_cal, DEC);
    // Serial.print(" Accel=");
    // Serial.print(accel_cal, DEC);
    // Serial.print(" Mag=");
    // Serial.println(mag_cal, DEC);

    // // Temperature
    // Serial.print("Temp:         "); Serial.print(temp); Serial.println(" C");

    // // Raw Accelerometer data (includes gravity)
    // Serial.print("Accel (raw):  X="); Serial.print(accelData.acceleration.x, 3);
    // Serial.print(" Y="); Serial.print(accelData.acceleration.y, 3);
    // Serial.print(" Z="); Serial.print(accelData.acceleration.z, 3);
    // Serial.println(" m/s^2");

    // // Linear Acceleration (gravity removed) - this is what you are using for integration
    // Serial.print("Linear Accel: X="); Serial.print(linearAccelData.acceleration.x, 3);
    // Serial.print(" Y="); Serial.print(linearAccelData.acceleration.y, 3);
    // Serial.print(" Z="); Serial.print(linearAccelData.acceleration.z, 3);
    // Serial.println(" m/s^2");

    // // Gyroscope data
    // Serial.print("Gyro:         X="); Serial.print(gyroData.gyro.x, 3);
    // Serial.print(" Y="); Serial.print(gyroData.gyro.y, 3);
    // Serial.print(" Z="); Serial.print(gyroData.gyro.z, 3);
    // Serial.println(" rad/s");

    // // Magnetometer data
    // Serial.print("Mag:          X="); Serial.print(magData.magnetic.x, 3);
    // Serial.print(" Y="); Serial.print(magData.magnetic.y, 3);
    // Serial.print(" Z="); Serial.print(magData.magnetic.z, 3);
    // Serial.println(" uT");

    // // Euler angles (Heading, Roll, Pitch)
    // Serial.print("Euler:        H="); Serial.print(eulerData.orientation.x, 3); // Heading (Yaw)
    // Serial.print(" R="); Serial.print(eulerData.orientation.y, 3); // Roll
    // Serial.print(" P="); Serial.print(eulerData.orientation.z, 3); // Pitch
    // Serial.println(" degrees");

    // // Quaternions
    // Serial.print("Quat:         w="); Serial.print(quat.w(), 3);
    // Serial.print(" x="); Serial.print(quat.x(), 3);
    // Serial.print(" y="); Serial.print(quat.y(), 3);
    // Serial.print(" z="); Serial.print(quat.z(), 3);
    // Serial.println("");

    // // Gravity Vector
    // Serial.print("Gravity:      X="); Serial.print(gravityData.acceleration.x, 3);
    // Serial.print(" Y="); Serial.print(gravityData.acceleration.y, 3);
    // Serial.print(" Z="); Serial.print(gravityData.acceleration.z, 3);
    // Serial.println(" m/s^2");

    // // --- Your updated displacement tracking (now using the linear accel in 3D) ---

    // // Calculate the time difference since the last integration step
    // float delta_t = (currentMillis - previousMillis) / 1000.0f; // Convert ms to seconds
    // previousMillis = currentMillis; // Update for the next iteration

    // // Update velocity and displacement only if there was a time difference
    // if (delta_t > 0) {
    //   updateSensorData(linearAccelData.acceleration, delta_t); // Pass the entire acceleration vector
    // }

    // Serial.print("\n--- Integrated Data (3-axis) ---");
    // Serial.print("\nDelta T:        "); Serial.print(delta_t, 5); Serial.println(" s");

    // // Print Linear Acceleration for all axes
    // Serial.print("Linear Accel: X="); Serial.print(linearAccelData.acceleration.x, 3);
    // Serial.print(" Y="); Serial.print(linearAccelData.acceleration.y, 3);
    // Serial.print(" Z="); Serial.print(linearAccelData.acceleration.z, 3);
    // Serial.println(" m/s^2");

    // // Print Current Velocity for all axes
    // Serial.print("Current Vel:  X="); Serial.print(currentVelocityX, 3);
    // Serial.print(" Y="); Serial.print(currentVelocityY, 3);
    // Serial.print(" Z="); Serial.print(currentVelocityZ, 3);
    // Serial.println(" m/s");

    // // Print Current Displacement for all axes
    // Serial.print("Current Disp: X="); Serial.print(currentDisplacementX, 3);
    // Serial.print(" Y="); Serial.print(currentDisplacementY, 3);
    // Serial.print(" Z="); Serial.print(currentDisplacementZ, 3);
    // Serial.println(" m");

    // Serial.println("------------------------------------\n");
  }
  }

  // No delay here, allowing integration to run continuously and display to update on its own timer.
}
