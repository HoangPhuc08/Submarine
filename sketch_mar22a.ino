// #include <SPI.h>
// #include <LoRa.h>

// // --- PIN DEFINITIONS FOR ESP32-S3 ---
// // Make sure these match your wiring!

// // LoRa Module SPI pins
// #define LORA_SCK  12  // GPIO12 - SCK
// #define LORA_MISO 13  // GPIO13 - MISO
// #define LORA_MOSI 11  // GPIO11 - MOSI
// #define LORA_CS   10  // GPIO10 - Chip Select (NSS)

// // LoRa Module Control pins
// #define LORA_RST  9   // GPIO9  - Reset
// #define LORA_IRQ  2   // GPIO2  - IRQ/Interrupt (DIO0)

// // Onboard LED for status indication
// #define LED_PIN   25  // GPIO25 - The pin your LED is connected to

// // --- LORA PARAMETERS ---
// // IMPORTANT: Make sure this frequency matches your transmitter!
// #define LORA_FREQUENCY 433E6

// void setup() {
//   // Initialize Serial Monitor for debugging
//   Serial.begin(115200);
//   while (!Serial); // Wait for serial port to connect. Needed for native USB on S3
//   Serial.println("ESP32-S3 LoRa Receiver");

//   // Set up the LED pin
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW); // Start with LED off

//   // --- Initialize LoRa Module ---
  
//   // 1. Set the SPI bus to use our custom pins
//   SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

//   // 2. Set the LoRa module control pins
//   LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

//   // 3. Initialize LoRa module at the specified frequency
//   if (!LoRa.begin(LORA_FREQUENCY)) {
//     Serial.println("Starting LoRa failed! Check your wiring.");
//     while (1); // If it fails, halt the program
//   }

//   Serial.println("LoRa Initialized Successfully!");
// }

// void loop() {
//   // Try to parse a packet
//   int packetSize = LoRa.parsePacket();

//   if (packetSize) {
//     // --- A packet was received! ---
//     Serial.print("Received packet: '");

//     // Read the packet into a string
//     String receivedMessage = "";
//     while (LoRa.available()) {
//       receivedMessage += (char)LoRa.read();
//     }
    
//     // Print the message content
//     Serial.print(receivedMessage);
//     Serial.print("'");

//     // Print the RSSI (Received Signal Strength Indicator)
//     Serial.print(" with RSSI ");
//     Serial.println(LoRa.packetRssi());
    
    
//   } else {
//     // --- No packet was received, print waiting message ---
//     Serial.println("Waiting for messages...");
//     delay(1000); // Wait 1 second to avoid spamming the serial monitor
//   }
// }








// #include <SPI.h>
// #include <LoRa.h>

// // --- PIN DEFINITIONS FOR ESP32-S3 AUTOMATIC TRANSMITTER ---

// // LoRa Module SPI and Control pins (Identical to receiver)
// #define LORA_SCK  12
// #define LORA_MISO 13
// #define LORA_MOSI 11
// #define LORA_CS   10
// #define LORA_RST  9
// #define LORA_IRQ  2

// // Output pin for status LED
// #define LED_PIN    25 // GPIO for the "message sent" indicator LED

// // --- LORA PARAMETERS ---
// // IMPORTANT: Make sure this frequency matches your receiver!
// #define LORA_FREQUENCY 433E6

// // Global variable to count sent messages
// int counter = 0;

// void setup() {
//   // Initialize Serial Monitor for debugging
//   Serial.begin(115200);
//   while (!Serial); // Wait for serial port to connect
//   Serial.println("ESP32-S3 LoRa Automatic Transmitter");

//   // Set up the LED pin
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW); // Start with LED off

//   // --- Initialize LoRa Module ---
//   SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
//   LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

//   if (!LoRa.begin(LORA_FREQUENCY)) {
//     Serial.println("Starting LoRa failed! Check your wiring.");
//     while (1); // Halt if it fails
//   }
//   Serial.println("LoRa Initialized Successfully. Starting automatic transmission...");
// }

// void loop() {
//   // 1. Increment the message counter
//   counter++;

//   // 2. Create the message string
//   String message = "Hello World " + String(counter);
  
//   // 3. Print to the local serial monitor for feedback
//   Serial.print("Sending packet: ");
//   Serial.println(message);

//   // 4. Send the packet via LoRa
//   LoRa.beginPacket();
//   LoRa.print(message);
//   LoRa.endPacket();




//   // 6. Wait for 3 seconds before sending the next packet
//   delay(3000); 
// }


// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h>
// #include <Adafruit_Sensor.h>



// // See Calibration Note!
// const float OffSet = 0.276; // See Calibration Note!
// float V1, P1, raw_val1;         // Voltage & Pressure
// float V2, P2, raw_val2;         // Voltage & Pressure
// float rho, g, H1, H2;
// // OTA
// //Flat && send data
// unsigned long startMillis;
// unsigned long currentMillis;
// const unsigned long period = 10000;  // the value is a number of milliseconds
// unsigned long lastSendTime = 0;
// int counter = 0;
// volatile bool doRead = false; // Flag set by callback to perform read process in main loop
// volatile int incomingPacketSize;
// void setup() {
//   // Start the serial communication
//   Serial.begin(115200);
//  // Initialize I2C for BNO055
//   // Pressure Sensor
//   pinMode(2, INPUT);
//   Serial.println("Water Pressure");
//   rho = 998;
//   g = 9.81;
//   // Set analog pin for pressure sensor
//   analogReadResolution(12); // ESP32-S3 ADC resolution is 12 bits
// }
  
// void loop() {
//   //Data
//   // Pressure sensor
//   // raw_val1 = analogRead(2);
//   // V1 = raw_val1 * (3.3 / 4095); // ESP32-S3 uses 3.3V for ADC reference and has 12-bit ADC
//   // P1 = (V1 - OffSet) * 6.89 * 2.5; // Convert to pressure in KPa
//   // H1 = P1 * 1000 / (rho * g);
//   // // Connect sensor’s output (SIGNAL) to Analog 0 (GPIO 36 on ESP32-S3) (sensor 2)
//   // unsigned long tStart = micros();

//   Serial.println(analogRead(2));
  
//   // Delay before sending next data
//   delay(100); // You may adjust this delay based on your requirements

// }

// UNIVERSAL LORA TRANSCEIVER CODE FOR ESP32-S3
// Upload this exact same code to BOTH of your devices.
// The only change you need to make is the BOARD_ID on the second device.










// #include <SPI.h>
// #include <LoRa.h>
// #include "FS.h" 
// #include "SD.h"

// // =========================================================================
// // ===================  CRITICAL CONFIGURATION  ============================
// //
// //
// #define BOARD_ID "SUBMARINE" 
// //
// // =========================================================================
// // =========================================================================


// // --- PIN DEFINITIONS (LoRa) ---
// #define LORA_SCK  13  // GPIO13 - SCK
// #define LORA_MISO 12  // GPIO12 - MISO
// #define LORA_MOSI 11  // GPIO11 - MOSI
// #define LORA_CS   10  // GPIO10 - Chip Select (NSS)
// #define LORA_RST  14  // GPIO14 - Reset
// #define LORA_IRQ  3   // GPIO3  - IRQ/Interrupt (DIO0)
// #define LED_PIN   25  // GPIO25 - Status LED

// // -- PIN DEFINITIONS (SD Card) --
// #define SD_CS 38 // SPI_CS

// // -- Personalization of communication
// #define LORA_FREQUENCY 433.2E6
// #define SPREADING_FACTOR 11
// #define CODING_RATE 5
// #define SYNC_WORD 0x36


// // --- NON-BLOCKING TIMER VARIABLES ---
// unsigned long lastSendTime = 0;        // Timestamp of the last transmission
// const unsigned long sendInterval = 5000; // Interval between transmissions
// int counter = 0;                         // Message counter

// // Your original Chip Select functions are kept exactly as they were.
// void selectLoRa() {
//   digitalWrite(SD_CS, HIGH);
//   digitalWrite(LORA_CS, LOW);
// }

// void selectSDCard() {
//   digitalWrite(LORA_CS, HIGH);
//   digitalWrite(SD_CS, LOW);
// }

// void freeCS() {
//   digitalWrite(LORA_CS, HIGH);
//   digitalWrite(SD_CS, HIGH);
// }

// // Your original appendFile function is kept exactly as it was.
// void appendFile(fs::FS &fs, const char * path, const char * message){
//     File file = fs.open(path, FILE_APPEND);
//     if(!file){
//         Serial.println("Failed to open file for appending");
//         return;
//     }
//     file.print(message);
//     file.close();
// }

// void setup() {
//   // Your original setup code is kept.
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("================================");
//   Serial.print("ESP32-S3 LoRa Transceiver: ");
//   Serial.println(BOARD_ID);
//   Serial.println("================================");

//   // ADDED: Initialize the Chip Select pins as outputs. This is required for them to work.
//   pinMode(LORA_CS, OUTPUT);
//   pinMode(SD_CS, OUTPUT);
//   freeCS(); // Start with both devices inactive.

//   // Your original LED pin setup is kept.
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   // Your original LoRa Module Initialization is kept.
//   // The SPI.begin() here initializes the bus for all devices.
//   SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

//   // --- Initialize LoRa Module ---
//   selectLoRa(); // ADDED: Select LoRa before initializing it.
//   LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
//   // Your personalization settings are kept exactly as they were.
//   LoRa.setSpreadingFactor(SPREADING_FACTOR);
//   LoRa.setSyncWord(SYNC_WORD);
//   LoRa.setCodingRate4(CODING_RATE);

//   if (!LoRa.begin(LORA_FREQUENCY)) {
//     Serial.println("Starting LoRa failed! Check wiring.");
//     while (1); // Halt if it fails
//   }
//   Serial.println("LoRa Initialized Successfully.");
//   freeCS(); // ADDED: Free the bus after LoRa init.

//   // ADDED: The SD card initialization test logic.
//   Serial.println("Initializing SD card for test...");
//   selectSDCard(); // Select the SD card.
//   if(!SD.begin(SD_CS)){
//     Serial.println("SD Card Mount Failed!");
//   } else {
//     Serial.println("SD Card Initialized Successfully.");
//     appendFile(SD, "/test_log.txt", "------ System Boot & Test Start ------\r\n");
//   }
//   freeCS(); // Free the bus after SD card init.

//   Serial.println("Setup complete. Starting main loop...");
// }

// void loop() {
//   // --- PART 1: ALWAYS LISTEN FOR MESSAGES (Your original code) ---
//   selectLoRa(); // ADDED: Select LoRa before checking for packets.
//   int packetSize = LoRa.parsePacket();
//   if (packetSize) {
//     // A packet was received!
//     String receivedMessage = "";
//     while (LoRa.available()) {
//       receivedMessage += (char)LoRa.read();
//     }
    
//     // Print the received message and signal strength
//     Serial.print("Received: '");
//     Serial.print(receivedMessage);
//     Serial.print("' with RSSI ");
//     Serial.println(LoRa.packetRssi());
//   }
//   freeCS(); // ADDED: Free the bus after checking.

//   // --- PART 2: SEND A MESSAGE PERIODICALLY (Your original code) ---
//   if (millis() - lastSendTime > sendInterval) {
//     // It's time to send a new message
//     lastSendTime = millis();
//     counter++;
//     String messageToSend = "Hello from " + String(BOARD_ID) + " - Packet " + String(counter);
//     Serial.print("Sending: '");
//     Serial.print(messageToSend);
//     Serial.println("'");
    
//     // --- LoRa Send Block ---
//     selectLoRa(); // ADDED: Select LoRa before sending.
//     LoRa.beginPacket();
//     LoRa.print(messageToSend);
//     LoRa.endPacket();
//     freeCS(); // ADDED: Free the bus after sending.

//     // ADDED: The SD card logging test feature.
//     // This happens immediately after sending a LoRa packet.
//     String logMessage = "Logged send of packet #" + String(counter) + "\r\n";
//     Serial.print("  -> TEST: Logging to SD card... ");
//     selectSDCard(); // Select the SD card.
//     appendFile(SD, "/test_log.txt", logMessage.c_str());
//     freeCS(); // Free the bus after logging.
//     Serial.println("Done.");
//   }
// }










// #include <Wire.h>
// #define SCL_PIN 41
// #define SDA_PIN 42

// void setup() {
//   // Start Serial communication
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("\n--- I2C Scanner (Single Scan) ---");

//   // Start the I2C bus. For ESP32, you can specify the SDA and SCL pins if needed.
//   // Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.begin(SDA_PIN, SCL_PIN); 

//   // --- The Scan Logic ---
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");
//   nDevices = 0;

//   // The I2C address space is 7-bit, from 0 to 127.
//   for(address = 1; address < 127; address++ ) {
//     // Wire.beginTransmission() sends a start condition and the address.
//     Wire.beginTransmission(address);
//     // The return value of endTransmission indicates the status.
//     // 0: success (a device acknowledged at this address)
//     // 4: other error
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) {
//         Serial.print("0");
//       }
//       Serial.println(address, HEX); // Print the address in hexadecimal format
//       nDevices++;
//     }
//     else if (error == 4) {
//       // This part is less common but good for debugging strange issues.
//       // It means a device is holding the bus in a strange state at this address.
//       Serial.print("Unknown error at address 0x");
//       if (address < 16) {
//         Serial.print("0");
//       }
//       Serial.println(address, HEX);
//     }    
//     delay(500);
//   }

//   // --- Final Report ---
//   Serial.println("---------------------------------");
//   if (nDevices == 0) {
//     Serial.println("Result: No I2C devices found.");
//     Serial.println("Please check your wiring, power, and pull-up resistors.");
//   }
//   else {
//     Serial.print("Result: Found ");
//     Serial.print(nDevices);
//     Serial.println(" device(s).");
//   }
//   Serial.println("Scan complete. The program will now idle.");
//   Serial.println("---------------------------------");
// }

// void loop() {
//   // The loop is intentionally left empty.
//   // The scan will only run once.
// }









// I2C DEVICE SWITCHING TEST - BNO055 IMU and LCD
// This code reads data from a BNO055 IMU and displays it on an I2C LCD screen,
// demonstrating how to address two different devices on the same bus.

// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>

// // These includes are from your BNO055 code
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// =========================================================================
// ===================  DEVICE CONFIGURATION  ==============================
//
//  Your specific I2C addresses that you found with the scanner.
//
// #define LCD_ADDRESS  0x3C
// #define IMU_ADDRESS  0x29

// #define SCL_PIN 41
// #define SDA_PIN 42

// // =========================================================================

// // Create objects for our I2C devices
// // The LCD constructor needs the address, columns, and rows.
// LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2); 

// // The BNO055 object, using the constructor that works with the default Wire library.
// // The -1 parameter indicates we are using I2C.
// Adafruit_BNO055 bno = Adafruit_BNO055(-1, IMU_ADDRESS);

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   // Start the I2C bus. The libraries will also call this, but it's good practice.
//   Wire.begin(SDA_PIN, SCL_PIN); 

//   // --- Initialize Device 1: The LCD ---
//   // This part is kept from your original template.
//   Serial.println("Initializing LCD...");
//   lcd.init();      // Initialize the LCD
//   lcd.backlight(); // Turn on the backlight
//   lcd.setCursor(0, 0);
//   lcd.print("BNO055 IMU Test");
//   delay(1000);

//   // --- Initialize Device 2: The BNO055 IMU ---
//   // This section is replaced with BNO055 logic.
//   Serial.println("Initializing BNO055 IMU...");
//   if (!bno.begin()) {
//     Serial.println("Failed to find BNO055 chip. Check wiring or I2C address.");
//     lcd.setCursor(0, 1);
//     lcd.print("IMU FAIL!");
//     while (1); // Halt the program if the IMU isn't found
//   }
//   bno.setExtCrystalUse(true); // Recommended for stability
//   Serial.println("BNO055 found!");
  
//   // The calibration wait loop from your IMU code is very important for accuracy.
//   Serial.println("Waiting for BNO055 calibration...");
//   lcd.clear();
//   lcd.print("Calibrating IMU");
//   uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
//   system_cal = gyro_cal = accel_cal = mag_cal = 0;
//   while (system_cal < 1) { // Wait for at least partial calibration
//     bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
//     lcd.setCursor(0, 1);
//     lcd.print("Sys:");
//     lcd.print(system_cal);
//     lcd.print(" G:");
//     lcd.print(gyro_cal);
//     lcd.print(" A:");
//     lcd.print(accel_cal);
//     lcd.print(" M:");
//     lcd.print(mag_cal);
//     delay(100);
//   }
  
//   Serial.println("IMU Calibrated!");
//   lcd.clear();
// }

// void loop() {
//   // --- ACTION A: Read data from the IMU ---
//   // The bno library handles addressing the IMU_ADDRESS (0x29) automatically.
//   sensors_event_t orientationData;
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

//   float heading = orientationData.orientation.x;
//   float roll = orientationData.orientation.y;
//   float pitch = orientationData.orientation.z;


//   // Print the IMU data to the Serial monitor for debugging
//   Serial.print("Heading: "); Serial.print(heading);
//   Serial.print("  Roll: "); Serial.print(roll);
//   Serial.print("  Pitch: "); Serial.println(pitch);


//   // --- ACTION B: Switch to the LCD to display the data ---
//   // The lcd library handles addressing the LCD_ADDRESS (0x3C) automatically.
//   lcd.setCursor(0, 0); // Move to the first line
//   lcd.print("Head(X): ");
//   lcd.print(heading, 1); // Print with 1 decimal place
//   lcd.print("   ");    // Add padding to clear old text

//   lcd.setCursor(0, 1); // Move to the second line
//   lcd.print("Roll(Y): ");
//   lcd.print(roll, 1);    // Print with 1 decimal place
//   lcd.print("   ");    // Add padding to clear old text
  
//   delay(250); // Update the screen 4 times a second
// }






// =========================================================================
// ==              STANDALONE U8G2 OLED TEST SKETCH                       ==
// =========================================================================
// This code tests an I2C OLED display on custom pins and includes a
// dynamic counter in the main loop to confirm continuous operation.

// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <Wire.h>

// // =========================================================================
// // ===================  YOUR HARDWARE CONFIGURATION  =======================
// //
// //  Define the custom I2C pins your OLED screen is wired to.
// #define OLED_SCL_PIN 41
// #define OLED_SDA_PIN 42
// //
// // =========================================================================

// // --- U8g2 CONSTRUCTOR ---
// // Using the Software I2C constructor for custom pins.
// // The arguments are: (rotation, scl, sda, reset)
// // U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0,  /* reset=*/ U8X8_PIN_NONE, /* scl=*/ OLED_SCL_PIN, /* sda=*/ OLED_SDA_PIN);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);


// unsigned long previousMillis = 0;
// const long interval = 1000;
// unsigned int seconds_counter = 0;

// void setup(void) {
//   Serial.begin(115200); 
//   while(!Serial);
//   Serial.println("--- Standalone U8g2 OLED Test (Hardware I2C) ---");

//   // --- Initialize the I2C bus ---
//   // We don't need to specify pins here because we are using the defaults.
//   Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

//   // --- Initialize the U8g2 Library ---
//   Serial.println("Attempting to initialize OLED display...");
//   u8g2.setI2CAddress(0x3C);
//   u8g2.begin();
//   u8g2.setContrast(255);
//   Serial.println("OLED Initialized.");
  
//   // Prime the display with the initial state
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_ncenB08_tr);
//   u8g2.drawStr(0, 12, "Elapsed Seconds:");
//   u8g2.setFont(u8g2_font_logisoso24_tr);
//   u8g2.drawStr(50, 50, "0");
//   u8g2.sendBuffer();
// }

// void loop(void) {
//   // This non-blocking loop logic is correct.
//   // It will now execute without being interrupted by software I2C conflicts.
//   unsigned long currentMillis = millis();
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis += interval;
//     seconds_counter++;

//     u8g2.clearBuffer();
//     u8g2.setFont(u8g2_font_ncenB08_tr);
//     u8g2.drawStr(0, 12, "Elapsed Seconds:");
    
//     u8g2.setFont(u8g2_font_logisoso24_tr);
//     char counter_str[10];
//     sprintf(counter_str, "%d", seconds_counter);
//     u8g2.drawStr(50, 50, counter_str);
    
//     u8g2.sendBuffer();

//     Serial.print("Display updated with seconds: ");
//     Serial.println(seconds_counter);
//   }
// }
// =========================================================================
// ==           STANDALONE U8G2 OLED - PURE SOFTWARE I2C                  ==
// =========================================================================
// This version uses ONLY the U8g2 library's built-in Software I2C and
// removes all conflicting calls to the hardware "Wire" library.

// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <Wire.h>

// // Use the default Hardware I2C pins for your board.
// // This example uses the common defaults for ESP32 DevKits.
// // VERIFY THESE PINS ON YOUR BOARD'S PINOUT!
// #define I2C_SDA_PIN 42
// #define I2C_SCL_PIN 41

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

// unsigned long counter = 0;

// void setup() {
//   Serial.begin(115200);
//   u8g2.begin();
//   u8g2.setI2CAddress(0x3C * 2);
//   for (unsigned long i=0;i<1000;i++) {
//     u8g2.setFont(u8g2_font_helvB12_tr);
//     u8g2.setFontMode(1);
//     u8g2.clearBuffer();
//     u8g2.setCursor(0,16);
//     char counter_str[10];
//     sprintf(counter_str, "%d", i);
//     u8g2.drawStr(50, 50, counter_str);
//     u8g2.sendBuffer();
//     Serial.println("IoT Demo on serial " + (String)i);
//     delay(1000);
//   }
// }

// void loop() {

// }

// // =========================================================================
// // ==              STANDALONE U8G2 OLED TEST SKETCH                       ==
// // =========================================================================
// // This code tests an I2C OLED display on custom pins using the U8g2 library.
// // It has no other dependencies.

// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <Wire.h>

// // =========================================================================
// // ===================  YOUR HARDWARE CONFIGURATION  =======================
// //
// //  Define the custom I2C pins your OLED screen is wired to.
// #define OLED_SCL_PIN 41
// #define OLED_SDA_PIN 42
// //
// // =========================================================================


// // --- U8g2 CONSTRUCTOR ---
// // This is the CRITICAL part.
// // We use the "_SW_I2C" (Software I2C) constructor because we are defining
// // custom SCL and SDA pins. The "_HW_I2C" version only works on default hardware pins.
// //
// // The arguments for SW_I2C are: (rotation, scl, sda, reset)
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,  /* reset=*/ U8X8_PIN_NONE, /* scl=*/ OLED_SCL_PIN, /* sda=*/ OLED_SDA_PIN);


// void setup(void) {
//   // Start the serial monitor for debugging messages
//   Serial.begin(115200); 
//   while(!Serial);
//   Serial.println("--- Standalone U8g2 OLED Test ---");

//   // --- STEP 1: INITIALIZE THE U8G2 LIBRARY ---
//   // The .begin() command initializes the display using the software I2C on the pins we defined above.
//   Serial.println("Attempting to initialize OLED display...");
//   u8g2.begin();
//   u8g2.setContrast(255);
//   Serial.println("OLED Initialized. Displaying test message.");

//   // --- STEP 2: DRAW THE INITIAL MESSAGE ---
//   u8g2.clearBuffer();                  // Clear the internal memory
//   u8g2.setFont(u8g2_font_ncenB10_tr);  // Choose a nice font
//   u8g2.drawStr(0, 15, "U8g2 Test OK!"); // Write string to the buffer
//   u8g2.sendBuffer();                   // Send the buffer to the display
  
//   delay(2000); // Wait 2 seconds to show the initial message
// }

// void loop(void) {
//   // --- DYNAMIC COUNTER TEST ---
//   // This part proves the display is continuously working.
  
//   u8g2.clearBuffer(); // Clear the buffer for the new content
  
//   // Display a static title
//   u8g2.setFont(u8g2_font_ncenB08_tr);
//   u8g2.drawStr(0, 12, "System is running!");
  
//   // Display a counter that updates every second
//   u8g2.setFont(u8g2_font_logisoso24_tr); // Use a large font for the counter
//   char counter_str[10];
//   sprintf(counter_str, "%d", millis() / 1000); // Get seconds since boot
//   u8g2.drawStr(30, 50, counter_str);
  
//   u8g2.sendBuffer(); // Send the updated buffer to the display
  
//   delay(1000); // Wait one second
// }










// // =========================================================================
// // ==     COMBINED I2C TEST: BNO055 IMU and U8G2 OLED Display             ==
// // =========================================================================
// // This code reads orientation data from a BNO055 IMU and displays it on
// // an I2C OLED screen using the U8g2 library on custom pins.

// // Libraries for both devices
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <U8g2lib.h>
// #include <utility/imumaths.h>

// // =========================================================================
// // ===================  YOUR HARDWARE CONFIGURATION  =======================
// //
// // Define the I2C pins that BOTH devices are connected to.
// #define I2C_SDA_PIN 42
// #define I2C_SCL_PIN 41

// // Define the I2C addresses for each device.
// #define IMU_ADDRESS  0x29
// // Note: The U8g2 library does not require the display address in its constructor.
// //
// // =========================================================================


// // --- OBJECT CREATION ---

// // 1. U8g2 CONSTRUCTOR (from your working test code)
// // This uses the correct Software I2C constructor for your custom pins.
// // The arguments are: (rotation, scl, sda, reset)
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* scl=*/ I2C_SCL_PIN, /* sda=*/ I2C_SDA_PIN);

// // 2. BNO055 IMU OBJECT
// // This constructor tells the library to use the default "Wire" object.
// Adafruit_BNO055 bno = Adafruit_BNO055(-1, IMU_ADDRESS);


// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   // --- STEP 1: INITIALIZE THE I2C BUS ---
//   // This single command initializes the bus for BOTH devices on your custom pins.
//   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

//   // --- STEP 2: INITIALIZE THE OLED DISPLAY ---
//   Serial.println("Initializing U8g2 OLED Display...");
//   u8g2.setI2CAddress(0x3C);
//   u8g2.begin();
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_ncenB08_tr);
//   u8g2.drawStr(0, 32, "OLED OK!");
//   u8g2.sendBuffer();
//   delay(1000);

//   // --- STEP 3: INITIALIZE THE BNO055 IMU ---
//   Serial.println("Initializing BNO055 IMU...");
//   if (!bno.begin()) {
//     Serial.println("Failed to find BNO055 chip. Halting.");
//     u8g2.clearBuffer();
//     u8g2.drawStr(0, 32, "IMU FAIL!");
//     u8g2.sendBuffer();
//     while (1);
//   }
//   bno.setExtCrystalUse(true); // Recommended for stability
//   Serial.println("BNO055 found!");

//   // --- STEP 4: CALIBRATE THE IMU ---
//   // Display calibration status on both the Serial Monitor and the OLED.
//   Serial.println("Waiting for BNO055 calibration...");
//   u8g2.clearBuffer();
//   u8g2.drawStr(0, 15, "Calibrating IMU...");
//   u8g2.sendBuffer();
  
//   uint8_t system_cal;
//   do {
//     bno.getCalibration(&system_cal, nullptr, nullptr, nullptr);
//     char cal_str[16];
//     sprintf(cal_str, "Sys Cal: %d", system_cal);
//     u8g2.setCursor(0, 40);
//     u8g2.print(cal_str);
//     u8g2.sendBuffer(); // Update screen with calibration status
//     delay(100);
//   } while (system_cal < 1); // Wait for at least partial calibration

//   Serial.println("IMU Calibrated!");
//   u8g2.clearBuffer();
//   u8g2.sendBuffer();
// }

// void loop() {
//   // --- ACTION A: Read orientation data from the IMU ---
//   // The bno library handles addressing the IMU automatically.
//   sensors_event_t orientationData;
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

//   float heading = orientationData.orientation.x;
//   float roll = orientationData.orientation.y;
//   float pitch = orientationData.orientation.z;

//   // --- ACTION B: Switch to the OLED to display the data ---
//   // The u8g2 library handles addressing the display automatically.
//   char line1_buf[20];
//   char line2_buf[20];
  
//   // Format the floating point numbers into character strings
//   sprintf(line1_buf, "H:%.1f R:%.1f", heading, roll);
//   sprintf(line2_buf, "P:%.1f", pitch);

//   // Draw the strings to the display buffer
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_ncenB10_tr);
//   u8g2.drawStr(0, 15, line1_buf);
//   u8g2.drawStr(0, 45, line2_buf);
//   u8g2.sendBuffer(); // Send the completed buffer to the screen

//   // Also print to Serial for debugging
//   Serial.printf("H: %.2f, R: %.2f, P: %.2f\n", heading, roll, pitch);
  
//   delay(100); // Update roughly 10 times a second
// }

// #include <Arduino.h>
// #include <U8g2lib.h>
// // NOTE: Wire.h is NOT needed because we are using Software I2C.

// // =========================================================================
// // ===================  YOUR HARDWARE CONFIGURATION  =======================
// //
// //  The custom I2C pins your OLED screen is wired to.
// #define OLED_SCL_PIN 41
// #define OLED_SDA_PIN 42
// //
// // =========================================================================


// // --- U8g2 CONSTRUCTOR ---
// // This uses the exact Software I2C constructor order you specified.
// // It tells the library to manually control pins 41 and 42.
// // Arguments are: (rotation, reset, scl, sda)
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,  /* scl=*/ OLED_SCL_PIN, /* sda=*/ OLED_SDA_PIN,/* reset=*/ U8X8_PIN_NONE);


// void setup(void) {
//   // Start the serial monitor for debugging messages
//   Serial.begin(115200); 
//   while(!Serial);
//   Serial.println("--- Simple U8g2 Text-Changing Test ---");

//   // --- INITIALIZE THE U8G2 LIBRARY ---
//   // The .begin() command initializes the display using the software I2C on the pins defined above.
//   Serial.println("Attempting to initialize OLED display...");
//   u8g2.setI2CAddress(0x3C);
//   u8g2.begin();
//   Serial.println("OLED Initialized.");
// }

// void loop(void) {
//   // --- DISPLAY MESSAGE 1 ---
//   Serial.println("Displaying Message 1...");
//   u8g2.clearBuffer();                  // Clear the internal memory
//   u8g2.setFont(u8g2_font_ncenB10_tr);  // Choose a font
//   u8g2.drawStr(25, 35, "Message 1");   // Write string to the buffer (centered)
//   u8g2.sendBuffer();                   // Send the buffer to the display
//   delay(2000);                         // Wait for 2 seconds


//   // --- DISPLAY MESSAGE 2 ---
//   Serial.println("Displaying Message 2...");
//   u8g2.clearBuffer();                  // Clear the internal memory
//   u8g2.setFont(u8g2_font_helvB12_tr);  // Use a different font to be obvious
//   u8g2.drawStr(25, 35, "Message 2");   // Write the new string
//   u8g2.sendBuffer();                   // Send the new buffer to the display
//   delay(2000);                         // Wait for 2 seconds
// }

// =========================================================================
// ==     FINAL COMBINED CODE - RESPECTING BOTH ORIGINAL CONSTRUCTORS     ==
// =========================================================================
// This code uses the default global "Wire" object for both devices,
// keeping your working U8g2 constructor exactly as you provided it.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <U8g2lib.h>
#include <utility/imumaths.h>

// =========================================================================
// ===================  YOUR HARDWARE CONFIGURATION  =======================
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 41

#define IMU_ADDRESS  0x29
#define LCD_ADDRESS  0x3C
// =========================================================================

// --- OBJECT CREATION ---

// 1. U8G2 OLED OBJECT (Your original, working constructor)
// This constructor is kept EXACTLY as you provided it in your working LCD sketch.
// It will use the global "Wire" object.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);


// 2. BNO055 IMU OBJECT (MODIFIED)
// This is the ONLY change. We switch to the constructor that also uses
// the default global "Wire" object, matching the U8g2 object.
Adafruit_BNO055 bno = Adafruit_BNO055(-1, IMU_ADDRESS);


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
  u8g2.setFont(u8g2_font_ncenB08_tr);
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
  Serial.println("BNO055 found!");

  // --- STEP 4: CALIBRATE THE IMU ---
  Serial.println("Waiting for BNO055 calibration...");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 15, "Calibrating IMU...");
  u8g2.sendBuffer();
  
  uint8_t system_cal;
  do {
    bno.getCalibration(&system_cal, nullptr, nullptr, nullptr);
    char cal_str[16];
    sprintf(cal_str, "Sys Cal: %d", system_cal);
    
    u8g2.clearBuffer();
    u8g2.drawStr(0, 15, "Calibrating IMU...");
    u8g2.drawStr(0, 40, cal_str);
    u8g2.sendBuffer();
    delay(100);
  } while (system_cal < 3);

  Serial.println("IMU Calibrated!");
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void loop() {
  // This loop remains unchanged, as its logic is correct.
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float heading = orientationData.orientation.x;
  float roll = orientationData.orientation.y;
  float pitch = orientationData.orientation.z;

  char line1_buf[20];
  char line2_buf[20];
  
  sprintf(line1_buf, "H:%.1f R:%.1f", heading, roll);
  sprintf(line2_buf, "P:%.1f", pitch);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 15, line1_buf);
  u8g2.drawStr(0, 45, line2_buf);
  u8g2.sendBuffer();

  Serial.printf("H: %.2f, R: %.2f, P: %.2f\n", heading, roll, pitch);
  
  delay(100);
}