#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoBLE.h>
#include <SPI.h>
// Define the I2C address (update based on your SDO pin connection)
#define BNO055_I2C_ADDRESS 0x28  // Use 0x29 if SDO is connected to VCC

// Create an instance of the BNO055 sensor
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, BNO055_I2C_ADDRESS);

// Create a custom BLE service and characteristic UUIDs
BLEService gyroService("12345678-1234-1234-1234-1234567890ab");  // Custom service UUID
BLECharacteristic gyroCharacteristic("87654321-4321-4321-4321-ba0987654321", BLERead | BLENotify, 20);  // 20-byte max length

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor to open

  // Initialize I2C with custom pins for ESP32: SDA=21, SCL=22
  Wire.begin(21, 22);
  delay(100); // Allow some time for I2C to settle

  Serial.println("Initializing BNO055...");

  // Initialize the sensor
  if (!myIMU.begin()) {
    Serial.println("Error: BNO055 not detected. Check wiring or I2C address!");
    while (1); // Halt if sensor is not detected
  }

  // Allow the sensor to stabilize
  delay(1000);

  // Enable the external crystal for better precision
  myIMU.setExtCrystalUse(true);

  Serial.println("BNO055 initialized successfully. Starting BLE...");

  // Begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set the local name and advertise the service
  BLE.setLocalName("GyroSensorx");
  BLE.setDeviceName("GyroSensor");
  BLE.setAdvertisedService(gyroService);

  // Add the characteristic to the service
  gyroService.addCharacteristic(gyroCharacteristic);

  // Add the service
  BLE.addService(gyroService);

  // Initialize the characteristic with a default value
  gyroCharacteristic.writeValue("0,0,0");

  // Start advertising the BLE service
  BLE.advertise();
  Serial.println("BLE device is now advertising, waiting for connections...");
}

void loop() {
  // Listen for BLE central connections
  BLEDevice central = BLE.central();

  // If a central is connected
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // While the central is connected, send gyroscope data
    while (central.connected()) {
      // Get gyroscope data
      imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      // Format the data as CSV (X,Y,Z)
      char dataString[20];
      snprintf(dataString, sizeof(dataString), "%.2f,%.2f,%.2f", gyro.x(), gyro.y(), gyro.z());

      // Write the new value to the BLE characteristic (automatically notifies connected devices)
      gyroCharacteristic.writeValue(dataString);

      // Print the data to Serial for debugging
      Serial.println(dataString);

      // Delay for a consistent update rate
      delay(50);
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
