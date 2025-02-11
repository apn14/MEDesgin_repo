#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define the I2C address (update based on your SDO pin connection)
#define BNO055_I2C_ADDRESS 0x28 // Use 0x29 if SDO is connected to VCC

// Create an instance of the BNO055 sensor
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, BNO055_I2C_ADDRESS);

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Initializing BNO055...");

  // Initialize the sensor
  if (!myIMU.begin()) {
    Serial.println("Error: BNO055 not detected. Check wiring or I2C address!");
    while (1); // Halt the program if the sensor is not detected
  }

  // Allow the sensor to stabilize
  delay(1000);

  // Enable the external crystal for better precision
  myIMU.setExtCrystalUse(true);

  Serial.println("BNO055 initialized successfully. Reading gyroscope data...");
}

void loop() {
  // Get gyroscope data
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Print gyroscope data in CSV format (X,Y,Z)
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.println(gyro.z());

  // Delay for a consistent output rate
  delay(50); // Adjust as needed for smoother visualization
}
