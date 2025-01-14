#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "wifiname";
const char* password = "wifipassword";

// MQTT Broker settings
const char* mqttServer = "youripaddress";  // Your laptop's local IP address or public broker
const int mqttPort = 1883;               // Default MQTT port
const char* mqttTopic = "imu_data";      // MQTT topic to send IMU data to

WiFiClient espClient;
PubSubClient client(espClient);

// BMX055 I2C Addresses
#define BMX055_ACCEL_ADDR 0x18
#define BMX055_GYRO_ADDR  0x68
#define BMX055_MAG_ADDR   0x10

// Accelerometer Registers
#define ACCEL_CHIPID_REG      0x00
#define ACCEL_X_LSB_REG       0x02
#define ACCEL_X_MSB_REG       0x03
#define ACCEL_Y_LSB_REG       0x04
#define ACCEL_Y_MSB_REG       0x05
#define ACCEL_Z_LSB_REG       0x06
#define ACCEL_Z_MSB_REG       0x07
#define ACCEL_RANGE_REG       0x0F

// Gyroscope Registers
#define GYRO_X_LSB_REG 0x02
#define GYRO_X_MSB_REG 0x03
#define GYRO_Y_LSB_REG 0x04
#define GYRO_Y_MSB_REG 0x05
#define GYRO_Z_LSB_REG 0x06
#define GYRO_Z_MSB_REG 0x07

// Magnetometer Registers
#define MAG_CHIPID_REG      0x32
#define MAG_X_LSB_REG       0x42
#define MAG_X_MSB_REG       0x43
#define MAG_Y_LSB_REG       0x44
#define MAG_Y_MSB_REG       0x45
#define MAG_Z_LSB_REG       0x46
#define MAG_Z_MSB_REG       0x47
#define MAG_POWER_REG       0x4B
#define MAG_CONTROL_REG     0x4C

void setup() {
  Serial.begin(9600);
  Wire.begin(9, 8); // Initialize I2C
  delay(100);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  // Initialize sensors
  initAccelerometer();
  initMagnetometer();
  initGyroscope();
}

void loop() {
  // Read accelerometer data
  int16_t accelX = readRegisterInt16(BMX055_ACCEL_ADDR, ACCEL_X_LSB_REG, ACCEL_X_MSB_REG);
  int16_t accelY = readRegisterInt16(BMX055_ACCEL_ADDR, ACCEL_Y_LSB_REG, ACCEL_Y_MSB_REG);
  int16_t accelZ = readRegisterInt16(BMX055_ACCEL_ADDR, ACCEL_Z_LSB_REG, ACCEL_Z_MSB_REG);
  
  // Read gyroscope data
  int16_t gyroX = readRegisterInt16(BMX055_GYRO_ADDR, GYRO_X_LSB_REG, GYRO_X_MSB_REG);
  int16_t gyroY = readRegisterInt16(BMX055_GYRO_ADDR, GYRO_Y_LSB_REG, GYRO_Y_MSB_REG);
  int16_t gyroZ = readRegisterInt16(BMX055_GYRO_ADDR, GYRO_Z_LSB_REG, GYRO_Z_MSB_REG);
  
  // Read magnetometer data
  int16_t magX = readRegisterInt16(BMX055_MAG_ADDR, MAG_X_LSB_REG, MAG_X_MSB_REG);
  int16_t magY = readRegisterInt16(BMX055_MAG_ADDR, MAG_Y_LSB_REG, MAG_Y_MSB_REG);
  int16_t magZ = readRegisterInt16(BMX055_MAG_ADDR, MAG_Z_LSB_REG, MAG_Z_MSB_REG);

  // Convert data to correct units
  float ax = (accelX / 1024.0f);
  float ay = (accelY / 1024.0f);
  float az = (accelZ / 1024.0f);
  float gx = (gyroX / 262.4f);
  float gy = (gyroY / 262.4f);
  float gz = (gyroZ / 262.4f);

  // Print data to Serial Monitor
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(", ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); Serial.print(", ");
  Serial.print(magX); Serial.print(", ");
  Serial.print(magY); Serial.print(", ");
  Serial.println(magZ);

  // Format the data as a JSON string to send via MQTT
  String imuData = String(ax, 3) + ", " + String(ay, 3) + ", " + String(az, 3) +
                   ", " + String(gx, 3) + ", " + String(gy, 3) + ", " + String(gz, 3) +
                   ", " + String(magX) + ", " + String(magY) + ", " + String(magZ);

  // Publish the IMU data to the MQTT topic
  client.publish(mqttTopic, imuData.c_str());
  Serial.println("Successful");
  // Wait for a short period before the next reading
  delay(50);
}

void initAccelerometer() {
  writeRegister(BMX055_ACCEL_ADDR, 0x14, 0x10);
  delay(10);
  writeRegister(BMX055_ACCEL_ADDR, ACCEL_RANGE_REG, 0x03);
}

void initMagnetometer() {
  writeRegister(BMX055_MAG_ADDR, MAG_POWER_REG, 0x01);
  delay(100);
  writeRegister(BMX055_MAG_ADDR, MAG_CONTROL_REG, 0x00);
  delay(10);
}

void initGyroscope() {
  writeRegister(BMX055_GYRO_ADDR, 0x14, 0xB6);
  delay(100);
  writeRegister(BMX055_GYRO_ADDR, 0x11, 0x00);
  delay(100);
  writeRegister(BMX055_GYRO_ADDR, 0x0F, 0x04);
  writeRegister(BMX055_GYRO_ADDR, 0x10, 0x07);
}

void writeRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.write(data);
  Wire.endTransmission();
}

int16_t readRegisterInt16(uint8_t deviceAddr, uint8_t lsbReg, uint8_t msbReg) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(lsbReg);
  Wire.endTransmission();
  Wire.requestFrom((int)deviceAddr, (int)2);
  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  return (msb << 8) | lsb;
}
