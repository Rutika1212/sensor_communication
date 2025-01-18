#include "Wire.h"
#include <MPU6050_light.h>
#include <ESP8266WiFi.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

const char* ssid = "Multifi-Kshirsagar";
const char* password = "Oldisgold";
WiFiServer server(2000);

int interval = 1000; // Default interval
WiFiClient client;

String formatHex(int16_t value) {
  // Ensure 4-character hexadecimal representation
  String hexString = String((uint16_t)value, HEX);
  while (hexString.length() < 4) {  // Pad with leading zeros to make it 4 characters
    hexString = "0" + hexString;
  }
  return hexString;
}

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  server.begin(); // Start server

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    delay(1000);
    Serial.println(F("Retrying MPU6050 initialization..."));
  }
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // Gyro and accelerometer offsets
  Serial.println("Offsets calculated. MPU6050 ready!");
}

void loop() {
 
  mpu.update();

  if ((millis() - timer) > 10) { // print data every 10ms
    Serial.print("ROLL : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tPITCH : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tYAW : ");
    Serial.println(mpu.getAngleZ());
    timer = millis();  
  }
  Serial.printf("client connection");
  client = server.available();  // Wait for client connection

  Serial.println("Client connected, waiting for data...");
  if (client) {
    String command = "";
   unsigned long commandTimeout = millis();
//    while (client.connected() && millis() - commandTimeout < 5000){
while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        command += c;
        if (command.endsWith("\r\n")) {
          command.trim();  // Remove leading and trailing whitespace
          processCommand(command);  // Process the command from client
          command = ""; // Reset command buffer after processing

        }
  
      }
      
    }
 
  }

}

void processCommand(String command) {
  if (command.startsWith("#03")) { // Start Command
    interval = strtol(command.substring(4, 8).c_str(), NULL, 16); // Parse interval
    Serial.print("Interval set to: ");
    Serial.println(interval);
    sendStatus(); // Send status after setting interval
  } else if (command.startsWith("#09")) { // Stop Command
    Serial.println("Stop Command received.");
    client.stop(); // Close client connection
  }
}

void sendStatus() {
  mpu.update();

  // Calculate Roll, Pitch, Yaw
  int16_t roll = mpu.getAngleX() * 10;  // Converting to deci-degrees
  int16_t pitch = mpu.getAngleY() * 10;
  int16_t yaw = mpu.getAngleZ() * 10;

  // Example raw values
  uint16_t supply_voltage = 3300; // Mock value in millivolts
  int16_t env_temp = 250;         // Mock value in deci-Celsius

  // Serial print for debugging
  Serial.println("Status Data:");
  Serial.printf("SUPPLY_VOLTAGE: %d (%s)\n", supply_voltage, formatHex(supply_voltage).c_str());
  Serial.printf("ENV_TEMP: %d (%s)\n", env_temp, formatHex(env_temp).c_str());
  Serial.printf("YAW: %d (%s)\n", yaw, formatHex(yaw).c_str());
  Serial.printf("PITCH: %d (%s)\n", pitch, formatHex(pitch).c_str());
  Serial.printf("ROLL: %d (%s)\n", roll, formatHex(roll).c_str());

  // Prepare response
  String response = "$11";
  response += formatHex(supply_voltage); // SUPPLY_VOLTAGE
  response += formatHex(env_temp);       // ENV_TEMP
  response += formatHex(yaw);            // YAW
  response += formatHex(pitch);          // PITCH
  response += formatHex(roll);           // ROLL
  response += "\r\n";


  // Send response to client
  if (client.connected()) {
    client.print(response);
  } else {
    Serial.println("Client disconnected before sending data.");
  }
  

}
