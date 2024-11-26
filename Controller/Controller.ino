/*
  Joystick controller
  This program lets an ESP32 to act as a joystick controller, sending data values
  (from the two joysticks) to a receiver ESP32 using the ESP-NOW. 

  This code will:
  - Reads analog value from two joysticks (right and left).
  - Adjusts for initial offsets values of the joystick positions.
  - Sends data only when enough movement is detected to reduce unnecessary transmission.

  Author: Mark Smart
*/

#include <esp_now.h>
#include <WiFi.h> // Including the libraries for ESP-NOW and WiFi

#define VRX_PIN_RIGHT  33 // Right joystick X axis pin
#define VRY_PIN_RIGHT  34 // Right joystick Y axis pin
#define VRX_PIN_LEFT   32 // Left joystick X axis pin

int valueXR = 0; // Current X axis value of the right joystick
int valueYR = 0; // Current Y axis value of the right joystick
int valueXL = 0; // Current X axis value of the left joystick

bool firstTime = true; // Flag to initialize joystick offsets only once

int originalXR; // Initial X axis offset for the right joystick
int originalYR; // Initial Y axis offset for the right joystick
int originalXL; // Initial X axis offset for the left joystick

uint8_t receiverAddress[] = {0x08, 0xD1, 0xF9, 0x3B, 0x27, 0x40}; // MAC address of the receiver ESP32

// Structure to hold joystick data for transmission
struct JoystickData { 
  int rightX;
  int rightY;
  int leftX;
};

// Function that is triggered after sending data
void onSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Prints whether the data was sent successfully or failed
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send Success" : "Send Fail");
}

void setup() {
  // Initialize communication
  Serial.begin(115200); 
  analogSetAttenuation(ADC_11db); // Set analog pin for a 3.3V range
  
  WiFi.mode(WIFI_STA); // Set WiFi mode to Station making sure both ESP32's are on the same protocal
  if (esp_now_init() != ESP_OK) { // Initialize ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    delay(1000);
    return;
  }
  
  esp_now_register_send_cb(onSent); // Register the functon name to call after a send
  esp_now_peer_info_t peerInfo; // ESP-NOW peer configuration
  memcpy(peerInfo.peer_addr, receiverAddress, 6); // Set the receiver MAC address
  peerInfo.channel = 0; // Use default WiFi channel
  peerInfo.encrypt = false; // Disable encryption no need in this case 
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { // Add the receiver as a peer
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  valueXR = analogRead(VRX_PIN_RIGHT); // Get the X axis value from the right joystick
  valueYR = analogRead(VRY_PIN_RIGHT); // Get the Y axis value from the right joystick
  valueXL = analogRead(VRX_PIN_LEFT);  // Get the X axis value from the left joystick

  if (firstTime) { // Only execute during the first loop iteration
    originalXR = valueXR; // Record initial X axis value of the right joystick
    originalYR = valueYR; // Record initial Y axis value of the right joystick
    originalXL = valueXL; // Record initial X axis value of the left joystick
    firstTime = false; // Ensure this code only runs once
  }

  // Check if any joystick value has moved significantly from its initial position
  if (abs(valueXR - originalXR) > 100 || abs(valueYR - originalYR) > 100 || abs(valueXL - originalXL) > 100) {
    JoystickData dataToSend; // Create an instance of JoystickData
    dataToSend.rightX = valueXR; // Assign the current X axis value of the right joystick
    dataToSend.rightY = valueYR; // Assign the current Y axis value of the right joystick
    dataToSend.leftX = valueXL;  // Assign the current X axis value of the left joystick

    // Send the data structure to the receiver
    esp_now_send(receiverAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
  }

  delay(50); // Add a small delay to avoid spamming transmissions
}
