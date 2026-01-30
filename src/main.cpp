#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

#include <structures.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t carAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

TaskHandle_t SendCommandsTaskHandle = NULL;

const uint8_t X_AXIS_JOYSTICK_PIN = 36;
const uint8_t Y_AXIS_JOYSTICK_PIN = 39;
const uint8_t JOYSTICK_BUTTON_PIN = 34;

// Create a struct_message called myData
CommandMessage command;
TelemetryMessage telemetry;
esp_now_peer_info_t peerInfo;

// --- Smoothing Variables ---
int offsetX = 0;
int offsetY = 0;

float smoothThrottle = 0;
float smoothSteering = 0;
// filterAlpha: 0.1 = very smooth but slight lag, 0.9 = raw and twitchy.
// 0.2 is the "sweet spot" for 10ms control.
const float filterAlpha = 0.2;

void SendCommandsTask(void *parameter) {
  for (;;) {
    if (Serial.available() >= 7) {
      if (Serial.peek() == 0xAA) {
        Serial.read(); // Discard the header byte (0xAA)
        // Create a temporary buffer
        uint8_t buffer[sizeof(CommandMessage)];
  
        // Read the exact number of bytes needed
        Serial.readBytes(buffer, sizeof(CommandMessage));
  
        // Copy buffer into the command struct
        memcpy(&command, buffer, sizeof(CommandMessage));
      }
      else {
        // Not a header. Discard 1 byte and try again in the next loop
        Serial.read();
      }
    } else {
      // read joystick
      int rawX = constrain(analogRead(X_AXIS_JOYSTICK_PIN) - offsetX, 0, 4095);
      int rawY = constrain(analogRead(Y_AXIS_JOYSTICK_PIN) - offsetY, 0, 4095);

      // Map to -100 to 100
      int throttle = map(rawY, 0, 4095, -100, 100);
      int steering = map(rawX, 0, 4095, -100, 100);

      // Apply a Deadzone
      if (abs(throttle) < 7)
        throttle = 0;
      if (abs(steering) < 7)
        steering = 0;

      // EMA Filter
      smoothThrottle =
          (filterAlpha * throttle) + ((1.0 - filterAlpha) * smoothThrottle);
      smoothSteering =
          (filterAlpha * steering) + ((1.0 - filterAlpha) * smoothSteering);

      // Mixing logic using smoothed values
      int left = constrain(smoothThrottle + smoothSteering, -100, 100);
      int right = constrain(smoothThrottle - smoothSteering, -100, 100);

      // Fill Struct
      command.leftMotorDirection = (left >= 0);
      command.leftMotorSpeed = abs(left);
      command.rightMotorDirection = (right >= 0);
      command.rightMotorSpeed = abs(right);
    }

    // Send message via ESP-NOW
    esp_err_t result =
        esp_now_send(carAddress, (uint8_t *)&command, sizeof(command));

    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void calibrateJoystick() {
  long sumX = 0;
  long sumY = 0;
  const int samples = 100;

  Serial.println("Calibrating... Do not touch joystick!");

  for (int i = 0; i < samples; i++) {
    sumX += analogRead(X_AXIS_JOYSTICK_PIN);
    sumY += analogRead(Y_AXIS_JOYSTICK_PIN);
    delay(10);
  }

  // Calculate the difference from the ideal 2047
  offsetX = (sumX / samples) - 2047;
  offsetY = (sumY / samples) - 2047;

  Serial.printf("Calibration Complete! Offsets: X=%d, Y=%d\n", offsetX,
                offsetY);
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&telemetry, incomingData, sizeof(telemetry));
  Serial.printf("Telemetry received!");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, carAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  calibrateJoystick();

  xTaskCreatePinnedToCore(SendCommandsTask,        // Task function
                          "SendCommandsTask",      // Task name
                          2048,                    // Stack size (bytes)
                          NULL,                    // Parameters
                          1,                       // Priority
                          &SendCommandsTaskHandle, // Task handle
                          1                        // Core 1
  );
}

void loop() {}