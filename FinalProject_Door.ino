/*FinalProject_Door.ino
 * @file   FinalProject_Door.ino
 *   @author    Helen Liu
 *   @author    Hongrui Wu
 *   @date      03-March-2025
 *   @brief     Peripheral controls of smart door security system.
 *   
 *  This file controls the motion sensor, door, and lock motor mechanisms
 *  on an ESP32 while also communicating to a 2nd ESP32 with ESP-NOW.
 */


// ================ Libraries ====================================
#include <Stepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freeRTOS/task.h>

// ================ Global Variables ==============================
// motion sensor variables
int inputPin = 12; // PIR (motion) sensor pin
int pirState = LOW; // previous PIR state; start w/no motion detected

// motor variables
int direction = 1; // direction of lock turn (1=unlock, -1=lock)
const int stepsPerRevolution = 2048;  // Number of steps per revolution
const int RevolutionsPerMinute = 15;  // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
// Create stepper instances of lock & door: IN1-IN3-IN2-IN4 pin order
Stepper lock = Stepper(stepsPerRevolution, 4, 6, 5, 7);
Stepper door = Stepper(stepsPerRevolution, 15, 17, 16, 18);

// Task and Timer Handles
TimerHandle_t sensorTimer;
TaskHandle_t lockTaskHandle;
TaskHandle_t doorTaskHandle;

// ESP32 communication variables
bool message; // message to send
uint8_t peerAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0xAF, 0x70}; // address to send to
esp_now_peer_info_t peerInfo; // holds peer ESP32 info


// ================ Function Prototypes ==========================
void senseMotion(TimerHandle_t xTimer);
void lockTask(void *pvParameters);
void doorTask(void *pvParameters);

// ================ Setup & Loop =================================
/**
 * @brief Intializes hardware & communication protocols.
 * 
 * This function intitializes:
 * - Serial communication for debugging
 * - Motion sensor input pin
 * - Stepper motor speeds
 * - ESP-NOW for wireless communication 
 * - FreeRTOS tasks for door and lock mechanisms
 * - FreeRTOS timer to check for motion detection
 */
void setup() {
  Serial.begin(9600);
  pinMode(inputPin, INPUT); // declare motion sensor pin input
  lock.setSpeed(RevolutionsPerMinute);
  door.setSpeed(RevolutionsPerMinute);
  WiFi.mode(WIFI_STA); // set device as Wi-Fi station
  // init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // register esp now recv cb to call receiveData function
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveData));
  // Register peer ESP32 communication
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer ESP32        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Create door and lock Tasks
  xTaskCreatePinnedToCore(doorTask, "doorTask", 1024, NULL, 1, &doorTaskHandle, 0);
  xTaskCreatePinnedToCore(lockTask, "lockTask", 1024, NULL, 1, &lockTaskHandle, 0);
  // set up motion sensor timer to run at 128 Hz
  sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(8), pdTRUE, NULL, senseMotion);
  if (sensorTimer != NULL) xTimerStart(sensorTimer, 0);
}
 
void loop(){} // not used

// ==================== Task Functions ================================
/**
 * @brief Detects motion & sends message via ESP-NOW
 *
 * This is the callback function triggered by sensorTimer at regular intervals.
 * It reads data from PIR motion sensor & if new motion detected, sends a
 * message to peer ESP32 to turn on ceilinglight.
 *
 * @param xTimer Handle to FreeRTOS timer that triggers this task.
 */
void senseMotion(TimerHandle_t xTimer) {
  while (1) {
    int val = digitalRead(inputPin);  // read motion sensor input value
    vTaskDelay(100);
    if (val == HIGH & pirState == LOW) { // if previously no motion but now sense something        	
      // Send message via ESP-NOW
      message = true;
      esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &message, sizeof(message));
      Serial.println( (result == ESP_OK) ? "Sent with success" : "Error sending data");
    }
    pirState = val; // update previous state
  }
}

/**
 * @brief Callback function triggered when data is received via ESP-NOW
 *
 * This function checks if received boolean data indicates valid keycard scan.
 * If true, notifies lockTask to unlock door.
 * 
 * @param mac MAC address of the sending ESP32
 * @param incomingData Pointer to received data.
 * @param len Length of received data in bytes.
 */
void receiveData(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("received Data");
  // if receive data from RFID scanner that it's the key, notify lock/door tasks
  bool receivedState = *(bool*) incomingData;
  if (receivedState) {
    xTaskNotifyGive(lockTaskHandle);
  }
}

/**
 * @brief Controls locking mechanism by rotating stepper motor.
 *
 * This task runs on Core 0. It waits to be notified before execution.
 * If triggered, it turns the lock motor 90 degrees, either unlocking if
 * positive direction or locking if negative. Once turned in one
 * direction, it sets direction variable to the opposite. If lock is
 * unlocked, it notifies doorTask to open door.
 * 
 * @param pvParameters pointer to task parameters
 */
void lockTask(void *pvParameters) {
  Serial.println("lockTask");
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
        lock.step(stepsPerRevolution*0.25*direction);
        direction *= -1;
        vTaskDelay(1000);
        if (direction == -1) xTaskNotifyGive(doorTaskHandle);
    }
  }
}

/**
 * @brief Controls the door opening and closing
 *
 * This task runs on Core 0. It waits for notification from lockTask.
 * When triggered, it turns the door motor 90 degrees ito open, waits briefly,
 * then rotates to close. After, it notifies lockTask to re-lock door.
 * 
 * @param pvParameters pointer to task parameters
 */
void doorTask(void *pvParameters) {
  Serial.println("doorTask");
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
        // open door 90 degrees
        Serial.println("open: clockwise");
        door.step(stepsPerRevolution*0.25);
        vTaskDelay(2000);
        // close door
        Serial.println("close: counterclockwise");
        door.step(-stepsPerRevolution*0.25);
        xTaskNotifyGive(lockTaskHandle);
    }
  }
}
