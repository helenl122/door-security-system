/**
 * @file    FinalProject_Interface.ino
 * @author  Hongrui Wu, Helen Liu
 * @date    13-March-2025
 * @brief   Interface side of the system: controls LEDs, LCD, and RFID scanner.
 * 
 * @details
 * This program implements the interface side of a secure access control system using ESP32.
 * It coordinates RFID scanning, visual feedback via LEDs, user prompts via LCD, and wireless 
 * communication with a paired ESP32 sensor device using ESP-NOW.
 * 
 * **Main functionalities include:**
 * - Scanning RFID tags with MFRC522 via SPI to identify users.
 * - Displaying prompts and access results on a 16x2 I2C LCD screen.
 * - Indicating access status with red (denied) and green (granted) LEDs.
 * - Controlling a ceiling light based on motion detection signals from another ESP32.
 * - Handling ESP-NOW peer-to-peer communication for secure access confirmation.
 * 
 * The system uses FreeRTOS to manage multiple concurrent tasks:
 * - LED control (red/green indicators and ceiling light).
 * - RFID tag scanning.
 * - ID comparison for access control.
 * - LCD updates for user interaction.
 * 
 */


/**
 * @section Libraries
 * @brief Libraries used in this program.
 * 
 * - @ref esp_now.h : Provides ESP-NOW protocol for peer-to-peer wireless communication.
 * - @ref WiFi.h : Required to initialize ESP-NOW in station mode.
 * - @ref SPI.h : Enables communication with the RFID scanner using SPI protocol.
 * - @ref MFRC522.h : Library for interfacing with MFRC522 RFID reader module.
 * - @ref Wire.h : Used for I2C communication with the LCD display.
 * - @ref LiquidCrystal_I2C.h : Controls the I2C-based LCD display.
 * - @ref FreeRTOS.h : Provides FreeRTOS real-time operating system functionality for multitasking.
 * - @ref task.h : Contains task management functions for FreeRTOS.
 */
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/**
 * @section Macros_Variables
 * @brief Macros and global variable definitions used throughout the program.
 */
#define RST_PIN 14  ///< RFID scanner SPI reset pin.
#define SS_PIN 10  ///< RFID scanner SPI slave select (SS) pin.
#define SDA_PIN 2  ///< LCD I2C SDA (Serial Data Line) pin.
#define SCL_PIN 1  ///< LCD I2C SCL (Serial Clock Line) pin.
#define LED_CEILING 16  ///< GPIO pin for ceiling light LED.
#define LED_RED 17  ///< GPIO pin for red LED (access denied indicator).
#define LED_GREEN 18  ///< GPIO pin for green LED (access granted indicator).

// Timer configurations
#define TIMER_DIVIDER_VALUE 80  ///< Timer divider value (80 MHz / 80 = 1 MHz timer clock).
#define TIMER_DIVIDER_SHIFT 13  ///< Bit position for the timer divider in the register.
#define LED_TOGGLE_INTERVAL 20000  ///< LED toggle interval in microseconds (20ms for 50Hz task).


/**
 * @section Handles
 * @brief Task and timer handles used for FreeRTOS task management and periodic operations.
 */

/**
 * @subsection TaskHandles
 * @brief Task handles for different system functionalities.
 */
TaskHandle_t redledHandle;  ///< Handle for the red LED task (access denied indicator).
TaskHandle_t greenledHandle;  ///< Handle for the green LED task (access granted indicator).
TaskHandle_t rfidscannerHandle;  ///< Handle for the RFID scanner task.
TaskHandle_t lcdupdateHandle;  ///< Handle for the LCD update task.
TaskHandle_t idcompareHandle;  ///< Handle for the RFID ID comparison task.
TaskHandle_t ceilinglightHandle;  ///< Handle for the ceiling light control task.

/**
 * @subsection TimerHandles
 * @brief Timer handles for periodic tasks.
 */
TimerHandle_t rfidTimer;  // Handle for the RFID scanner periodic timer (50Hz scanning).

/**
 * @section LCD_Initialization
 * @brief LCD instance for displaying access control messages.
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD display object, I2C address 0x27, 16 columns x 2 rows.

/**
 * @section RFID_Initialization
 * @brief Variables and instances used for RFID scanning and tag management.
 */
// RFID card read buffer
byte readCard[4];  // Buffer to hold the 4-byte UID of the scanned RFID card.

/**
 * @brief Authorized RFID tag ID for granting access.
 */
String MasterTag = "54DB48A3";  // Predefined authorized RFID tag ID (Master key).

// String tag2 = "6D95FA21";  // (Optional) Alternative tag ID for additional authorized users.

/**
 * @brief Last scanned RFID tag ID (updated dynamically when a card is scanned).
 */
String tagID = "";  // Holds the last read RFID tag ID in uppercase hexadecimal format.

/**
 * @brief RFID reader instance using MFRC522 on specified SPI pins.
 */
MFRC522 mfrc522(SS_PIN, RST_PIN);  // RFID scanner instance (Slave Select and Reset pins configured).

/**
 * @section ESP_NOW_Communication
 * @brief Peer MAC address used for ESP-NOW communication.
 */
uint8_t peerAddress[] = {0x24, 0xEC, 0x4A, 0x0E, 0xB5, 0x1C};  // ESP32 peer MAC address for ESP-NOW.



/**
 * @brief Callback function to handle the result of ESP-NOW data transmission.
 *
 * This function is automatically called after attempting to send data via ESP-NOW. 
 * It prints the status of the transmission (Success or Failed) to the Serial monitor 
 * for debugging and monitoring purposes.
 *
 * @param mac_addr Pointer to the MAC address of the peer device to which the data was sent.
 * @param status Status of the data transmission, indicating success or failure.
 * @return No value is returned (void function).
 */
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}


/**
 * @brief Callback function to handle incoming ESP-NOW data.
 *
 * This function is triggered automatically when data is received via ESP-NOW. 
 * It interprets the incoming data as a boolean value, representing the state 
 * of a motion sensor (detected or not detected). The received state is printed 
 * to the Serial monitor for debugging and monitoring purposes. 
 * If motion is detected (receivedState is true), the function notifies 
 * the ceiling light task to turn on the ceiling light.
 *
 * @param recv_info Pointer to information about the sender of the received data.
 * @param incomingData Pointer to the received data buffer.
 * @param len Length of the received data in bytes.
 * @return No value is returned (void function).
 */
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  Serial.print("Received: ");

  // Assuming the received data is a boolean
  bool receivedState = *(bool*)incomingData;
  Serial.print("Motion Sensor state: ");
  Serial.println(receivedState ? "detected" : "Not detected");

  // if a message from another esp32 is recieved
  if (receivedState) {
    // notify the ceiling light to turn on 
    xTaskNotifyGive(ceilinglightHandle);
  }
}


/**
 * @brief Initializes hardware components, ESP-NOW communication, and FreeRTOS tasks.
 * 
 * This function performs the following initializations:
 * - Sets up serial communication for debugging.
 * - Configures ESP-NOW in WiFi Station mode and registers communication callbacks.
 * - Adds a peer device for wireless communication.
 * - Creates FreeRTOS tasks for handling LEDs, LCD, RFID scanning, and access control logic.
 * - Starts a periodic software timer for RFID scanning at 50Hz.
 */
void setup() {
  Serial.begin(115200);  // Initialize serial monitor for debugging output.

  // ----------------- ESP-NOW Initialization -----------------
  WiFi.mode(WIFI_STA);  // Set ESP32 to WiFi Station mode (required for ESP-NOW).

  // Initialize ESP-NOW and check status
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");  // Output error if initialization fails.
    return;  // Stop further setup if ESP-NOW initialization fails.
  } else {
    Serial.println("ESP-NOW init succeeded!");  // Confirm successful initialization.
  }

  // Register callbacks for ESP-NOW communication
  esp_now_register_send_cb(onDataSent);  // Register callback for data sent.
  esp_now_register_recv_cb(onDataRecv);  // Register callback for data received.

  // ----------------- Add Peer Device -----------------
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));  // Clear peerInfo structure.
  memcpy(peerInfo.peer_addr, peerAddress, 6);  // Copy peer MAC address.
  peerInfo.channel = 0;  // Set default communication channel.
  peerInfo.encrypt = false;  // Disable encryption for ESP-NOW (plain communication).

  // Add peer to ESP-NOW network and check status
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");  // Output error if peer addition fails.
    return;  // Stop setup if peer addition fails.
  }

  // ----------------- FreeRTOS Task Creation -----------------
  xTaskCreatePinnedToCore(redLed, "RedLedBlink", 1024, NULL, 2, &redledHandle, 0); // Controls red LED for access denied
  xTaskCreatePinnedToCore(greenLed, "GreenLedBlink", 1024, NULL, 2, &greenledHandle, 0); // Controls green LED for access granted
  xTaskCreatePinnedToCore(lcdUpdate, "LcdUpdate", 4096, NULL, 2, &lcdupdateHandle, 0); // Handles LCD display updates
  xTaskCreatePinnedToCore(idCompare, "IDCompare", 4096, NULL, 1, &idcompareHandle, 0); // Compares RFID tags with authorized list
  xTaskCreatePinnedToCore(ceilingLight, "CeilingLight", 1024, NULL, 1, &ceilinglightHandle, 0); // Controls ceiling light on motion

  // ----------------- RFID Timer Initialization -----------------
  // Create a periodic software timer for RFID scanning (50Hz)
  rfidTimer = xTimerCreate(
    "rfidTimer", pdMS_TO_TICKS(20), pdTRUE, NULL, rfidScanner
  );  // Timer set to trigger every 20ms (50Hz).

  // Start the RFID scanner timer if successfully created
  if (rfidTimer != NULL) {
    xTimerStart(rfidTimer, 0);  // Start periodic RFID scanning.
  }
}


/**
 * @brief RFID scanner task to detect and read RFID tags.
 *
 * This function initializes the RFID scanner using the SPI interface 
 * and continuously scans for new RFID tags. Once a tag is detected, 
 * its unique ID (UID) is read, converted to a string in uppercase hexadecimal 
 * format, and stored in the global `tagID` variable. After reading, 
 * the function sends a notification to the `idcompare` task to validate 
 * the detected tag. 
 * 
 * The scanning loop runs indefinitely with a delay to avoid busy-waiting.
 *
 * @param rfidTimer Handle to the software timer (currently unused within this function).
 * @return No value is returned (void function).
 */
void rfidScanner(TimerHandle_t rfidTimer) {
  // Initialize RFID scanner with SPI protocol
  SPI.begin();  // Start SPI bus for RFID communication.
  mfrc522.PCD_Init();  // Initialize MFRC522 RFID reader.

  while (1) {
    // Check if a new RFID tag is detected and readable
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      tagID = "";  // Clear previous tag ID.
      // Read and concatenate 4-byte UID into a string
      for (uint8_t i = 0; i < 4; i++) {
        tagID.concat(String(mfrc522.uid.uidByte[i], HEX));  // Convert each byte to HEX string.
      }
      tagID.toUpperCase();  // Convert tag ID to uppercase for consistency.
      mfrc522.PICC_HaltA();  // Halt communication with the RFID tag.

      xTaskNotifyGive(idcompareHandle);  // Notify ID comparison task for validation.
    }
    vTaskDelay(100);  // Delay to avoid busy-waiting (time unit: ticks).
  }
}


/**
 * @brief Task to control the red LED indication.
 *
 * This function initializes the red LED pin and keeps it turned off initially. 
 * It waits for a notification to turn on the LED. Upon receiving a notification, 
 * the LED is turned on for 1 second and then turned off. 
 * The task runs indefinitely, waiting for subsequent notifications to repeat the process.
 *
 * @param pvParameters Pointer to parameters passed during task creation (unused).
 * @return No value is returned (void function).
 */
void redLed(void *pvParameters) {
  // ------------------- Initialization -------------------
  pinMode(LED_RED, OUTPUT);  // Configure red LED GPIO as output.
  digitalWrite(LED_RED, LOW);  // Start with LED turned off.

  // ------------------- Task Loop -------------------
  while (1) {
    // Wait indefinitely until notified to blink the red LED
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      digitalWrite(LED_RED, HIGH);  // Turn on the red LED.
      vTaskDelay(1000);  // Keep LED on for 1 second.
      digitalWrite(LED_RED, LOW);  // Turn off the red LED.
    }
  }
}


/**
 * @brief Task to control the green LED indication.
 *
 * This function initializes the green LED pin and ensures it is turned off at startup. 
 * The task waits indefinitely for a notification. Upon receiving a notification, 
 * it turns on the green LED for 1 second and then turns it off. 
 * The task runs continuously, responding to each notification in the same manner.
 *
 * @param pvParameters Pointer to parameters passed during task creation (unused).
 * @return No value is returned (void function).
 */
void greenLed(void *pvParameters) {
  // ------------------- Initialization -------------------
  pinMode(LED_GREEN, OUTPUT);  // Configure green LED GPIO as output.
  digitalWrite(LED_GREEN, LOW);  // Start with LED turned off.

  // ------------------- Task Loop -------------------
  while (1) {
    // Wait indefinitely for a notification to trigger LED blink
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      digitalWrite(LED_GREEN, HIGH);  // Turn on the green LED.
      vTaskDelay(1000);  // Keep LED on for 1 second.
      digitalWrite(LED_GREEN, LOW);  // Turn off the green LED.
    }
  }
}


/**
 * @brief Task to compare scanned RFID tag ID with authorized IDs and take appropriate actions.
 *
 * This function waits for a notification indicating that a tag has been scanned. 
 * Once triggered, it compares the scanned `tagID` with the predefined `MasterTag`. 
 * If the tag matches (access granted), it displays an "Access Granted" message on the LCD, 
 * notifies the green LED task to turn on the green LED, and sends a confirmation message 
 * to a paired ESP32 device via ESP-NOW. 
 * If the tag does not match (access denied), it displays an "Access Denied" message 
 * and notifies the red LED task to turn on the red LED.
 * After either action, it notifies the LCD update task to refresh the display.
 *
 * @param pvParameters Pointer to parameters passed during task creation (unused).
 * @return No value is returned (void function).
 */
void idCompare(void *pvParameters) {
  while (1) {
    // Wait indefinitely for a notification to trigger ID comparison
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      lcd.clear();  // Clear LCD display before showing result.
      lcd.setCursor(0, 0);  // Set cursor to first line.

      // ----------- Access Check -----------
      if (tagID == MasterTag) {  // Compare scanned tag ID with authorized MasterTag.
        lcd.print(" Access Granted!");  // Show access granted message.
        xTaskNotifyGive(greenledHandle);  // Trigger green LED to indicate access granted.

        bool message = true;  // Message to send via ESP-NOW (true = access granted).
        // Send confirmation message to paired ESP32
        esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &message, sizeof(message));
        // (Optional: Check 'result' for transmission success if needed)
      } else {
        lcd.print(" Access Denied!");  // Show access denied message.
        xTaskNotifyGive(redledHandle);  // Trigger red LED to indicate access denied.
      }

      // ----------- Notify LCD Update Task -----------
      xTaskNotifyGive(lcdupdateHandle);  // Notify LCD update task to refresh display.
    }
  }
}


/**
 * @brief Task to handle LCD updates for the RFID access control system.
 *
 * This function initializes the LCD using the I2C protocol, turns on its backlight, 
 * and displays the default prompt for scanning an RFID card. 
 * The task then runs indefinitely, waiting for a notification to update the display. 
 * Upon receiving a notification (when a new tag is detected), it shows the scanned tag ID 
 * on the LCD for 1 second, then restores the default prompt for the next scan.
 *
 * @param pvParameters Pointer to parameters passed during task creation (unused).
 * @return No value is returned (void function).
 */
void lcdUpdate(void *pvParameters) {
  // ------------------- LCD Initialization -------------------
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C communication with specified SDA and SCL pins.
  lcd.init();  // Initialize LCD module.
  lcd.backlight();  // Turn on LCD backlight.
  lcd.begin(16, 2);  // Set LCD size to 16 columns and 2 rows.

  // Display initial UI prompt for scanning RFID card
  lcd.clear();  // Clear the display.
  lcd.print(" Access Control ");  // Display system title.
  lcd.setCursor(0, 1);  // Move to the second line.
  lcd.print("Scan Your Card>>");  // Display scan prompt.

  // ------------------- Task Loop -------------------
  while (1) {
    // Wait indefinitely until notified to update the LCD
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      // ----------- Display Scanned RFID Tag -----------
      lcd.setCursor(0, 1);  // Set cursor to the second line.
      lcd.print(" ID : ");
      lcd.print(tagID);  // Display the scanned RFID tag ID.

      vTaskDelay(1000);  // Keep tag ID displayed for 1 second.

      // ----------- Restore Default Prompt -----------
      lcd.clear();  // Clear display.
      lcd.print(" Access Control ");  // Display system title again.
      lcd.setCursor(0, 1);
      lcd.print("Scan Your Card>>");  // Prompt for next scan.
    }
  }
}


/**
 * @brief Task to control the ceiling light LED indicator.
 *
 * This function initializes the ceiling light LED pin and ensures it is turned off initially. 
 * The task waits indefinitely for a notification. Upon receiving a notification, 
 * it turns on the ceiling light LED for 1 second and then turns it off. 
 * The task runs continuously, responding to each notification in the same way.
 *
 * @param pvParameters Pointer to parameters passed during task creation (unused).
 * @return No value is returned (void function).
 */
void ceilingLight(void *pvParameters) {
  // --------------- Ceiling LED Initialization ---------------
  pinMode(LED_CEILING, OUTPUT);  // Configure ceiling light LED GPIO as output.
  digitalWrite(LED_CEILING, LOW);  // Start with LED turned off.

  // ------------------- Task Loop -------------------
  while (1) {
    // Wait indefinitely for motion-detection notification
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      digitalWrite(LED_CEILING, HIGH);  // Turn on ceiling light LED.
      vTaskDelay(1000);  // Keep light on for 1 second.
      digitalWrite(LED_CEILING, LOW);  // Turn off ceiling light LED.
    }
  }
}


/**
 * @brief FreeRTOS does not use the standard Arduino loop.
 */
void loop() {
  // Empty loop for freeRTOS
}