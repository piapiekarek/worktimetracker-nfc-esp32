#include <Adafruit_PN532.h>
#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include "credentials/credentials.h"

#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif


#include "ESPDateTime.h"

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long currentMillis = millis();

const long interval = 1000;           // interval at which to blink (milliseco

unsigned long count = 0;
// HardwareSerial hardwareSerial {2};
// Adafruit_PN532 nfc(38, hardwareSerial);
#define PN532_IRQ   (25)
#define PN532_RESET (26)  // Not connected by default on the NFC Shield

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

TaskHandle_t Task1;
TaskHandle_t Task2;

bool nfcReaderFoundCard = false;
uint32_t nfcReaderID = 0;

FirebaseJson json;

uint8_t CardUid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t CardUidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

void setupDateTime() {
    // Get German NTP Time
    DateTime.setServer("de.pool.ntp.org");
    DateTime.setTimeZone("CST-2");
    DateTime.begin();

    if (!DateTime.isTimeValid()) {
        Serial.println("Failed to get time from server.");
    } else {
        Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
        Serial.printf("Timestamp is %ld\n", DateTime.now());
    }
}


void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    currentMillis = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            Serial.print(".");
        }
    }
    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
}


void setupFirebase() {

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.database_url = DATABASE_URL;
    config.token_status_callback = tokenStatusCallback; 

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    // // Optional, set number of error retry
    // Firebase.setMaxRetry(fbdo, 3);

    // // Optional, set number of error resumable queues
    // Firebase.setMaxErrorQueue(fbdo, 30);

    config.fcs.upload_buffer_size = 16384;

}


void setupNFC(){

    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
        Serial.print("Didn't find PN53x board\n");
        nfc.begin();
        versiondata = nfc.getFirmwareVersion();
        delay(200);
    }
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
    
}

void printCardUID(const byte *data, const uint32_t numBytes, char *CardIDBuffer, uint8_t CardIDBufferLength){
  uint32_t szPos;
  for (szPos = 0; szPos < numBytes; szPos++) {
    // Append leading 0 for small values
    if (data[szPos] <= 0xF){
        sprintf(CardIDBuffer + szPos, "0");
    }
    sprintf(CardIDBuffer + szPos, "%x", data[szPos]);
  }
//   Serial.println("\n\n %s \n\n", CardIDBuffer);
}


void Task1code( void * pvParameters ){
    
    setupWiFi();
    delay(200);
    setupFirebase();
    delay(200);
    setupDateTime();

    for(;;){
        currentMillis = millis();
        
        if (Firebase.ready() 
        && (currentMillis - previousMillis >= interval)
        && nfcReaderFoundCard) {
        
            previousMillis = currentMillis;

            json.set("users/piekarekpia/" + DateTime.format("%Y%m%d") + "/" + DateTime.format("%H%M%S") + "/State" , "ON");
            json.set("users/piekarekpia/" + DateTime.format("%Y%m%d") + "/" + DateTime.format("%H%M%S") + "/Timestamp" , DateTime.now());
            uint8_t CardIDBufferLength = 8;
            char CardIDBuffer[CardIDBufferLength] = {};
            printCardUID(CardUid, CardUidLength, CardIDBuffer, CardIDBufferLength);

            bool pathExists = Firebase.RTDB.pathExisted(&fbdo, "cards/" + std::string(CardIDBuffer));
            if(!pathExists){
                json.set("cards/" + std::string(CardIDBuffer) + "/Registered" , DateTime.now());
            }

            json.set("cards/" + std::string(CardIDBuffer) + "/LastUsed" , DateTime.now());

            Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, "/worktimetracker", &json) ? "ok" : fbdo.errorReason().c_str());

            count++;
            nfcReaderFoundCard = false;
                
        }
    } 

}

void Task2code( void * pvParameters ){
    setupNFC();
    static long int buttonpresstime = millis(); 

    for(;;){
        uint8_t success;

        success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, CardUid, &CardUidLength);

        if (success&& (millis()-buttonpresstime > 1000)) {
            Serial.println("Found an ISO14443A card");
            nfc.PrintHex(CardUid, CardUidLength);

            buttonpresstime = millis();

            nfcReaderFoundCard = true;
        }
    } 
}



void setup()
{
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500);

    disableCore0WDT();

}


void loop()
{
    
}