/* Kasko Lock - Slave (Full Integration, FINAL VERSION) */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>

// --- Door sensor pins (reed switches) ---
#define DOOR1_PIN 32
#define DOOR2_PIN 33

// --- Locker outputs (simulate lock/unlock relay/servo) ---
#define LOCKER1_PIN 25
#define LOCKER2_PIN 26

// Track last states
int lastDoor1 = HIGH;
int lastDoor2 = HIGH;

// ESP-NOW data structs (MUST MATCH MASTER)
typedef struct { uint8_t command; uint8_t locker; uint32_t counter; } ENSMsg;
typedef struct { uint8_t door1; uint8_t door2; uint32_t uptime; } ENSStatus;

// ❗ CORRECTED MASTER MAC ADDRESS: 0x80, 0xF3, 0xDA, 0x53, 0x79, 0x10
uint8_t masterMac[6] = {0x80, 0xF3, 0xDA, 0x53, 0x79, 0x10};

// --- Send callback ---
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
    Serial.print("[Slave] Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// --- Send door state update to Master (Used for door change and Ping reply) ---
void sendDoorStatus() {
    ENSStatus status;
    status.door1 = (digitalRead(DOOR1_PIN) == LOW ? 0 : 1); 
    status.door2 = (digitalRead(DOOR2_PIN) == LOW ? 0 : 1);
    status.uptime = millis() / 1000;

    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&status, sizeof(status));
    if (result == ESP_OK) {
        Serial.printf("[Slave] Sent Door Status: D1=%s D2=%s\n",
                      status.door1 ? "OPEN" : "CLOSED",
                      status.door2 ? "OPEN" : "CLOSED");
    } else {
        Serial.println("[Slave] Error sending door status!");
    }
}

// --- Receive callback (Handles Ping and Unlock) ---
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(ENSMsg)) {
        ENSMsg msg;
        memcpy(&msg, incomingData, sizeof(msg));
        Serial.printf("[Slave] Command=%d Locker=%d Counter=%lu\n", msg.command, msg.locker, msg.counter);

        // 1. Handshake/Ping Command (Command 0)
        if (msg.command == 0) {
            Serial.println("[Slave] Received Ping. Responding with Status...");
            sendDoorStatus(); // Reply immediately to the Master's ping
        }

        // 2. Unlock Command (Command 1)
        else if (msg.command == 1) { 
            if (msg.locker == 1) {
                digitalWrite(LOCKER1_PIN, HIGH);
                delay(1000); // 1-second pulse
                digitalWrite(LOCKER1_PIN, LOW);
                Serial.println("[Slave] Locker 1 UNLOCK pulse");
            } else if (msg.locker == 2) {
                digitalWrite(LOCKER2_PIN, HIGH);
                delay(1000); // 1-second pulse
                digitalWrite(LOCKER2_PIN, LOW);
                Serial.println("[Slave] Locker 2 UNLOCK pulse");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n-- Kasko Slave Booting...");

    // Door sensors
    pinMode(DOOR1_PIN, INPUT_PULLUP);
    pinMode(DOOR2_PIN, INPUT_PULLUP);

    // Locker outputs
    pinMode(LOCKER1_PIN, OUTPUT);
    pinMode(LOCKER2_PIN, OUTPUT);
    digitalWrite(LOCKER1_PIN, LOW);
    digitalWrite(LOCKER2_PIN, LOW);

    // ESP-NOW setup
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("[Slave] ESP-NOW init failed!");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Add Master as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[Slave] Failed to add Master peer!");
        return;
    }

    // Capture and send initial state
    lastDoor1 = digitalRead(DOOR1_PIN);
    lastDoor2 = digitalRead(DOOR2_PIN);
    Serial.printf("[Slave] Initial Door1=%s Door2=%s\n",
                  lastDoor1 == LOW ? "CLOSED" : "OPEN",
                  lastDoor2 == LOW ? "CLOSED" : "OPEN");
    sendDoorStatus();
}

void loop() {
    int d1 = digitalRead(DOOR1_PIN);
    int d2 = digitalRead(DOOR2_PIN);

    // Check for door state change and send update
    if (d1 != lastDoor1) {
        lastDoor1 = d1;
        Serial.printf("Locker 1: %s\n", d1 == LOW ? "CLOSED" : "OPEN");
        sendDoorStatus();
    }

    if (d2 != lastDoor2) {
        lastDoor2 = d2;
        Serial.printf("Locker 2: %s\n", d2 == LOW ? "CLOSED" : "OPEN");
        sendDoorStatus();
    }

    delay(50);
}