// ---- KASKO LOCK MASTER (Concurrent Multi-User Support) ----

#include <WiFi.h>
#include <esp_now.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <Adafruit_Fingerprint.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h> 

// --- 1. GLOBAL DEFINITIONS & PINS ---
uint8_t SLAVE_MAC[] = {0x80, 0xF3, 0xDA, 0x53, 0xAC, 0xF0}; 

// ADMIN CREDENTIALS
const char* MASTER_PIN = "4321";
const String ADMIN_TAG_UID = "86c331b8";

// FREE ACCESS UIDS (Dev Perks)
const String FREE_UIDS[] = {"1205af05", "placeholder1", "placeholder2", "placeholder3"};

// I2C LCD Pins (SDA=15, SCL=4)
#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 4
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// RFID RC522 Setup (SPI)
#define RST_PIN_RFID 22 
#define SS_PIN_RFID 21   
MFRC522 mfrc522(SS_PIN_RFID, RST_PIN_RFID);

// Fingerprint AS608 Setup (UART2)
HardwareSerial mySerial(2); 
#define FP_RX_PIN 16 
#define FP_TX_PIN 17 
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// Touch/Coin Pins
#define PIN_TOUCH_ENROLL 2   
#define PIN_TOUCH_RETRIEVE 5 
#define PIN_COIN_IN 34       
#define PIN_COIN_ENABLE 19   

// Keypad setup 
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {32, 33, 25, 26}; 
byte colPins[COLS] = {27, 14, 12, 13};  
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// EEPROM Setup
#define EEPROM_SIZE 4
#define PRICE_ADDR 0

// --- 2. SYSTEM VARIABLES & DATA STRUCTURES ---
bool ADMIN_MODE_ACTIVE = false;
unsigned long ADMIN_TIMEOUT_MILLIS = 0;
const unsigned long INACTIVITY_DURATION = 300000; 
float CURRENT_PRICE;

// Boot lockout
unsigned long BOOT_TIME = 0;
const unsigned long BOOT_LOCKOUT_MS = 5000;

// LCD cycling for screen burn prevention
unsigned long lastLCDCycle = 0;
const unsigned long LCD_CYCLE_INTERVAL = 5000;
bool showingAltText = false;

// Coin pulse tracking
volatile uint16_t coinPulseCount = 0;
unsigned long lastCoinPulse = 0;

// Round-robin locker assignment for even wear distribution
uint8_t lastAssignedLocker = 0; // Track last assigned locker (0 = none, 1 or 2)

// Door state tracking
volatile int doorState1 = 0; 
volatile int doorState2 = 0; 

// ESP-NOW Data Structures
typedef struct { uint8_t command; uint8_t locker; uint32_t counter; } ENSMsg;
typedef struct { uint8_t door1; uint8_t door2; uint32_t uptime; } ENSStatus;

// Locker State Structure with waiting-for-close tracking
struct LockerState {
    uint8_t id;
    bool isOccupied;
    String credentialHash; 
    bool isFingerprint;
    bool waitingForClose; // NEW: Track if locker is open and waiting to be closed
    unsigned long openedAt; // NEW: When was it opened
    bool isRetrievalMode; // NEW: Track if this is retrieval (clear on close) or enrollment (keep data)
};
LockerState lockerStates[2] = {
    {1, false, "", false, false, 0, false},
    {2, false, "", false, false, 0, false}
};

// Timeouts / retry config
const unsigned long ENROLL_CRED_TIMEOUT_MS = 15000UL;
const unsigned long PAYMENT_TIMEOUT_MS = 60000UL;
const unsigned long DOOR_CLOSE_TIMEOUT_MS = 120000UL; // 2 minutes to close
const int MAX_UNLOCK_ATTEMPTS = 3;
const unsigned long UNLOCK_RETRY_DELAY_MS = 2000UL;

// --- 3. FUNCTION PROTOTYPES ---
void loadSystemSettings();
void saveSystemSettings();
void checkAdminTimeout();
void handleUserMode();
void handleAdminMode();
void initiateAdminMode();
void exitAdminMode();
String readRFIDTag_timed(unsigned long timeout);
int enrollFingerprint(uint8_t id);
int recognizeFingerprint();
void clearDatabase();
bool unlockLocker(uint8_t locker_id);
void sendUnlockCommand(uint8_t locker_id, uint8_t *mac_addr);
void handleAdminA();
void handleAdminB();
void handleAdminC();
void handleAdminD();
int findAvailableLocker();
int findLockerByCredential(String hash);
void initiateEnrollment();
void initiateRetrieval();
void lcd_print_mirror(String line1, String line2 = "");
void updateIdleLCD();
bool isBootLockoutActive();
void checkOpenLockers(); // NEW: Background check for open lockers

// --- Coin ISR ---
void IRAM_ATTR coinPulseISR() {
  unsigned long now = millis();
  if (now - lastCoinPulse > 50) {
    coinPulseCount++;
    lastCoinPulse = now;
    Serial.println("[COIN] Pulse detected! Count: " + String(coinPulseCount));
  }
}

// --- 4. ESP-NOW CALLBACKS ---
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(ENSStatus)) {
        ENSStatus status;
        memcpy(&status, incomingData, sizeof(status));
        Serial.printf("[ESP-NOW] Door1=%s Door2=%s\n", 
                     status.door1 ? "OPEN" : "CLOSED",
                     status.door2 ? "OPEN" : "CLOSED");
        doorState1 = status.door1 ? 1 : 0;
        doorState2 = status.door2 ? 1 : 0;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  Serial.printf("[ESP-NOW] Send Status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// --- 5. SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== KASKO LOCK MASTER (CONCURRENT MODE) ===");
    
    BOOT_TIME = millis();
    
    EEPROM.begin(EEPROM_SIZE);
    loadSystemSettings();

    // Hardware Pin Setup
    pinMode(PIN_TOUCH_ENROLL, INPUT_PULLUP);
    pinMode(PIN_TOUCH_RETRIEVE, INPUT_PULLUP);
    pinMode(PIN_COIN_IN, INPUT);
    pinMode(PIN_COIN_ENABLE, OUTPUT);
    digitalWrite(PIN_COIN_ENABLE, LOW);
    
    // LCD Initialization 
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    lcd.init();
    lcd.backlight();
    lcd_print_mirror("Kasko Lock", "Initializing...");
    
    // SPI/RFID Initialization
    SPI.begin();
    mfrc522.PCD_Init();
    delay(100);
    Serial.println("✅ RFID RC522 initialized");
    
    // Fingerprint Initialization
    mySerial.begin(57600, SERIAL_8N1, FP_RX_PIN, FP_TX_PIN); 
    delay(100);
    if (finger.verifyPassword()) {
      Serial.println("✅ Fingerprint sensor found!");
    } else {
      Serial.println("❌ Fingerprint sensor not found");
    }

    // ESP-NOW Setup
    WiFi.mode(WIFI_STA);
    Serial.print("Master MAC: ");
    Serial.println(WiFi.macAddress());
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed!");
        lcd_print_mirror("ESP-NOW Failed!", "Check Slave");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, SLAVE_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("✅ Slave peer added");
    } else {
        Serial.println("❌ Failed to add Slave peer");
    }
    
    delay(1000);
    exitAdminMode(); 
    Serial.println("=== SYSTEM READY (CONCURRENT MODE) ===");
    Serial.println("⏳ Boot lockout active for 5 seconds...\n");
}

// --- 6. MAIN LOOP ---
void loop() {
    checkAdminTimeout();
    checkOpenLockers(); // NEW: Monitor open lockers in background
    
    if (ADMIN_MODE_ACTIVE) {
        handleAdminMode();
    } else {
        handleUserMode();
        updateIdleLCD();
    }
}

// --- 7. CORE LOGIC FUNCTIONS ---

bool isBootLockoutActive() {
    return (millis() - BOOT_TIME < BOOT_LOCKOUT_MS);
}

// NEW: Background check for open lockers and auto-clear when closed
void checkOpenLockers() {
    for (int i = 0; i < 2; i++) {
        if (lockerStates[i].waitingForClose) {
            uint8_t locker_id = lockerStates[i].id;
            
            // Check if door closed
            bool isClosed = (locker_id == 1) ? (doorState1 == 0) : (doorState2 == 0);
            
            if (isClosed) {
                Serial.printf("✅ [AUTO] Locker %d closed by user\n", locker_id);
                
                // Only clear if this is RETRIEVAL mode
                if (lockerStates[i].isRetrievalMode) {
                    // Clear locker data (user took items out)
                    lockerStates[i].isOccupied = false;
                    lockerStates[i].credentialHash = "";
                    lockerStates[i].waitingForClose = false;
                    lockerStates[i].isRetrievalMode = false;
                    
                    // Delete fingerprint if applicable
                    if (lockerStates[i].isFingerprint) {
                        finger.deleteModel(locker_id);
                        Serial.println("[FP] Deleted ID " + String(locker_id));
                        lockerStates[i].isFingerprint = false;
                    }
                    
                    Serial.printf("🔓 [AUTO] Locker %d cleared and available\n", locker_id);
                } else {
                    // ENROLLMENT mode - just mark as no longer waiting, KEEP the data!
                    lockerStates[i].waitingForClose = false;
                    Serial.printf("✅ [AUTO] Locker %d secured - items stored, credential active\n", locker_id);
                }
            }
            // Check timeout
            else if (millis() - lockerStates[i].openedAt > DOOR_CLOSE_TIMEOUT_MS) {
                Serial.printf("⚠️ [AUTO] Locker %d timeout (2min) - force clearing wait flag\n", locker_id);
                
                // Just clear the waiting flag on timeout, keep assignment
                // User can still retrieve later with their credential
                lockerStates[i].waitingForClose = false;
                lockerStates[i].isRetrievalMode = false;
                
                Serial.printf("⚠️ [AUTO] Locker %d still assigned, credential still valid\n", locker_id);
            }
        }
    }
}

void updateIdleLCD() {
    // Don't cycle if any locker is waiting for close (user actively using)
    bool anyWaiting = false;
    for (int i = 0; i < 2; i++) {
        if (lockerStates[i].waitingForClose) {
            anyWaiting = true;
            break;
        }
    }
    
    if (anyWaiting) {
        // Show helpful status
        if (millis() - lastLCDCycle >= LCD_CYCLE_INTERVAL) {
            lastLCDCycle = millis();
            
            String msg = "";
            for (int i = 0; i < 2; i++) {
                if (lockerStates[i].waitingForClose) {
                    msg += "L" + String(i+1) + " ";
                }
            }
            lcd_print_mirror(msg + "in use", "Ready for next!");
        }
    } else {
        // Normal idle cycling
        if (millis() - lastLCDCycle >= LCD_CYCLE_INTERVAL) {
            lastLCDCycle = millis();
            showingAltText = !showingAltText;
            
            if (showingAltText) {
                lcd_print_mirror("Smart Locker", "P" + String((int)CURRENT_PRICE) + "/use");
            } else {
                lcd_print_mirror("Kasko Lock", "Enroll/Retrieve");
            }
        }
    }
}

void lcd_print_mirror(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    
    Serial.println("--- LCD OUTPUT ---");
    Serial.println("L1: " + line1);
    
    if (line2 != "") {
        lcd.setCursor(0, 1);
        lcd.print(line2);
        Serial.println("L2: " + line2);
    }
    Serial.println("------------------");
}

void handleUserMode() {
    // Check boot lockout FIRST before any input processing
    if (isBootLockoutActive()) {
        return; // Ignore ALL inputs during boot lockout
    }
    
    char key = keypad.getKey();
    
    if (key) {
        Serial.println("[KEYPAD] Key pressed: " + String(key));
        lastLCDCycle = millis();
    }
    
    if (key == '*') {
        initiateAdminMode();
        return;
    }

    // Button state tracking
    static bool enrollPressed = false;
    static bool retrievePressed = false;
    static unsigned long enrollDebounce = 0;
    static unsigned long retrieveDebounce = 0;
    
    unsigned long now = millis();
    
    // Enroll button
    bool enrollState = (digitalRead(PIN_TOUCH_ENROLL) == LOW);
    if (enrollState && !enrollPressed && (now - enrollDebounce > 300)) {
        enrollPressed = true;
        enrollDebounce = now;
        Serial.println("[BTN] Enroll button pressed");
        lastLCDCycle = millis();
        initiateEnrollment();
    } else if (!enrollState && enrollPressed) {
        enrollPressed = false;
    }
    
    // Retrieve button
    bool retrieveState = (digitalRead(PIN_TOUCH_RETRIEVE) == LOW);
    if (retrieveState && !retrievePressed && (now - retrieveDebounce > 300)) {
        retrievePressed = true;
        retrieveDebounce = now;
        Serial.println("[BTN] Retrieve button pressed");
        lastLCDCycle = millis();
        initiateRetrieval();
    } else if (!retrieveState && retrievePressed) {
        retrievePressed = false;
    }
}

void initiateEnrollment() {
    Serial.println("\n=== ENROLLMENT START ===");
    lcd_print_mirror("1. Enroll Cred...");
    delay(300);
    
    uint8_t locker_id = findAvailableLocker();
    if (locker_id == 0) { 
        lcd_print_mirror("No Lockers Avail"); 
        Serial.println("❌ No available lockers");
        delay(1500); 
        return; 
    }

    String tempHash = "";
    bool isFP = false;
    
    lcd_print_mirror("Scan RFID or", "Place Finger");
    Serial.println("[Enroll] Waiting for RFID or Fingerprint...");
    
    unsigned long credentialTimeout = millis() + ENROLL_CRED_TIMEOUT_MS;
    
    while (millis() < credentialTimeout && tempHash == "") {
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                tempHash += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
                tempHash += String(mfrc522.uid.uidByte[i], HEX);
            }
            tempHash.toLowerCase();
            mfrc522.PICC_HaltA();
            Serial.println("✅ [RFID] Tag: " + tempHash);
            isFP = false;
            break;
        }
        
        if (finger.getImage() == FINGERPRINT_OK) {
            lcd_print_mirror("Finger Detected", "Enrolling...");
            if (enrollFingerprint(locker_id) == 1) {
                tempHash = "FP" + String(locker_id);
                isFP = true;
                Serial.println("✅ [FP] Enrolled as ID " + String(locker_id));
                break;
            } else {
                Serial.println("❌ [FP] Enrollment failed");
            }
        }
        delay(50);
    }
    
    if (tempHash == "") {
        lcd_print_mirror("Enroll Timeout!");
        Serial.println("❌ Credential timeout");
        delay(1200);
        return;
    }

    lockerStates[locker_id - 1].credentialHash = tempHash;
    lockerStates[locker_id - 1].isFingerprint = isFP;

    Serial.println("[Enroll] Credential saved: " + tempHash);

    // Check if UID is free (Dev Perks)
    bool isFreeUID = false;
    if (!isFP) { // Only check for RFID, not fingerprint
        for (int i = 0; i < sizeof(FREE_UIDS)/sizeof(FREE_UIDS[0]); i++) {
            if (tempHash == FREE_UIDS[i]) {
                isFreeUID = true;
                Serial.println("🎁 [Enroll] Free access UID detected: " + tempHash);
                break;
            }
        }
    }

    bool payment_received = false;
    bool manual_override = false;

    if (!isFreeUID) {
        // Payment Phase
        lcd_print_mirror("Insert P" + String((int)CURRENT_PRICE), "Coins");
        Serial.println("[Payment] Waiting for payment...");

        digitalWrite(PIN_COIN_ENABLE, HIGH);
        coinPulseCount = 0;
        attachInterrupt(digitalPinToInterrupt(PIN_COIN_IN), coinPulseISR, FALLING);

        unsigned long timer_start = millis();
        unsigned long lastUpdate = millis();

        while (millis() - timer_start < PAYMENT_TIMEOUT_MS) {
            char key = keypad.getKey();
            if (key) Serial.println("[KEYPAD] Payment screen: " + String(key));

            if (key == '#') {
                manual_override = true;
                Serial.println("⚠️ [Payment] MANUAL OVERRIDE");
                lcd_print_mirror("Manual Override");
                delay(800);
                break;
            }

            if (coinPulseCount > 0) {
                Serial.printf("💰 [Payment] Coins: %d / %d\n", coinPulseCount, (int)CURRENT_PRICE);

                if (coinPulseCount >= (int)CURRENT_PRICE) {
                    payment_received = true;
                    Serial.println("✅ [Payment] Payment complete!");
                    break;
                }

                if (millis() - lastUpdate > 1000) {
                    lcd_print_mirror("Coins: " + String(coinPulseCount), "Need: " + String((int)CURRENT_PRICE));
                    lastUpdate = millis();
                    timer_start = millis();
                }
            }

            delay(50);
        }

        detachInterrupt(digitalPinToInterrupt(PIN_COIN_IN));
        digitalWrite(PIN_COIN_ENABLE, LOW);
    } else {
        // Free access - skip payment
        payment_received = true;
        Serial.println("🎁 [Enroll] Free access granted - skipping payment");
    }

    if (payment_received || manual_override) {
        if (payment_received) {
            lcd_print_mirror("Payment OK!", "Opening L" + String(locker_id));
        } else {
            lcd_print_mirror("Success", "Opening L" + String(locker_id));
        }
        
        lockerStates[locker_id - 1].isOccupied = true;

        // NEW: Non-blocking unlock - just open and mark as waiting
        bool unlocked = unlockLocker(locker_id);
        
        if (unlocked) {
            // Mark locker as waiting for close (ENROLLMENT MODE - keep data!)
            lockerStates[locker_id - 1].waitingForClose = true;
            lockerStates[locker_id - 1].openedAt = millis();
            lockerStates[locker_id - 1].isRetrievalMode = false; // NOT retrieval - keep data!
            
            lcd_print_mirror("L" + String(locker_id) + " Open!", "Store & close");
            Serial.printf("🔓 [Enroll] Locker %d opened - will keep credential after close\n", locker_id);
            delay(1500);
            
            // IMMEDIATELY return to idle - next user can enroll!
            Serial.println("✅ [Enroll] Ready for next user while L" + String(locker_id) + " is in use");
        } else {
            // Unlock failed - rollback
            lockerStates[locker_id - 1].isOccupied = false;
            lockerStates[locker_id - 1].credentialHash = "";
            lockerStates[locker_id - 1].isFingerprint = false;
            lcd_print_mirror("Unlock Failed", "Try Again");
            Serial.println("❌ [Enroll] Failed - cleared");
            delay(1500);
        }
    } else {
        lockerStates[locker_id - 1].credentialHash = "";
        lockerStates[locker_id - 1].isFingerprint = false;
        lcd_print_mirror("Payment Timeout", "Cleared."); 
        Serial.println("❌ Payment timeout");
        delay(1200);
    }
    
    Serial.println("=== ENROLLMENT END ===\n");
}

void initiateRetrieval() {
    Serial.println("\n=== RETRIEVAL START ===");
    lcd_print_mirror("Scan Credential...");
    
    String credential = "";
    int fpID = 0;
    bool found = false;
    
    unsigned long timeout = millis() + ENROLL_CRED_TIMEOUT_MS;
    
    while (millis() < timeout && !found) {
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            credential = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                credential += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
                credential += String(mfrc522.uid.uidByte[i], HEX);
            }
            credential.toLowerCase();
            mfrc522.PICC_HaltA();
            Serial.println("✅ [RFID] Scanned: " + credential);
            found = true;
            break;
        }
        
        if (finger.getImage() == FINGERPRINT_OK) {
            lcd_print_mirror("Checking Finger...");
            if (finger.image2Tz() == FINGERPRINT_OK) {
                if (finger.fingerFastSearch() == FINGERPRINT_OK) {
                    fpID = finger.fingerID;
                    credential = "FP" + String(fpID);
                    Serial.println("✅ [FP] Match ID: " + String(fpID));
                    found = true;
                    break;
                }
            }
        }
        delay(50);
    }
    
    if (!found) {
        lcd_print_mirror("Timeout!");
        Serial.println("❌ Retrieval timeout");
        delay(1200);
        return;
    }
    
    uint8_t locker_id = findLockerByCredential(credential);
    
    if (locker_id > 0 && lockerStates[locker_id - 1].isOccupied) {
        lcd_print_mirror("Welcome!", "Opening L" + String(locker_id));
        Serial.println("✅ Opening Locker " + String(locker_id));
        
        // NEW: Non-blocking unlock
        bool unlocked = unlockLocker(locker_id);
        
        if (unlocked) {
            // Mark as waiting for close - RETRIEVAL MODE (clear on close!)
            lockerStates[locker_id - 1].waitingForClose = true;
            lockerStates[locker_id - 1].openedAt = millis();
            lockerStates[locker_id - 1].isRetrievalMode = true; // Retrieval - will clear on close!
            
            lcd_print_mirror("L" + String(locker_id) + " Open!", "Take items & close");
            Serial.printf("🔓 [Retrieval] Locker %d opened - will auto-clear on close\n", locker_id);
            delay(1500);
            
            Serial.println("✅ [Retrieval] Ready for next user");
        } else {
            lcd_print_mirror("Unlock Failed", "Try Admin");
            Serial.println("❌ Retrieval unlock failed");
            delay(1500);
        }
    } else {
        lcd_print_mirror("Access Denied!");
        Serial.println("❌ Credential not found");
        delay(1200);
    }
    
    Serial.println("=== RETRIEVAL END ===\n");
}

// NEW: Non-blocking unlock with retry
bool unlockLocker(uint8_t locker_id) {
    Serial.printf("[UNLOCK] Attempting unlock for Locker %d\n", locker_id);
    
    for (int attempt = 1; attempt <= MAX_UNLOCK_ATTEMPTS; ++attempt) {
        sendUnlockCommand(locker_id, SLAVE_MAC);
        Serial.printf("[UNLOCK] Attempt %d sent for L%d\n", attempt, locker_id);

        // Wait for door to open
        unsigned long t0 = millis();
        bool opened = false;
        while (millis() - t0 <= UNLOCK_RETRY_DELAY_MS) {
            if ((locker_id == 1 && doorState1 == 1) || (locker_id == 2 && doorState2 == 1)) {
                opened = true;
                break;
            }
            delay(50);
        }

        if (opened) {
            Serial.printf("✅ [UNLOCK] Locker %d opened (attempt %d)\n", locker_id, attempt);
            return true;
        } else {
            Serial.printf("⚠️ [UNLOCK] Attempt %d: no open detected for L%d\n", attempt, locker_id);
            if (attempt < MAX_UNLOCK_ATTEMPTS) {
                Serial.printf("[UNLOCK] Retrying in %lums\n", UNLOCK_RETRY_DELAY_MS);
                delay(UNLOCK_RETRY_DELAY_MS);
            }
        }
    }
    
    Serial.printf("❌ [UNLOCK] All %d attempts failed for L%d\n", MAX_UNLOCK_ATTEMPTS, locker_id);
    return false;
}

void sendUnlockCommand(uint8_t locker_id, uint8_t *mac_addr) {
    ENSMsg msg;
    msg.command = 1; 
    msg.locker = locker_id;
    msg.counter = (uint32_t) millis();
    
    Serial.printf("[ESP-NOW] Sending unlock for Locker %d...\n", locker_id);
    esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.println("✅ [ESP-NOW] Command sent");
    } else {
        Serial.printf("❌ [ESP-NOW] Send failed: %d\n", result);
    }
}

String readRFIDTag_timed(unsigned long timeout) {
    String uid = "";
    lcd_print_mirror("Scan RFID Tag...");
    Serial.printf("[RFID] Waiting %lu ms...\n", timeout);
    unsigned long startTime = millis();
    
    while (millis() - startTime < timeout) {
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                uid += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
                uid += String(mfrc522.uid.uidByte[i], HEX);
            }
            uid.toLowerCase();
            mfrc522.PICC_HaltA();
            Serial.println("✅ [RFID] Tag: " + uid);
            return uid;
        }
        delay(50);
    }
    Serial.println("❌ [RFID] Timeout");
    return "";
}

int enrollFingerprint(uint8_t id) {
    lcd_print_mirror("Place Finger #1");
    Serial.println("[FP] Waiting for image 1...");
    
    unsigned long timeout = millis() + 30000; 
    while (millis() < timeout) {
        if (finger.getImage() == FINGERPRINT_OK) {
            Serial.println("[FP] Image 1 OK");
            break; 
        }
        if (millis() >= timeout) {
            Serial.println("[FP] Timeout image 1");
            return 0;
        }
        delay(50);
    }

    if (finger.image2Tz(1) != FINGERPRINT_OK) { 
        Serial.println("[FP] Convert 1 FAIL"); 
        return 0; 
    }

    lcd_print_mirror("Remove Finger"); 
    delay(1000); 
    while (finger.getImage() != FINGERPRINT_NOFINGER) delay(50);
    
    lcd_print_mirror("Place Finger #2");
    Serial.println("[FP] Waiting for image 2...");

    timeout = millis() + 30000; 
    while (millis() < timeout) {
        if (finger.getImage() == FINGERPRINT_OK) {
            Serial.println("[FP] Image 2 OK");
            break;
        }
        if (millis() >= timeout) {
            Serial.println("[FP] Timeout image 2");
            return 0;
        }
        delay(50);
    }
    
    if (finger.image2Tz(2) != FINGERPRINT_OK) { 
        Serial.println("[FP] Convert 2 FAIL"); 
        return 0; 
    }
    
    Serial.println("[FP] Creating model...");
    if (finger.createModel() != FINGERPRINT_OK) { 
        Serial.println("[FP] Create FAIL"); 
        return 0; 
    }
    
    if (finger.storeModel(id) == FINGERPRINT_OK) {
        Serial.printf("✅ [FP] Stored ID %d\n", id);
        return 1; 
    }
    Serial.println("[FP] Store FAIL");
    return 0; 
}

int recognizeFingerprint() {
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        if (finger.getImage() != FINGERPRINT_OK) { delay(50); continue; }
        if (finger.image2Tz() != FINGERPRINT_OK) { delay(50); continue; }
        if (finger.fingerFastSearch() == FINGERPRINT_OK) {
            Serial.printf("✅ [FP] Match ID: %d\n", finger.fingerID);
            return finger.fingerID;
        }
        delay(50);
    }
    return 0;
}

void clearDatabase() {
    finger.emptyDatabase();
    Serial.println("[FP] Database cleared");
}

// --- 8. ADMIN MODE IMPLEMENTATION --- 
void initiateAdminMode() {
    lcd_print_mirror("Admin: Scan Tag", "or Enter PIN");
    Serial.println("\n[ADMIN] Waiting for auth...");

    unsigned long startTime = millis();
    String pinAttempt = "";
    
    while (millis() - startTime < 30000) { 
        char key = keypad.getKey();
        if (key) Serial.println("[KEYPAD] Admin auth: " + String(key));

        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String rfidUID = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                rfidUID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
                rfidUID += String(mfrc522.uid.uidByte[i], HEX);
            }
            rfidUID.toLowerCase();
            mfrc522.PICC_HaltA();
            
            Serial.println("[ADMIN] RFID: " + rfidUID);
            
            if (rfidUID == ADMIN_TAG_UID) {
                ADMIN_MODE_ACTIVE = true; 
                ADMIN_TIMEOUT_MILLIS = millis() + INACTIVITY_DURATION;
                lcd_print_mirror("ADMIN MODE", "ACTIVE");
                Serial.println("✅ [ADMIN] Access granted");
                delay(1500);
                return;
            }
        }

        if (key) {
            if (isdigit(key)) {
                pinAttempt += key;
                lcd.setCursor(15 - pinAttempt.length(), 1);
                lcd.print("*");
            } else if (key == '#') {
                if (pinAttempt == MASTER_PIN) {
                    ADMIN_MODE_ACTIVE = true; 
                    ADMIN_TIMEOUT_MILLIS = millis() + INACTIVITY_DURATION;
                    lcd_print_mirror("ADMIN MODE", "ACTIVE"); 
                    Serial.println("✅ [ADMIN] PIN accepted");
                    delay(1500);
                    return;
                } else {
                    lcd_print_mirror("Access Denied"); 
                    Serial.println("❌ [ADMIN] Wrong PIN");
                    delay(1000); 
                    exitAdminMode(); 
                    return;
                }
            }
        }
        delay(50);
    }
    
    lcd_print_mirror("Access Timeout"); 
    Serial.println("❌ [ADMIN] Timeout");
    delay(1000); 
    exitAdminMode();
}

void handleAdminMode() {
    char key = keypad.getKey();
    
    if (key) {
        Serial.println("[KEYPAD] Admin menu: " + String(key));
        ADMIN_TIMEOUT_MILLIS = millis() + INACTIVITY_DURATION; 
        
        if (key == '0') {
            lcd_print_mirror("Press # to Exit");
            char confirm = keypad.waitForKey();
            Serial.println("[KEYPAD] Admin exit: " + String(confirm));
            if (confirm == '#') { 
                exitAdminMode(); 
                return; 
            }
        } else if (key == 'A') { 
            handleAdminA();
        } else if (key == 'B') { 
            handleAdminB();
        } else if (key == 'C') { 
            handleAdminC();
        } else if (key == 'D') { 
            handleAdminD();
        }
    }
}

void checkAdminTimeout() {
    if (ADMIN_MODE_ACTIVE && millis() > ADMIN_TIMEOUT_MILLIS) {
        Serial.println("[ADMIN] Auto timeout");
        exitAdminMode();
    }
}

void exitAdminMode() {
    ADMIN_MODE_ACTIVE = false;
    lcd_print_mirror("Kasko Lock", "Enroll/Retrieve");
    lastLCDCycle = millis();
    Serial.println("[ADMIN] Exited");
}

void handleAdminA() {
    Serial.println("[ADMIN] Function A: Clear Specific Locker");
    lcd_print_mirror("A: Locker ID:", "");
    String idStr = ""; 
    char key;
    
    while ((key = keypad.waitForKey()) != '#') { 
        Serial.println("[KEYPAD] Admin A input: " + String(key));
        if (isdigit(key)) {
            idStr += key; 
            lcd.setCursor(0, 1); 
            lcd.print(idStr); 
        }
    }
    
    uint8_t locker_id = idStr.toInt();
    if (locker_id >= 1 && locker_id <= 2) {
        sendUnlockCommand(locker_id, SLAVE_MAC);
        
        // Clear all states
        lockerStates[locker_id - 1].isOccupied = false;
        lockerStates[locker_id - 1].credentialHash = "";
        lockerStates[locker_id - 1].waitingForClose = false;
        lockerStates[locker_id - 1].isRetrievalMode = false;
        
        if (lockerStates[locker_id - 1].isFingerprint) {
            finger.deleteModel(locker_id);
            lockerStates[locker_id - 1].isFingerprint = false;
        }
        
        lcd_print_mirror("Locker " + String(locker_id), "CLEARED");
        Serial.println("[ADMIN] Locker " + String(locker_id) + " cleared");
    } else { 
        lcd_print_mirror("Invalid ID"); 
        Serial.println("[ADMIN] Invalid locker ID");
    }
    delay(1200);
}

void handleAdminB() {
    Serial.println("[ADMIN] Function B: Check/Open Available");
    lcd_print_mirror("B: *Check #Open");
    char next_key = keypad.waitForKey(); 
    Serial.println("[KEYPAD] Admin B option: " + String(next_key));
    
    if (next_key == '*') {
        int count = 0;
        int waiting = 0;
        for (int i = 0; i < 2; i++) { 
            if (!lockerStates[i].isOccupied) count++;
            if (lockerStates[i].waitingForClose) waiting++;
        }
        lcd_print_mirror("Avail:" + String(count) + " Open:" + String(waiting), "Total: 2");
        Serial.printf("[ADMIN] Available: %d, Open: %d\n", count, waiting);
    } else if (next_key == '#') {
        lcd_print_mirror("Opening Avail...");
        Serial.println("[ADMIN] Opening available lockers");
        for (int i = 0; i < 2; i++) {
            if (!lockerStates[i].isOccupied && !lockerStates[i].waitingForClose) { 
                sendUnlockCommand(lockerStates[i].id, SLAVE_MAC); 
                delay(700); 
            }
        }
        lcd_print_mirror("Unlock Complete");
    }
    delay(1200);
}

void handleAdminC() {
    Serial.println("[ADMIN] Function C: WIPE ALL");
    lcd_print_mirror("C: WIPE ALL?", "Press # Confirm");
    char confirm = keypad.waitForKey();
    Serial.println("[KEYPAD] Admin C confirm: " + String(confirm));
    
    if (confirm == '#') {
        lcd_print_mirror("Wiping System...");
        Serial.println("[ADMIN] Wiping all data");
        
        clearDatabase(); 
        
        for (int i = 0; i < 2; i++) {
            sendUnlockCommand(lockerStates[i].id, SLAVE_MAC);
            lockerStates[i].isOccupied = false;
            lockerStates[i].credentialHash = "";
            lockerStates[i].isFingerprint = false;
            lockerStates[i].waitingForClose = false;
            lockerStates[i].isRetrievalMode = false;
            lockerStates[i].openedAt = 0;
            delay(500);
        }
        lcd_print_mirror("System Wiped!");
        Serial.println("[ADMIN] Wipe complete");
    } else {
        lcd_print_mirror("Cancelled");
        Serial.println("[ADMIN] Wipe cancelled");
    }
    delay(1200);
}

void handleAdminD() {
    Serial.println("[ADMIN] Function D: Set Price");
    lcd_print_mirror("D: Current P" + String((int)CURRENT_PRICE), "Enter New:");
    
    String priceStr = ""; 
    char key;
    
    while ((key = keypad.waitForKey()) != '#') { 
        Serial.println("[KEYPAD] Admin D price input: " + String(key));
        if (isdigit(key)) {
            priceStr += key; 
            lcd.setCursor(0, 1); 
            lcd.print("P" + priceStr);
        }
    }
    
    float new_price = priceStr.toFloat();
    if (new_price > 0.0) {
        CURRENT_PRICE = new_price; 
        saveSystemSettings();
        lcd_print_mirror("Price Updated", "P" + String((int)CURRENT_PRICE));
        Serial.println("[ADMIN] Price set to P" + String(CURRENT_PRICE));
    } else { 
        lcd_print_mirror("Invalid Price");
        Serial.println("[ADMIN] Invalid price");
    }
    delay(1200);
}

// --- 9. EEPROM FUNCTIONS --- 
void loadSystemSettings() {
    EEPROM.get(PRICE_ADDR, CURRENT_PRICE);
    if (isnan(CURRENT_PRICE) || CURRENT_PRICE < 1.0) { 
        CURRENT_PRICE = 10.0; 
        Serial.println("[EEPROM] Using default price: P10");
    } else {
        Serial.println("[EEPROM] Loaded price: P" + String(CURRENT_PRICE));
    }
}

void saveSystemSettings() {
    EEPROM.put(PRICE_ADDR, CURRENT_PRICE);
    EEPROM.commit();
    Serial.println("[EEPROM] Price saved: P" + String(CURRENT_PRICE));
}

// --- 10. Utility Functions ---
int findAvailableLocker() {
    // Round-robin assignment for even wear distribution
    // Start checking from the locker AFTER the last assigned one
    
    int checkOrder[2];
    if (lastAssignedLocker == 0) {
        // First assignment ever, start with locker 1
        checkOrder[0] = 0; // Index for locker 1
        checkOrder[1] = 1; // Index for locker 2
    } else if (lastAssignedLocker == 1) {
        // Last was locker 1, prioritize locker 2
        checkOrder[0] = 1; // Index for locker 2
        checkOrder[1] = 0; // Index for locker 1
    } else {
        // Last was locker 2, prioritize locker 1
        checkOrder[0] = 0; // Index for locker 1
        checkOrder[1] = 1; // Index for locker 2
    }
    
    // Check in round-robin order
    for (int j = 0; j < 2; j++) {
        int i = checkOrder[j];
        // Locker is available if not occupied AND not waiting for close
        if (!lockerStates[i].isOccupied && !lockerStates[i].waitingForClose) {
            lastAssignedLocker = lockerStates[i].id; // Update for next round-robin
            Serial.printf("[System] Round-robin assigned locker: %d (even wear distribution)\n", 
                         lockerStates[i].id);
            return lockerStates[i].id;
        }
    }
    
    Serial.println("[System] No lockers available");
    return 0;
}

int findLockerByCredential(String hash) {
    Serial.println("[Search] Looking for: " + hash);
    for (int i = 0; i < 2; i++) {
        Serial.println("[Search] L" + String(i+1) + ": " + lockerStates[i].credentialHash + 
                      " (Occupied: " + String(lockerStates[i].isOccupied) + 
                      ", Waiting: " + String(lockerStates[i].waitingForClose) + ")");
        if (lockerStates[i].credentialHash == hash && lockerStates[i].isOccupied) {
            return lockerStates[i].id;
        }
    }
    return 0;
}