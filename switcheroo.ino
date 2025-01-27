#include <string>

#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"

#define ADVERTISEMENT "Switcheroo"

#define CUTOFF 2000

#define LED_PIN 2

#define TRIG_PIN 16
#define ECHO_PIN 17

uint8_t modifierv = 0b0000;

void bluetoothKeyTask(void*);

struct InputReport {
  uint8_t modifiers; // 0b1 = ctrl, 0b10 = shift, 0b100 = alt
  uint8_t reserved;
  uint8_t pressedKeys[6];
};

const uint8_t REPORT_DESCRIPTOR_MAP[] = {
  USAGE_PAGE(1),      0x01, // Generic Desktop Controls
  USAGE(1),           0x06, // Keyboard
  COLLECTION(1),      0x01, // Application
  REPORT_ID(1),       0x01, // Report ID (1)
  USAGE_PAGE(1),      0x07, // Keyboard/Keypad
  USAGE_MINIMUM(1),   0xE0, // Keyboard Left Control
  USAGE_MAXIMUM(1),   0xE7, // Keyboard Right Control
  LOGICAL_MINIMUM(1), 0x00, // Each bit is either 0 or 1
  LOGICAL_MAXIMUM(1), 0x01,
  REPORT_COUNT(1),    0x08, // 8 bits for the modifier keys
  REPORT_SIZE(1),     0x01,
  HIDINPUT(1),        0x02, // Data, Var, Abs
  REPORT_COUNT(1),    0x01, // 1 byte (unused)
  REPORT_SIZE(1),     0x08,
  HIDINPUT(1),        0x01, // Const, Array, Abs
  REPORT_COUNT(1),    0x06, // 6 bytes (for up to 6 concurrently pressed keys)
  REPORT_SIZE(1),     0x08,
  LOGICAL_MINIMUM(1), 0x00,
  LOGICAL_MAXIMUM(1), 0x65, // 101 keys
  USAGE_MINIMUM(1),   0x00,
  USAGE_MAXIMUM(1),   0x65,
  HIDINPUT(1),        0x00, // Data, Array, Abs
  REPORT_COUNT(1),    0x05, // 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
  REPORT_SIZE(1),     0x01,
  USAGE_PAGE(1),      0x08, // LEDs
  USAGE_MINIMUM(1),   0x01, // Num Lock
  USAGE_MAXIMUM(1),   0x05, // Kana
  LOGICAL_MINIMUM(1), 0x00,
  LOGICAL_MAXIMUM(1), 0x01,
  HIDOUTPUT(1),       0x02, // Data, Var, Abs
  REPORT_COUNT(1),    0x01, // 3 bits (Padding)
  REPORT_SIZE(1),     0x03,
  HIDOUTPUT(1),       0x01, // Const, Array, Abs
  END_COLLECTION(0)         // End application collection
};

bool connected = false;
BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

const InputReport NONEKEY = { };

class BleKeyboardCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    ((BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)))->setNotifications(true);
    connected = true;

    Serial.println("\nConnected\n");
  }

  void onDisconnect(BLEServer* server) {
    ((BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)))->setNotifications(false);
    connected = false;

    Serial.println("\nDisconnected\n");
  }
};

void bluetoothKeyTask(void*) {
  BLEDevice::init(ADVERTISEMENT);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new BleKeyboardCallbacks());

  hid = new BLEHIDDevice(server);
  input = hid->inputReport(1);
  output = hid->outputReport(1);

  hid->manufacturer()->setValue("Nil");
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x02);

  BLESecurity* security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  hid->reportMap((uint8_t*)REPORT_DESCRIPTOR_MAP, sizeof(REPORT_DESCRIPTOR_MAP));
  hid->startServices();

  hid->setBatteryLevel(100);

  BLEAdvertising* advertising = server->getAdvertising();
  advertising->setAppearance(HID_KEYBOARD);
  advertising->addServiceUUID(hid->hidService()->getUUID());
  advertising->addServiceUUID(hid->deviceInfo()->getUUID());
  advertising->addServiceUUID(hid->batteryService()->getUUID());
  advertising->start();

  Serial.println("\nReady\n");

  delay(portMAX_DELAY);
};

void keyPress(uint8_t val) {
  if (!connected)
    return;
  if (val > KEYMAP_SIZE)
    return;

  KEYMAP map = keymap[val];
  
  InputReport report = {
    .modifiers = modifierv,
    .reserved = 0,
    .pressedKeys = {
      map.usage,
      0, 0, 0, 0, 0
    }
  };

  Serial.print((char)val);
  Serial.print(" ");
  Serial.println(val);

  input->setValue((uint8_t*)&report, sizeof(report));
  input->notify();
  delay(100);
  input->setValue((uint8_t*)&NONEKEY, sizeof(NONEKEY));
  input->notify();
}

void setup() {
  Serial.begin(9600);

  xTaskCreate(bluetoothKeyTask, "bluetooth", 20000, NULL, 5, NULL);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Startup complete");
}

int lastDisp = 0;

int distance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);

  long distance = pulseIn(ECHO_PIN, HIGH);

  if (distance == 0)
    return lastDisp;

  lastDisp = distance;

  return distance;
}

bool debounce = false;

void loop() {
  long dist = distance();

  if (debounce) {
    if (dist > CUTOFF) {
      debounce = false;

      Serial.println("Uncovered");
    }
  }
  else {
    if (dist < CUTOFF) {
      debounce = true;

      Serial.println("Covered");

      InputReport report = {
        .modifiers = 0x08 | 0b0001,
        .reserved = 0,
        .pressedKeys = {
          0x4F, // arrow right
          0, 0, 0, 0, 0
        }
      };

      input->setValue((uint8_t*)&report, sizeof(report));
      input->notify();
      delay(100);
      input->setValue((uint8_t*)&NONEKEY, sizeof(NONEKEY));
      input->notify();
    }
  }
}