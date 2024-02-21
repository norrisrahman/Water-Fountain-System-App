#include <OneWire.h>
#include <DallasTemperature.h>
#include "RTClib.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

#define RELAY_PIN 27
#define ONE_WIRE_BUS 25
#define TdsSensorPin 32
#define VREF 3.3   // analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // sum of sample point

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

int address = 0;

float averageVoltage = 0;
float tdsValue = 0;
float water_temperature = 25;  // current temperature for compensation
String waterStatus = "SAFE";
float air_temperature = 0;
String dateCycle = "";
int daysLeft = 0;

const int numSlides = 2;
int currentSlide = 0;
int targetDay, targetMonth, targetYear;

const char* bleServiceUUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
#define BLEDATAUUID "94bedc82-1bc3-44a8-88bd-17318eb59a44"
const char* deviceName = "BPE Smart Water Fountain";

LiquidCrystal_I2C lcd(0x26, 20, 4);
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

uint8_t sensor2[8] = { 0x28, 0xC8, 0x3E, 0x57, 0x04, 0xE1, 0x3C, 0x6A };
uint8_t sensor1[8] = { 0x28, 0x45, 0x0C, 0x57, 0x04, 0xE1, 0x3C, 0xCD };

DateTime now;


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device Disconnected");
    BLEDevice::startAdvertising();
  }
};



void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  delay(500);

  digitalWrite(RELAY_PIN, LOW);
  delay(500);

  digitalWrite(RELAY_PIN, HIGH);
  delay(500);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  rtc.adjust(DateTime(__DATE__, __TIME__));

  pinMode(TdsSensorPin, INPUT);
  

  sensors.begin();

  bleSetup();

  dateCycle = readStringFromEEPROM();
  // Serial.println(dateCycle);
  parseDate(dateCycle.c_str(), &targetDay, &targetMonth, &targetYear);
  Serial.println(targetDay);
}



void loop() {
  handleStatus();
  getTemperature();
  analogSamplingTDS();
  getTDSValue();
  showLCD();

  // lcd.setCursor(0,0);
  // lcd.print("KONTOL");
}

void getTemperature() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 3000) {  //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    sensors.requestTemperatures();
    water_temperature = sensors.getTempC(sensor1);
    air_temperature = sensors.getTempC(sensor2);
  }
}

void analogSamplingTDS() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }
}

void getTDSValue() {
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 200) {
    printTimepoint = millis();
    // sendBLEdata();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (water_temperature - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      tdsValue = tdsValue/1.5;
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      // Serial.print("TDS Value:");
      // Serial.print(tdsValue,0);
      // Serial.println("ppm");
    }
  }
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


void showLCD() {
  now = rtc.now();
  static unsigned long previousMillis = millis();

  if (millis() - previousMillis >= 3000) {
    sendBLEdata();
    filterChangeDays();

    // It's time to change the slide
    currentSlide = (currentSlide + 1) % numSlides;
    if (currentSlide == 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WATER TEMPERATURE :");
      lcd.setCursor(0, 1);
      lcd.print(String(water_temperature) + " " + (char)223 + "C");
      lcd.setCursor(0, 2);
      lcd.print("AIR TEEMPERATURE  : ");
      lcd.setCursor(0, 3);
      lcd.print(String(air_temperature) + " " + (char)223 + "C");
    } else if (currentSlide == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FILTR CHG: " + String(daysLeft) + " Days");
      // lcd.print("DATE   : " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
      lcd.setCursor(0, 1);
      lcd.print("TDS    : " + String(tdsValue, 0) + " ppm");
      lcd.setCursor(0, 2);
      lcd.print("STATUS : " + waterStatus);
    }

    previousMillis = millis();
  }
}

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    String vall;
    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value received: ");
      for (int i = 0; i < value.length(); i++) {
        // Serial.print(value[i]);
        vall += value[i];
      }
      Serial.println(vall);
      writeStringToEEPROM(vall);
      Serial.println();
      Serial.println("*********");
      dateCycle = readStringFromEEPROM();
      // Serial.println(dateCycle);
      parseDate(dateCycle.c_str(), &targetDay, &targetMonth, &targetYear);
    }
  }
};

void bleSetup() {
  BLEDevice::init(deviceName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(bleServiceUUID);

  // Create the BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    BLEDATAUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  // Start the service
  pService->start();

  // Advertise the service
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void writeStringToEEPROM(String data) {
  EEPROM.begin(512);  // Initialize EEPROM with size (you can adjust the size based on your needs)

  // Write each character of the string to EEPROM
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(address + i, data[i]);
  }

  EEPROM.write(address + data.length(), '\0');  // Null-terminate the string
  EEPROM.commit();                              // Commit the changes
  EEPROM.end();                                 // Free up resources
}



String readStringFromEEPROM() {
  String result = "";
  EEPROM.begin(512);  // Initialize EEPROM with size (should match the size used for writing)

  // Read each character from EEPROM until null terminator is reached
  for (int i = 0;; i++) {
    char c = EEPROM.read(address + i);
    if (c == '\0') {
      break;
    }
    result += c;
  }

  EEPROM.end();  // Free up resources
  return result;
}

int valll = 0;

void sendBLEdata() {
  if (deviceConnected) {
    valll++;
    String dataToSend = String(tdsValue, 0) + "#" + String(water_temperature, 0) + "#" + String(air_temperature, 0) + "#" + String(daysLeft) + "#" + waterStatus + "#";
    Serial.println(dataToSend);
    pCharacteristic->setValue(dataToSend.c_str());
    pCharacteristic->notify();
  }
}


void parseDate(const char* dateString, int* day, int* month, int* year) {
  // Make a copy of the input string because strtok modifies the string
  char dateStringCopy[strlen(dateString) + 1];
  strcpy(dateStringCopy, dateString);

  // Tokenize the string using '/'
  char* token = strtok(dateStringCopy, "/");
  *day = atoi(token);

  token = strtok(NULL, "/");
  *month = atoi(token);

  token = strtok(NULL, "/");
  *year = atoi(token);
}

void filterChangeDays() {
  DateTime targetDate = DateTime(targetYear, targetMonth, targetDay, 0, 0, 0);

  if (now < targetDate) {
    TimeSpan timeLeft = targetDate - now;
    daysLeft = timeLeft.days();

    Serial.print("Days Left: ");
    Serial.println(daysLeft);
  } else {
    Serial.println("Target date reached!");
  }
}

void handleStatus() {
  if (tdsValue >= 500 || daysLeft == 0 ) {
    waterStatus =  "DANGEROUS";
    digitalWrite(RELAY_PIN, LOW);
  }
  else {
  waterStatus = "SAFE";
  digitalWrite(RELAY_PIN, HIGH);
  }
}