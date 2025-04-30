#include <Arduino.h>
#include "SDM15.h"

HardwareSerial* COM[] = {&Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8};
constexpr int numSensors = 2; // <-- Set number of connected sensors here

SDM15* distanceSensor[numSensors];

void SDM15_setup();
void getSDM15Data();

void setup() {
  Serial.begin(115200);
  SDM15_setup();
  delay(1000);
}

void loop() {
  getSDM15Data();
  delay(50);
}

void SDM15_setup() {
  for (int x = 0; x < numSensors; x++) {
    COM[x]->begin(460800);
    distanceSensor[x] = new SDM15(*COM[x]);  // Only the ones you need

    TestResult test = distanceSensor[x]->SelfCheckTest();
    if (test.checksum_error) {
      Serial.print("SelfCheck checksum error: Sensor ");
      Serial.println(x);
      continue;
    }
    if (test.self_check_result) {
      Serial.print("SelfCheck success: Sensor ");
      Serial.println(x);
    } else {
      Serial.print("SelfCheck failure: Sensor ");
      Serial.print(x);
      Serial.print(", Error code: ");
      Serial.println(test.self_check_error_code);
      continue;
    }

    if (!distanceSensor[x]->SetOutputFrequency(Freq_1000Hz)) {
      Serial.println("Set output frequency checksum error.");
    }
    if (!distanceSensor[x]->StartScan()) {
      Serial.println("Start scan checksum error.");
    }
  }
}

void getSDM15Data() {
  for (int x = 0; x < numSensors; x++) {
    ScanData data = distanceSensor[x]->GetScanData();
    if (data.checksum_error) {
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.println(" checksum error, distance = 0");
    } else {
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.print(" distance: ");
      Serial.print(data.distance);
      Serial.print(" intensity: ");
      Serial.print(data.intensity);
      Serial.print(" disturb: ");
      Serial.println(data.disturb);
    }
  }
}
