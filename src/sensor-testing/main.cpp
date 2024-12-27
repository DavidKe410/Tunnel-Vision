#include <Arduino.h>
#include "SDM15.h"

HardwareSerial *COM[7] = {&Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8};
SDM15 **distanceSensor;
int numSensors;
int count = 100;


// put function declarations here:
void cleanup();
void SDM15_setup();
void getSDM15Data();

void setup(){
  Serial.begin(115200);
  Serial1.begin(57600); // HArdware Serial for Holybro telemetry radio
  SDM15_setup();
  delay(1000);
}

void loop(){
  while (count > 0){
    getSDM15Data();
    delay(250);
    count--;
    Serial1.println(count);
  }
  cleanup();
}

// put function definitions here:
void cleanup(){
  for (byte x = 0; x < numSensors; x++){ //Get distance measurements of all sensors
    bool result = distanceSensor[x]->StopScan();
    if (!result){
      Serial.println("stop scan checksum error");
    }else{
      Serial.println("stop scan");
    }
  }
  for (byte x = 0; x < numSensors; x++) { delete distanceSensor[x]; } // Free memory for each sensor object
  delete[] distanceSensor; // Free the array of pointers
}

void SDM15_setup(){
  numSensors = 3;
  distanceSensor = new SDM15*[numSensors];

  for (byte x = 0; x < numSensors; x++){
    COM[x]->begin(460800);
    distanceSensor[x] = new SDM15(*COM[x]);
    TestResult test = distanceSensor[x]->SelfCheckTest();
    if (test.checksum_error) {
      String msg = "SelfCheck checksum error: SDM15 Sensor " + (String) x;
      Serial.println(msg);
      return;
    }
    if (test.self_check_result) {
      String msg = "SelfCheck success: Sensor " + (String) x;
      Serial.println(msg);
    } else {
      char msg[128];
      snprintf(msg, sizeof msg, "SelfCheck failure: Sensor %d, Error code: %d", x, test.self_check_error_code);
      String msgs = (String) msg;
      Serial.println(msg);
      return;
    }
    bool result2 = distanceSensor[x]->SetOutputFrequency(Freq_500Hz);
    if (!result2){
      Serial.println("Set output frequency checksum error.");
    }
    bool result = distanceSensor[x]->StartScan();
    if (!result) {
      Serial.println("Start scan checksum error.");
    }  
  }
}

void getSDM15Data(){
    short distance[numSensors]; //Temporary storage of distance measurements
    for (byte x = 0; x < numSensors; x++){ //Get distance measurements of all sensors
      ScanData data = distanceSensor[x]->GetScanData();
      if (data.checksum_error){//If we encounter a checksum issue zero the data, ideally we would attempt to get sensor data again from the sensor, but after testing this caused major data frequency issues
        distance[x] = 0;
      }else{
        distance[x] = data.distance;
      }   
      Serial1.print("Sensor: ");
      Serial1.print(x);
      Serial1.print(" distance: ");
      Serial1.print(data.distance);
      Serial1.print(" intensity: ");
      Serial1.print(data.intensity);
      Serial1.print(" disturb: ");
      Serial1.println(data.disturb);
    }
}