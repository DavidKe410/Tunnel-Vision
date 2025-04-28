#include <Arduino.h>
#include <Wire.h>
#include <PulsePosition.h>
#include "SDM15.h"

//============================================ Configuration ============================================
//====== Airspeed Sensor ======
#define MS4525DO_ADDR 0x28  // Default I2C address

// Sensor calibration constants (Type A)
const float MIN_COUNTS = 1638.3;
const float MAX_COUNTS = 14744.7;
const float FULL_SCALE_PSI = 2.0;  // ±1 psi → span is 2 psi
const float PRESSURE_SCALE = FULL_SCALE_PSI / (MAX_COUNTS - MIN_COUNTS);
const float PSI_TO_PA = 6894.76;
const float RHO = 1.225;  // air density at sea level (kg/m³)
float offset_psi = 0;
//====== End Airspeed Sensor ======

//====== Duplex Receiver ======
PulsePositionInput ReceiverInput(RISING);
const int defaultValues[] = {
    1000,  // Default value for throttle
    1500,  // leftAileron
    1500,  // rightAileron
    1500,  // stabilator
    1500,  // rudder
    1000   // manualOverride
};
//====== End Duplex Receiver ======

//====== SDM15 ======
HardwareSerial* COM[] = { &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8 };
constexpr int numSensors = 2;
SDM15* distanceSensor[numSensors];
constexpr int SDM15_BAUD = 460800;
//====== End SDM15 ======

//========================================== End Configuration ==========================================

//============================================ Data Structs ============================================
// Struct to store airspeed data
struct AirspeedData {
    float pressure_psi;
    float pressure_pa;
    float airspeed_mps;
    float temperature_c;
    bool valid;
};

// Struct to store Duplex receiver data
struct DuplexData {
    int throttle;
    int leftAileron;
    int rightAileron;
    int stabilator;
    int rudder;
    int manualOverride;
    bool defaultValues;
};

// Struct to store SDM15 data
struct SDM15Data {
    short distance[numSensors];
    short intensity[numSensors];
    short disturb[numSensors];
    bool valid[numSensors];
};

// Struct to store BNO085 IMU data
struct IMUData {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    bool valid;
  };

// Struct to consolidate all data
struct AllData {
    AirspeedData airspeed;
    DuplexData duplex;
    IMUData imu;
};
//============================================= End Data Structs ============================================

// Function Declarations (Prototypes)
void initMS4525();
void readMS4525(AirspeedData &data);
void readReceiver(DuplexData &data);
void initSDM15();
void readSDM15(SDM15Data &data);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz, remove if unreliability occurs
    delay(100);

    initMS4525();

    ReceiverInput.begin(6); // PPM capable pin on Teensy 4.1

    initSDM15();
}

void loop() {
    unsigned long startTime = millis();
    for (int i = 0; i < 100; i++) {
        DuplexData data;
        readReceiver(data);
        if (data.defaultValues) {
            Serial.println("Default values used for receiver channels.");
        } else {
            Serial.print("Throttle: ");
            Serial.print(data.throttle);
            Serial.print(", Left Aileron: ");
            Serial.print(data.leftAileron);
            Serial.print(", Right Aileron: ");
            Serial.print(data.rightAileron);
            Serial.print(", Stabilator: ");
            Serial.print(data.stabilator);
            Serial.print(", Rudder: ");
            Serial.print(data.rudder);
            Serial.print(", Manual Override: ");
            Serial.println(data.manualOverride);
        }
        AirspeedData air;
        readMS4525(air);

        if (air.valid) {
            Serial.print(air.pressure_pa, 5);
            Serial.print(",");
            Serial.print(air.airspeed_mps, 2);
            Serial.print(",");
            Serial.println(air.temperature_c, 2);
        }
        SDM15Data distances;
        readSDM15(distances);
        for (int x = 0; x < numSensors; x++) {
            if (distances.valid[x]) {
                Serial.print("Distance: ");
                Serial.print(distances.distance[x]);
                Serial.print(" , ");
            }
        }
        Serial.println();
        //delay(1);
    }
    
    unsigned long endTime = millis();
    float averageCycleTime = (endTime - startTime) / 100.0;
    Serial.print("Average cycle time (ms): ");
    Serial.println(averageCycleTime);
    Serial.print("Sampling frequency (Hz): ");
    Serial.println(1000.0 / averageCycleTime);
    delay(500);
}

//============================================ Functions ============================================

void initSDM15() {
    for (int x = 0; x < numSensors; x++) {
      COM[x]->begin(SDM15_BAUD);
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

void readSDM15(SDM15Data &data) {
    for (int x = 0; x < numSensors; x++) {
      ScanData tempData = distanceSensor[x]->GetScanData();
      if (tempData.checksum_error) {
        data.valid[x] = false;
        Serial.print("Sensor ");
        Serial.print(x);
        Serial.println(" checksum error, distance = 0");
      } else {
        data.distance[x] = tempData.distance;
        data.intensity[x] = tempData.intensity;
        data.disturb[x] = tempData.disturb;
        data.valid[x] = true;
      }
    }
}

void readReceiver(DuplexData &data) {
    int available = ReceiverInput.available();
    if (available > 0) {

        int* fields[] = {
            &data.throttle,
            &data.leftAileron,
            &data.rightAileron,
            &data.stabilator,
            &data.rudder,
            &data.manualOverride
        };

        const int numFields = sizeof(fields) / sizeof(fields[0]);
        int channelsToRead = (available < numFields) ? available : numFields;

        data.defaultValues = (channelsToRead < numFields);

        for (int i = 0; i < numFields; i++) {
            if (i < channelsToRead) {
                *fields[i] = ReceiverInput.read(i + 1);  // Read from receiver if available
            } else {
                *fields[i] = defaultValues[i];  // Set default value if no channel available
            }
        }
    }
}


//======================== Pitot-static Tube Airspeed Sensor ========================
void readMS4525(AirspeedData &data) { // Around 2068 Hz for stnd I2C, with 400k, reaches ~7000 Hz

    // Request 4 bytes from the sensor
    Wire.requestFrom(MS4525DO_ADDR, 4);
    if (Wire.available() == 4) {
        uint8_t b1 = Wire.read();
        uint8_t b2 = Wire.read();
        uint8_t b3 = Wire.read();
        uint8_t b4 = Wire.read();

        // 4 bytes requested, 14-bit pressure, 11-bit temperature from datasheet
        uint16_t raw_pressure = ((b1 & 0x3F) << 8) | b2;
        uint16_t raw_temp = ((b3 << 8) | b4) >> 5;

        // Convert raw values to pressure in psi and temperature in C
        float pressure_psi = ((raw_pressure - MIN_COUNTS) * PRESSURE_SCALE) - (FULL_SCALE_PSI / 2.0);
        float tempC = ((float)raw_temp * 200.0 / 2047.0) - 50.0;

        // Apply offset calibration and conversions to Pa
        pressure_psi -= offset_psi;
        float pressure_pa = pressure_psi * PSI_TO_PA;

        // Calculate airspeed in m/s using Bernoulli's equation
        float airspeed_mps = sqrt(fabs(2.0 * pressure_pa / RHO));

        // Update airspeed data struct
        data.pressure_psi = pressure_psi;
        data.pressure_pa = pressure_pa;
        data.airspeed_mps = airspeed_mps;
        data.temperature_c = tempC;
        data.valid = true;

    }else {
        data.valid = false;
        Serial.println("Failed to read from airspeed sensor.");
    }
}

void initMS4525() {
    // Check if the sensor is responding
    Wire.beginTransmission(MS4525DO_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("Airspeed sensor not responding.");
    }

    // Calibrate the sensor zero/offset by averaging first 100 readings while stationary
    Serial.println("Calibrating airspeed sensor...");
    float zero_sum = 0;
    AirspeedData tempAirspd;
    int countCal = 0;
    while (countCal < 100){
        readMS4525(tempAirspd);
        if (tempAirspd.valid) {
            zero_sum += tempAirspd.pressure_psi;
            countCal++;
        }
        delay(5);
    }
    offset_psi = zero_sum / 100.0;
    Serial.print("Calibrated zero offset: ");
    Serial.println(offset_psi,5);
}
//======================== End Pitot-static Tube Airspeed Sensor ========================

//============================================ End Functions ============================================