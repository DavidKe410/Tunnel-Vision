#include <Arduino.h>
#include <Wire.h>

//============================================ Configuration ============================================
#define MS4525DO_ADDR 0x28  // Default I2C address

// Sensor calibration constants (Type A)
const float MIN_COUNTS = 1638.3;
const float MAX_COUNTS = 14744.7;
const float FULL_SCALE_PSI = 2.0;  // ±1 psi → span is 2 psi
const float PRESSURE_SCALE = FULL_SCALE_PSI / (MAX_COUNTS - MIN_COUNTS);
const float PSI_TO_PA = 6894.76;
const float RHO = 1.225;  // air density at sea level (kg/m³)
float offset_psi = 0;

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
    int thrust;
    int leftAileron;
    int rightAileron;
    int stabilator;
    int rudder;
    bool manualOverride;
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


//============================================ Functions ============================================

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

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz, remove if unreliability occurs
    delay(100);
    initMS4525();
}

void loop() {
    delay(1000);
    AirspeedData air;
    readMS4525(air);

    if (air.valid) {
        Serial.print(air.pressure_pa, 5);
        Serial.print(",");
        Serial.print(air.airspeed_mps, 2);
        Serial.print(",");
        Serial.println(air.temperature_c, 2);
    }

    delay(500);
}
