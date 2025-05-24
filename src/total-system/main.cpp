#include <Arduino.h>
#include <Wire.h>
#include <PulsePosition.h>
#include "SDM15.h"
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include "RingBuf.h"
#include "SdFat.h"

//============================================ Configuration ============================================

// ======== General ========
const short CYCLE_TIME_MS = 10; // 100 Hz
unsigned long previousMillis = 0; // For cycle time tracking
// ====== End General ======

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
HardwareSerial* COM[] = { &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7};
constexpr int numSensors = 6;
SDM15* distanceSensor[numSensors];
constexpr int SDM15_BAUD = 460800;
//====== End SDM15 ======

//====== BNO085 ======
#define BNO08X_CS 10
#define BNO08X_INT 5
#define BNO08X_RESET 9
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t orien_reportType = SH2_ARVR_STABILIZED_RV;
sh2_SensorId_t accel_reportType = SH2_ACCELEROMETER;
long reportIntervalUs = 5000;
//====== End BNO085 ======

//======== Servos and ESC ========
Servo esc;
Servo leftAileron;
Servo rightAileron;
Servo stabilator;
Servo rudder;
//======== End Servos and ESC ========

//======= Data Logger ========
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
const size_t LOG_FILE_SIZE = 10 * 25000 * 600;  // 150,000,000 bytes.

// Space to hold around 2s of data for 80 byte lines at 100 sps.
const size_t RING_BUF_CAPACITY  = 8000 * 2;

// Max RingBuf used bytes. Useful to understand RingBuf overrun.
size_t maxUsed = 0;

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;
//===== End Data Logger ========

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
    float yaw;
    float pitch;
    float roll;
    bool valid_accel;
    bool valid_orien;
  };

// Struct to consolidate all data
struct AllData {
    AirspeedData airspeed;
    DuplexData duplex;
    IMUData imu;
    SDM15Data sdm15;
    short state = 0;
};
//============================================= End Data Structs ============================================

// Function Declarations (Prototypes)
void setupSD();
void cleanupSD();
void logData(AllData &data);
void initMS4525();
void readMS4525(AirspeedData &data);
void readReceiver(DuplexData &data);
void initSDM15();
void readSDM15(SDM15Data &data);
void setReports(sh2_SensorId_t reportType, long report_interval);
void readBNO085(IMUData &data);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz, remove if unreliability occurs
    delay(100);

    setupSD();

    initMS4525();

    ReceiverInput.begin(6); // PPM capable pin on Teensy 4.1

    initSDM15();

    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) { delay(10); }
    }
    setReports(orien_reportType, reportIntervalUs);
    setReports(accel_reportType, reportIntervalUs);

    esc.attach(2);
    leftAileron.attach(33);
    rightAileron.attach(3);
    stabilator.attach(36);
    rudder.attach(37);
    esc.writeMicroseconds(1000); // Set ESC to minimum throttle
    leftAileron.writeMicroseconds(1500);
    rightAileron.writeMicroseconds(1500);
    stabilator.writeMicroseconds(1500);
    rudder.writeMicroseconds(1500);

    delay(3000);
}

void loop() {
    AllData all_data;

    readReceiver(all_data.duplex);
    readMS4525(all_data.airspeed);
    readSDM15(all_data.sdm15);
    readBNO085(all_data.imu);

    esc.writeMicroseconds(all_data.duplex.throttle);
    leftAileron.writeMicroseconds(all_data.duplex.leftAileron);
    rightAileron.writeMicroseconds(all_data.duplex.rightAileron);
    stabilator.writeMicroseconds(all_data.duplex.stabilator);
    rudder.writeMicroseconds(all_data.duplex.rudder);

    switch(all_data.state){
        case 0: // Idle state
            Serial.println("State: Idle");
            if(all_data.duplex.manualOverride > 1500) {
                Serial.println("Manual override active, transitioning to state 1.");
                all_data.state = 1; // Transition to active state
            }
            break;
        case 1: // Armed/Primed state
            Serial.println("State: Armed/Primed");
            logData(all_data); 
            float combinedAccelMag = sqrt(pow(all_data.imu.accel_x,2) + pow(all_data.imu.accel_y,2) + pow(all_data.imu.accel_z,2));
            if (combinedAccelMag > 20) { // Example condition to transition
                Serial.println("Conditions met, transitioning to state 2.");
                all_data.state = 2; // Transition to next state
            }
            break;
        case 2: // Launch/Flight state
            Serial.println("State: Launched/Flight");
            logData(all_data);
            if(all_data.duplex.manualOverride < 1500) {
                Serial.println("Transitioning to state 3.");
                all_data.state = 3;
            }
            break;
        case 3: // Landing/ed state
            Serial.println("State: Landing/ed");
            cleanupSD(); // Cleanup SD card
            esc.writeMicroseconds(defaultValues[0]); // Set ESC to minimum throttle
            leftAileron.writeMicroseconds(defaultValues[1]); // Set to default values
            rightAileron.writeMicroseconds(defaultValues[2]);
            stabilator.writeMicroseconds(defaultValues[3]);
            rudder.writeMicroseconds(defaultValues[4]);
            while(true) { // Wait indefinitely after landing
                delay(1000);
            }
            //break;
        default:
            Serial.println("Unknown State");
            all_data.state = 3; // Just cleanup and shutdown
    }
}

//============================================ Functions ============================================
void logData(AllData &data) {
    // Amount of data in ringBuf.
    size_t n = rb.bytesUsed();
    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
        Serial.println("File full - quitting.");
        return;
    }
    if (n > maxUsed) {
        maxUsed = n;
    }
    if (n >= 512 && !file.isBusy()) {
        // Not busy only allows one sector before possible busy wait.
        // Write one sector from RingBuf to file.
        if (512 != rb.writeOut(512)) {
        Serial.println("writeOut failed");
        return;
        }
    }
    rb.print(millis());
    rb.write(',');
    rb.print(data.state);
    rb.write(',');
    rb.print(data.airspeed.pressure_pa, 5);
    rb.write(',');
    rb.print(data.airspeed.airspeed_mps, 2);
    rb.write(',');
    rb.print(data.airspeed.temperature_c, 2);
    rb.write(',');
    rb.print(data.airspeed.valid);
    rb.write(',');
    rb.print(data.duplex.throttle);
    rb.write(',');
    rb.print(data.duplex.leftAileron);
    rb.write(',');
    rb.print(data.duplex.rightAileron);
    rb.write(',');
    rb.print(data.duplex.stabilator);
    rb.write(',');
    rb.print(data.duplex.rudder);
    rb.write(',');
    rb.print(data.duplex.manualOverride);
    rb.write(',');
    rb.print(data.duplex.defaultValues);
    rb.write(',');
    for (int i = 0; i < numSensors; i++) {
        rb.print(data.sdm15.distance[i], 3);
        rb.write(',');
        rb.print(data.sdm15.valid[i]);
        rb.write(',');
    }
    rb.print(data.imu.accel_x, 4);
    rb.write(',');
    rb.print(data.imu.accel_y, 4);
    rb.write(',');
    rb.print(data.imu.accel_z, 4);
    rb.write(',');
    rb.print(data.imu.valid_accel);
    rb.write(',');
    rb.print(data.imu.yaw, 5);
    rb.write(',');
    rb.print(data.imu.pitch, 5);
    rb.write(',');
    rb.print(data.imu.roll, 5);
    rb.write(',');
    rb.println(data.imu.valid_orien);
    if (rb.getWriteError()) {
        // Error caused by too few free bytes in RingBuf.
        Serial.println("WriteError");
        return;
      }
    // Write any RingBuf data to file.
    rb.sync();
}

void cleanupSD(){
    file.truncate();
    Serial.print("fileSize: ");
    Serial.println((uint32_t)file.fileSize());
    Serial.print("maxBytesUsed: ");
    Serial.println(maxUsed);
    file.close();
}

void setupSD(){
    // Initialize the SD.
    if (!sd.begin(SD_CONFIG)) {
      sd.initErrorHalt(&Serial);
    }

    int fileIteration = 0;
    boolean fileCreated = false;
    while(!fileCreated && fileIteration < 1000){
        String tempName = "FLIGHT" + String(fileIteration) + ".csv";
        int str_len = tempName.length() + 1;
        char LOG_FILENAME[str_len];
        tempName.toCharArray(LOG_FILENAME, str_len);
        // Try to create a new file, fail if it already exists
        if (file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_EXCL)) {
            fileCreated = true;
        } else {
            fileIteration++;
        }
    }

    if(!fileCreated){
        Serial.println("No available filename - file not open/created.");
    }

    // File must be pre-allocated to avoid huge
    // delays searching for free clusters.
    if (!file.preAllocate(LOG_FILE_SIZE)) {
      Serial.println("preAllocate failed\n");
      file.close();
      return;
    }
    // initialize the RingBuf.
    rb.begin(&file);
}

void readBNO085(IMUData &data) {
    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports(orien_reportType, reportIntervalUs);
        setReports(accel_reportType, reportIntervalUs);
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                data.accel_x = sensorValue.un.accelerometer.x;
                data.accel_y = sensorValue.un.accelerometer.y;
                data.accel_z = sensorValue.un.accelerometer.z;
                data.valid_accel = true;
                break;
            case SH2_ARVR_STABILIZED_RV:
                data.yaw = sensorValue.un.arvrStabilizedRV.real;
                data.pitch = sensorValue.un.arvrStabilizedRV.i;
                data.roll = sensorValue.un.arvrStabilizedRV.j;
                data.valid_orien = true;
                break;
            default:
                // Poor system, instead of bool, use a short or smth to specify: good data=1,bad=0,old=2, cause this is just old data
                data.valid_accel = false;
                data.valid_orien = false;
                break;
        }
    } else {
        data.valid_accel = false;
        data.valid_orien = false;
        Serial.println("Failed to read from BNO085.");
    }
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
    Serial.println("Setting desired reports");
    if (! bno08x.enableReport(reportType, report_interval)) {
      Serial.println("Could not enable stabilized remote vector or acceleration report");
    }
}

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
        if (!test.self_check_result) {
            Serial.print("SelfCheck failure: Sensor ");
            Serial.print(x);
            Serial.print(", Error code: ");
            Serial.println(test.self_check_error_code);
            continue;
        }
        if (!distanceSensor[x]->SetOutputFrequency(Freq_500Hz)) {
            Serial.println("Set output frequency checksum error.");
        }
        if (!distanceSensor[x]->StartScan()) {
            Serial.println("Start scan checksum error.");
        }
        delay(50);
    }
}

void readSDM15(SDM15Data &data) {
    for (int x = 0; x < numSensors; x++) {
      ScanData tempData = distanceSensor[x]->GetScanData();
      if (tempData.checksum_error) {
        data.valid[x] = false;
        Serial.print("Sensor ");
        Serial.print(x);
        Serial.println(" checksum error");
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