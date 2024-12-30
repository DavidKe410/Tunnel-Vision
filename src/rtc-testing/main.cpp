// Test of time-stamp callback with Teensy 3/4.
// The upload time will be used to set the RTC.
// You must arrange for syncing the RTC.

/*
 * time_t t = processSyncMessage();
 *   Think this just fetches PC time- Serial.find, Serial.parseInt

 * Teensy3Clock.set(t);
 * setTime(t);
 */



#include <TimeLib.h>
#include "SdFat.h"

#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFs sd;
FsFile file;

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());

  // Return low time bits in units of 10 ms.
  *ms10 = second() & 1 ? 100 : 0;
}
//------------------------------------------------------------------------------
time_t getTeensy3Time() { return Teensy3Clock.get(); }
//------------------------------------------------------------------------------
void printField(Print* pr, char sep, uint8_t v) {
  if (sep) {
    pr->write(sep);
  }
  if (v < 10) {
    pr->write('0');
  }
  pr->print(v);
}
//------------------------------------------------------------------------------
void printNow(Print* pr) {
  pr->print(year());
  printField(pr, '-', month());
  printField(pr, '-', day());
  printField(pr, ' ', hour());
  printField(pr, ':', minute());
  printField(pr, ':', second());
}
//------------------------------------------------------------------------------
void setup() {
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  Serial.begin(9600);
  while (!Serial) {
    yield();
  }
  Serial.println(F("Type any character to begin"));
  while (!Serial.available()) {
    yield();
  }
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
    return;
  }
  Serial.print(F("DateTime::now "));
  printNow(&Serial);
  Serial.println();

  // Set callback
  FsDateTime::setCallback(dateTime);

  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  // Remove old version to set create time.
  if (sd.exists("RtcTest.txt")) {
    sd.remove("RtcTest.txt");
  }
  if (!file.open("RtcTest.txt", FILE_WRITE)) {
    Serial.println(F("file.open failed"));
    return;
  }
  // Print current date time to file.
  file.print(F("Test file at: "));
  printNow(&file);
  file.println();

  file.close();
  // List files in SD root.
  sd.ls(LS_DATE | LS_SIZE);
  Serial.println(F("Done"));
}
//------------------------------------------------------------------------------
void loop() {}