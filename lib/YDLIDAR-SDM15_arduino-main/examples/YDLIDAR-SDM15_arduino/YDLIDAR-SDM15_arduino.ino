#include "SDM15.h"

SDM15 sdm15(Serial1);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(460800);

  // wait 1s
  delay(5000);

  Serial.println("get data");

  // get version info
  VersionInfo info = sdm15.ObtainVersionInfo();

  if (info.checksum_error)
  {
    // String message = "";
    Serial.println("checksum error");
    // for (int i = 0; i < 25; i++)
    //   message += String(info.recv[i], HEX);

    // Serial.println(message);
    return;
  }

  // print version info
  Serial.print("model: ");
  Serial.println(info.model);
  Serial.print("hardware_version: ");
  Serial.println(info.hardware_version);
  Serial.print("firmware_version_major: ");
  Serial.println(info.firmware_version_major);
  Serial.print("firmware_version_minor: ");
  Serial.println(info.firmware_version_minor);
  Serial.print("serial_number: ");
  Serial.println(info.serial_number);

  // get self check test
  TestResult test = sdm15.SelfCheckTest();

  if (test.checksum_error)
  {
    Serial.println("checksum error");
    return;
  }

  if (test.self_check_result)
  {
    Serial.println("self check success");
  }
  else
  {
    Serial.println("self check failed");
    Serial.print("error code: ");
    Serial.println(test.self_check_error_code);
    return;
  }

  delay(1000);
}

void loop()
{
  Serial.println("start scan");

  // start scan
  bool result = sdm15.StartScan();

  if (!result)
    Serial.println("start scan checksum error");

  // get scan data 50 times
  for (int i = 0; i < 50000; i++)
  {
    ScanData data = sdm15.GetScanData();

    if (data.checksum_error)
      Serial.println("checksum error");

    Serial.print("scan times: ");
    Serial.print(i);
    Serial.print(" distance: ");
    Serial.print(data.distance);
    Serial.print(" intensity: ");
    Serial.print(data.intensity);
    Serial.print(" disturb: ");
    Serial.println(data.disturb);
  }

  // stop scan
  result = sdm15.StopScan();

  if (!result)
    Serial.println("stop scan checksum error");

  Serial.println("stop scan");

  // wait 1s
  delay(1000);

  // exit(0);
}
