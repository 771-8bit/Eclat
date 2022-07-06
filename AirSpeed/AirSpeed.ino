#include <Arduino.h>
#include <SensirionI2CSdp.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

File myFile;
File logFile;
char fileName[16];
int fileNum = 0;

SensirionI2CSdp sdp;

int sense_time_millis = millis();
int SD_time_millis = millis();

float differentialPressure_Pa = 0;
float temperature_C = 0;
float rho_kgm3 = 0;
float airspeed_ms = 0;

void setup() {
  pinMode(LEDR, OUTPUT); // error SDP810
  pinMode(LEDG, OUTPUT); // blink while recored
  pinMode(LEDB, OUTPUT); // error SD
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);

  Serial.begin(115200);


  //SD
  Serial.print("Initializing SD card...");
  while (!SD.begin(0)) {
    Serial.println("initialization failed!");
    digitalWrite(LEDB, LOW);
    delay(200);
  }
  digitalWrite(LEDB, HIGH);
  Serial.println("initialization done.");

  String s;
  while (1) {
    s = "LOG";
    if (fileNum < 10) {
      s += "00";
    } else if (fileNum < 100) {
      s += "0";
    }
    s += fileNum;
    s += ".CSV";
    s.toCharArray(fileName, 16);
    if (!SD.exists(fileName)) break;
    fileNum++;
  }
  Serial.print("logFile number is ");
  Serial.println(fileNum);

  myFile = SD.open(fileName, FILE_WRITE);

  if (myFile) {
    myFile.println("differentialPressure[Pa], temperature[C], rho[kgm3], airspeed[ms]");
    myFile.close();
    digitalWrite(LEDB, HIGH);
  } else {
    Serial.println("error opening test.txt");
    digitalWrite(LEDB, LOW);
  }


  //SDP810
  uint16_t error;
  char errorMessage[256];
  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;
  Wire.begin();
  sdp.begin(Wire, SDP8XX_I2C_ADDRESS_0);
  sdp.stopContinuousMeasurement();

  error = sdp.readProductIdentifier(productNumber, serialNumber, serialNumberSize);
  if (error) {
    Serial.print("Error trying to execute readProductIdentifier(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    digitalWrite(LEDR, LOW);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      Serial.print(serialNumber[i], HEX);
    }
    Serial.println();
  }

  error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTCompAndAveraging(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    digitalWrite(LEDR, LOW);
  }

  delay(100);
}

void loop() {
  if (millis() - SD_time_millis >= 500) {
    SD_time_millis = millis();
    if (myFile) {
      myFile.close();
    }
    myFile = SD.open(fileName, FILE_WRITE);
  }

  if (millis() - sense_time_millis >= 100) {
    sense_time_millis += 100;
    digitalWrite(LEDG, !digitalRead(LEDG));
    Serial.print(sense_time_millis);
    Serial.print(",");
    Serial.println(millis());

    uint16_t error;
    char errorMessage[256];

    error = sdp.readMeasurement(differentialPressure_Pa, temperature_C);

    if (error) {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      digitalWrite(LEDR, LOW);
    } else {
      digitalWrite(LEDR, HIGH);
      rho_kgm3 = 0.0034837 * 101325.0 / (temperature_C + 273.5);
      airspeed_ms = sqrt(abs(2.0 * differentialPressure_Pa / rho_kgm3));

      /*
        Serial.print(differentialPressure_Pa);
        Serial.print(",");
        Serial.print(temperature_C);
        Serial.print(",");
        Serial.print(rho_kgm3);
        Serial.print(",");
        Serial.print(airspeed_ms);
        Serial.println();
      */

      if (myFile) {
        myFile.print(differentialPressure_Pa);
        myFile.print(",\t");
        myFile.print(temperature_C);
        myFile.print(",\t");
        myFile.print(rho_kgm3);
        myFile.print(",\t");
        myFile.print(airspeed_ms);
        myFile.println();
        digitalWrite(LEDB, HIGH);
      } else {
        Serial.println("error opening test.txt");
        digitalWrite(LEDB, LOW);
      }
    }
  }

}
