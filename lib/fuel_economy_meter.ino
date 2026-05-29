#include <Arduino.h>
#include <driver/ledc.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <Preferences.h>
#include "Wire.h"
#include <MPU6050_light.h>

#include "trip.h"
#include "config.h"
#include "page_main.h"
#include "page_json.h"
#include "page_dashboard.h"

static Trip trip;

AsyncWebServer server(80);
const char *ap_ssid = "Reborn";
const char *ap_password = "Zain@1122";
IPAddress customIP(192, 168, 4, 1);

const char *ssid = "Logout";
const char *password = "ms120400427";

JSONVar readings;

bool IS_HEIGH = false;

char currentGear = '-';
bool converterLockup = false;
float lph = 0.0f;
float Kmh = 0.0f;
float kmh2 = 0.0f;
uint32_t inj_timing = 0;
float duty_cycle = 0.0f;
float trip_fuel = 0.0f;
float inst_kml = 0.0f;
uint_fast16_t rpm = 0;

float vss1_pulses = 0.0f;
float vss2_pulses = 0.0f;
float trip_distance = 0.0f;
float trip_avg = 0;

int vss1_outPut = 0;

float g_force = 0.0f;

int o2_val = 59;
bool relay_status = true;
float inj_freq = 0.0f;

unsigned long engineStartTime = 0;    // Variable to store the start time
unsigned long engineEndTime = 0;      // Variable to store the end time
unsigned long engineRunningTime = 0;  // Variable to store the running time

float totalFuelUsed = 0;
float totalDistance = 0;
float totalAverage = 0;
bool isTripNeedToSave = false;

MPU6050 mpu(Wire);
Preferences preferences;
const int numReadings = 10;    // Number of readings to average
float zReadings[numReadings];  // Array to store sensor readings
int indx = 0;                  // Index for storing new readings
float total = 0;               // Running total of sensor readings

String mili2Time(unsigned long totMillis) {
  // Convert milliseconds to minutes and seconds
  unsigned long minutes = totMillis / (60 * 1000);           // 60,000 milliseconds in a minute
  unsigned long seconds = (totMillis % (60 * 1000)) / 1000;  // Remaining milliseconds converted to seconds

  // Create a String to store the result
  String tm = String(minutes);

  // Add a leading zero for single-digit minutes
  if (minutes < 10) {
    tm = "0" + tm;
  }

  // Add a colon between minutes and seconds
  tm += ":";

  // Add a leading zero for single-digit seconds
  if (seconds < 10) {
    tm += "0";
  }

  // Concatenate the seconds to the result String
  tm += String(seconds);

  // Return the formatted time
  return tm;
}
const float alpha = 0.6;
float ZsmoothedValue = 0;
void AccelerationCheck() {
  mpu.fetchData();
  float x = mpu.getAccX();
  //Serial.println("X val is "+String(x));
  float y = mpu.getAccY();
  float z = mpu.getAccZ();
  float t = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) - 1;
  ZsmoothedValue = alpha * t + (1 - alpha) * ZsmoothedValue;
  total = total - zReadings[indx];
  zReadings[indx] = ZsmoothedValue;
  total = total + zReadings[indx];
  indx = (indx + 1) % numReadings;
  if (indx >= numReadings) {
    indx = 0;
  }
  // Calculate the average
  g_force = total / numReadings;
}

void outputVSS() {
  if (Kmh > 20) {
    vss1_outPut = vss1_pulses + (vss1_pulses * 10 / 100);
    ledcWriteTone(0, vss1_outPut);
  }
}
int a = 0;
int b = 0;
int c = 0;
int d = 0;
unsigned long lastGearCheck = 0;
void checkCurrentGear() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastGearCheck >= 20) {
    lastGearCheck = currentMillis;
    a = digitalRead(A_PIN);
    b = digitalRead(B_PIN);
    c = digitalRead(C_PIN);
    d = digitalRead(D_PIN);

    if (a == IS_HEIGH && b == IS_HEIGH && c == IS_HEIGH && Kmh > 10.0f) {
      currentGear = '4';
    } else if (a != IS_HEIGH && b != IS_HEIGH && c == IS_HEIGH && Kmh > 10.0f) {
      currentGear = '3';
    } else if (a == IS_HEIGH && b != IS_HEIGH && c != IS_HEIGH && Kmh > 5.0f) {
      currentGear = '2';
    } else if (a != IS_HEIGH && b == IS_HEIGH && c != IS_HEIGH && Kmh > 42.0f) {
      currentGear = '5';
    } else if (a != IS_HEIGH && b == IS_HEIGH && c != IS_HEIGH /*&& Kmh >  0.0f*/) {
      currentGear = '1';
    } else if (a != IS_HEIGH && b != IS_HEIGH && c != IS_HEIGH && d != IS_HEIGH) {
      currentGear = 'N';
    } else if (a != IS_HEIGH && b != IS_HEIGH && c != IS_HEIGH && d == IS_HEIGH) {
      currentGear = 'P';
    } else if (a != IS_HEIGH && b == IS_HEIGH && c == IS_HEIGH && d == IS_HEIGH) {
      currentGear = 'R';
    }

    if (d == IS_HEIGH) {
      converterLockup = true;
    } else {
      converterLockup = false;
    }
    //Serial.println("D" + String(d));
    //pinTest();
  }
}

unsigned long lastConsumptionCheck = 0;
void checkConsumption() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastConsumptionCheck >= 20) {
    lastConsumptionCheck = currentMillis;
    rpm = trip.getRpm();
    trip_distance = trip.getKm();
    Kmh = round(trip.getKmh());
    lph = trip.getLph();
    //inst_kml = (kmh > 0.0f && lph > 0.0f) ? ( kmh / lph ) : 0.0f
    inst_kml = trip.getInstantLkm();
    trip_avg = trip_distance / lph;
    kmh2 = round(trip.getKmh2());
    vss1_pulses = trip.getVel();
    vss2_pulses = trip.getVel2();
    inj_timing = trip.getInjectionTime();
    inj_freq = trip.getInjFreq();
    duty_cycle = trip.getDuty();
    trip_fuel = trip.getLiters();
    AccelerationCheck();

    if (engineStartTime == 0 && rpm > 500) {
      engineStartTime = millis();
      isTripNeedToSave = true;
    } else if (engineStartTime != 0) {
      engineEndTime = millis();
      engineRunningTime = engineRunningTime + engineEndTime - engineStartTime;
      engineStartTime = 0;

      if (isTripNeedToSave) {
        isTripNeedToSave = false;
        preferences.begin("zApp", false);
        preferences.putFloat("totalFuelUsed", totalFuelUsed + lph);
        preferences.putFloat("totalDistance", totalDistance + trip_distance);
        preferences.end();
      }
    }

    //Serial.print("g_force :");
    //Serial.println(g_force);
  }
}

String getData() {
  readings["gear"] = String(currentGear);
  readings["rpm"] = String(rpm);
  readings["speed"] = String(Kmh);
  readings["speed2"] = String(kmh2);
  readings["trip_distance"] = String(trip_distance, 4);
  readings["inj"] = String(inj_timing);
  readings["duty_cycle"] = String(duty_cycle, 4);
  readings["inj_freq"] = String(inj_freq);
  readings["lph"] = String(lph, 2);
  readings["kml"] = String(inst_kml, 2);
  readings["trip_fuel"] = String(trip_fuel, 4);
  readings["trip_avg"] = String(trip_avg, 2);
  readings["tot_fuel"] = String(totalFuelUsed, 4);
  readings["tot_avg"] = String(totalAverage, 2);
  readings["con_loc"] = String(converterLockup);
  readings["vss1"] = String(vss1_pulses);
  readings["vss2"] = String(vss2_pulses);
  readings["relay"] = String(relay_status);
  readings["o2_val"] = String(o2_val);
  readings["g_force"] = String(g_force, 2);
  readings["A"] = String(!a);
  readings["B"] = String(!b);
  readings["C"] = String(!c);
  readings["D"] = String(!d);
  readings["run_time"] = mili2Time(engineRunningTime);

  String jsonString = JSON.stringify(readings);

  return jsonString;
}

void setup() {
 // pinMode(VSS_RELAY_PIN, OUTPUT);
  //digitalWrite(VSS_RELAY_PIN, LOW);
  Serial.begin(115200);
  delay(10);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    zReadings[thisReading] = 0;
  }

  preferences.begin("zApp", true);
  trip.INJ_USEC_LITER = preferences.getFloat("INJ_USEC_LITER", INJ_USEC_LITER_D);
  trip.VSS_PULSE_KM = preferences.getFloat("VSS_PULSE_KM", VSS_PULSE_KM_D);
  relay_status = preferences.getBool("RELAY_STATUS", true);
  o2_val = preferences.getInt("O2_VAL", 59);
  totalFuelUsed = preferences.getFloat("totalFuelUsed", 0);
  totalDistance = preferences.getFloat("totalDistance", 0);
  totalAverage = totalDistance / totalFuelUsed;
  preferences.end();

  WiFi.mode(WIFI_AP_STA);

  WiFi.softAPConfig(customIP, customIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.hostname(String("Reborn").c_str());
  WiFi.begin(ssid, password);

  randomSeed(analogRead(0));
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  mpu.setAccConfig(0);
  mpu.calcOffsets(true, true);  // gyro and accelero

  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(C_PIN, INPUT_PULLUP);
  pinMode(D_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);


  digitalWrite(RELAY_PIN, relay_status);

  dacWrite(DAC_CH1, o2_val);
  Serial.println("Trip");
  trip.begin();


  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", P_MAIN_PAGE, null);
  });

  server.on("/dash", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", P_DASH_BOARD, null);
  });

  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", P_JSON_PAGE, null);
  });


  server.on("/getdata", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getData();
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/update_o2", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("o2_val")) {
      int val = request->arg("o2_val").toInt();
      if (val > 0) {
        o2_val = val;

        if (o2_val > 77) {
          o2_val = 77;
        }
        dacWrite(DAC_CH1, o2_val);
        preferences.begin("zApp", false);
        preferences.putInt("O2_VAL", o2_val);
        preferences.end();
      }
    }
    request->send(200, "text/html", String(o2_val));
  });

  server.on("/relay_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    relay_status = true;
    digitalWrite(RELAY_PIN, relay_status);
    preferences.begin("zApp", false);
    preferences.putBool("RELAY_STATUS", relay_status);
    preferences.end();
    request->send(200, "text/html", "OK");
  });

  server.on("/relay_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    relay_status = false;
    digitalWrite(RELAY_PIN, relay_status);
    preferences.begin("zApp", false);
    preferences.putBool("RELAY_STATUS", relay_status);
    preferences.end();
    request->send(200, "text/html", "OK");
  });

  server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "");  // Respond with an empty body and 200 OK status
  });

  server.on("/update_speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("val")) {
      int Speed = request->arg("val").toInt();
      if (Speed > 0) {
        trip.VSS_PULSE_KM = 3600 / Speed * vss1_pulses;
        preferences.begin("zApp", false);
        preferences.putInt("VSS_PULSE_KM", trip.VSS_PULSE_KM);
        preferences.end();
      }
    }
    request->send(200, "text/html", String(trip.VSS_PULSE_KM, 2));
  });

  server.on("/update_fuel", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("val")) {
      float InstantLph = request->arg("val").toFloat();
      if (InstantLph > 0) {
        trip.INJ_USEC_LITER = 3600000000.0f / InstantLph * duty_cycle;
        preferences.begin("zApp", false);
        preferences.putInt("INJ_USEC_LITER", trip.INJ_USEC_LITER);
        preferences.end();
      }
    }
    request->send(200, "text/html", String(trip.INJ_USEC_LITER, 2));
  });
  server.begin();
  //float vss50 = 50 * 1023 / 100;
  //ledcSetup(0, 0, 10);
  //ledcAttachPin(VSS_OUTPUT_PIN, 0);
  //ledcWrite(0, vss50);

  /*ledcSetup(0, 10, 10); // Channel 0,  10 Hz frequency,10-bit resolution
  //ledcSetup(0, 6, 16);  // channel 0, 6 Hz frequency, 16-bit resolution
  ledcAttachPin(5, 0); // Attach the channel to the GPIO pin
  int dutyCycle = 97.5 * 1023 / 100; // 5% of the maximum value (1023 for 10-bit resolution)
  ledcWrite(0, dutyCycle);*/

  Serial.println("Setup done");
}

void pinTest() {
  if (digitalRead(VSS1_PIN)) {
    Serial.print("VSS1 Off ");
  } else {
    Serial.print("VSS1 On ");
  }

  if (digitalRead(VSS2_PIN)) {
    Serial.print("VSS2 Off ");
  } else {
    Serial.print("VSS2 On ");
  }

  if (digitalRead(INJ_PIN)) {
    Serial.print("Inj Off ");
  } else {
    Serial.print("Inj On ");
  }

  if (digitalRead(A_PIN)) {
    Serial.print("A Off ");
  } else {
    Serial.print("A On ");
  }

  if (digitalRead(B_PIN)) {
    Serial.print("B Off ");
  } else {
    Serial.print("B On ");
  }

  if (digitalRead(C_PIN)) {
    Serial.print("C Off ");
  } else {
    Serial.print("C On ");
  }

  if (digitalRead(D_PIN)) {
    Serial.print("D Off ");
  } else {
    Serial.print("D On ");
  }

  Serial.println("");
}
uint32_t frq = 0;
unsigned long zm = 0;

float Speed = 20.0;  // Replace with the actual speed value
float VSS_PLUS = (Speed * 105000) / 3600;

void loop() {
  checkCurrentGear();
  checkConsumption();
  /*
  unsigned long currentMillis = millis();
  if (currentMillis - zm >= 100) {

    VSS_PLUS = (Speed * 105000) / 3600;
    ledcWriteTone(0, VSS_PLUS);  // Set the PWM frequency
    Serial.print(vss1_pulses);
    Serial.print(" = ");
    Serial.println(VSS_PLUS);
    Speed++;
    if (Speed > 85) {
      Speed = 20;
    }
    zm = currentMillis;
  }
  */



  // Allow the ESP32 to handle interrupts and other tasks
  delay(0);
  //pinTest();
  //delay(300); // Add a delay for stability
}
