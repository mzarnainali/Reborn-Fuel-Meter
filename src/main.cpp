#include <Arduino.h>
#define DEBUG 0 // SET TO 0 OUT TO REMOVE TRACES
#define DUMMY_DATA 0
#define ACCELERATION_ON 0
#define RESET_FLASH 0

#define TYPE_LCD 1
#define TYPE_DOOR 2

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif
// #include "helper.h"
#include "RTClib.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include <Arduino_JSON.h>
#include <Preferences.h>
#include "Wire.h"
#include <MPU6050_light.h>
// #include <Smoothed.h>
#include <esp_now.h>
// #include <Ticker.h>
#include <SPI.h>
#include "trip.h"
#include "config.h"
#include "page_main.h"
#include "page_json.h"
#include "page_dashboard.h"
#include "page_export.h"
#include "page_door_lock.h"
#include <driver/mcpwm.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#if ACCELERATION_ON
#include "LowPass.h"
LowPass<2> lp(3, 10, true);
#endif

#if RESET_FLASH
#include "nvs_flash.h"
void resetFlash()
{
  Serial.begin(115200);
  Serial.println("Trying to erse all NVS flash...");
  if (nvs_flash_erase() != ESP_OK)
    Serial.println("===> Error with Flash Erase.");
  if (nvs_flash_init() != ESP_OK)
    Serial.println("===> Error with Flash INIT.");
  Serial.println("Done with NVS flash...");
  delay(1410065407);
}
#endif

const char *serverIndex = "<meta name='viewport' content='width=device-width, initial-scale=1.0'><form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>\n    <input type='file' name='update'>\n    <input type='submit' value='Update'>\n</form>\n<div id='prg'>progress: 0%</div>\n<script>\ndocument.getElementById('upload_form').addEventListener('submit', function(e) {\n    e.preventDefault();\n    \n    var form = document.getElementById('upload_form');\n    var data = new FormData(form);\n\n    var xhr = new XMLHttpRequest();\n    xhr.open('POST', '/ota_update', true);\n\n    xhr.upload.addEventListener('progress', function(evt) {\n        if (evt.lengthComputable) {\n            var per = evt.loaded / evt.total;\n            document.getElementById('prg').innerHTML = 'progress: ' + Math.round(per * 100) + '%';\n        }\n    }, false);\n\n    xhr.onload = function() {\n        if (xhr.status === 200) {\n            console.log('success!');\n        }\n    };\n\n    xhr.onerror = function() {\n        // Handle error\n    };\n\n    xhr.send(data);\n});\n</script>";

volatile int mainLoopCount;
bool debug = true;
bool isAccelerationOk = false;
bool isRTC_Connected = false;
float outSideTempC = 0.0f;
static Trip trip;
RTC_DS3231 rtc;
AsyncWebServer server(80);
DNSServer dnsServer;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_PIN);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature tempSensor(&oneWire);

float global_fuel_price = 0;

// ESP32 Mac Address used in LCD
uint8_t broadcastAddress[] = {0xFC, 0xE8, 0xC0, 0x74, 0x2E, 0x4C};

typedef struct LCD_Data
{
  byte dataType;
  char gear;
  bool converter_lock;
  float inst_kml;
  float inst_lph;
  byte totalTrips;
  byte currentTrip;
  float trip_distance;
  float trip_avg;
  float trip_fuel;
  int trip_price;
  float speed;
  float outSideTemp;
} LCD_Data;

typedef struct Door_Data
{
  byte dataType;
  byte doorStatus;
} Door_Data;

typedef struct LCD_Response
{
  byte tripNo;
  byte buttonState;
} LCD_Response;

struct TripData
{
  float distance;
  float fuel;
  float savedDistance;
  float savedFuel;
  // Function to calculate the average
  float getAverage() const
  {
    return (fuel + savedFuel) != 0
               ? (distance + savedDistance) / (fuel + savedFuel)
               : 0.0; // Prevent division by zero
  }

  int getFuelPrice() const
  {
    return (fuel + savedFuel) * global_fuel_price;
  }

  float getDistance() const
  {
    return distance + savedDistance;
  }

  float getFuel() const
  {
    return fuel + savedFuel;
  }

  void reset()
  {
    distance = 0.0;
    fuel = 0;
    savedDistance = 0;
    savedFuel = 0;
  }
};

const int maxTrips = 5; // Maximum number of trips
byte totalTrips;        // Actual number of trips, to be read from EEPROM
byte currentTrip = 0;
TripData trips[maxTrips]; // Array to hold trip data

LCD_Data lcdData;
LCD_Response dataFromLCD;
Door_Data doorData;
esp_now_peer_info_t peerInfo;

char daysOfWeek[7][12] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"};

JSONVar readings;
bool isGyroConnected = false;
bool IS_HEIGH = true;
bool IS_LOW = false;
bool is_spoofing = false;
char currentGear = '*';
bool converterLockup = false;
float lph = 0.0f;
float speed = 0.0f;
float prvSpeed = 0.0f;
float acceleration = 0.0f;
uint32_t inj_timing = 0;
float duty_cycle = 0.0f;
uint64_t trip_totalInjectionMs = 0;
float inst_kml = 0.0f;
int rpm = 0;
int prvRpm = 0;
uint_fast16_t injectorRPM = 0;
float vss1_pulses = 0.0f;
int vss1_outPut = 0;
float g_force = 0.0f;
int o2_val = 59;
bool relay_status = true;
bool vss_relay_status = false;
float inj_freq = 0.0f;
float current_speed = 0;

unsigned long engineStartTime = 0;   // Variable to store the start time
unsigned long engineEndTime = 0;     // Variable to store the end time
unsigned long engineRunningTime = 0; // Variable to store the running time
unsigned long last_timestamp;

bool isTripNeedToSave = false;
int spoof_gear = 0;

int spoof_per = 0;

long current_time = 0;
unsigned long lastSpeedSpoof = 0;
int oldSpoofValue = 0;

MPU6050 mpu(Wire);
Preferences preferences;

int a = 0;
int b = 0;
int c = 0;
int d = 0;
unsigned long lastGearCheck = 0;
int gear_cnt = 0;

const float alpha = 0.2;
float ZsmoothedValue = 0;

unsigned long lastConsumptionCheck = 0;

int dataUpdateInterval = 100;
String jsonData = "";
bool updateJson = true;
bool checkConsumptionWeb = true;
int lcd_refresh_rate = 5;
int trip_reset_min = 0;
// Smoothed<float> accelerationData;
unsigned long last_lcd_print = 0;
int lcdDataSize = 0;
int doorDataSize = 0;

void printTime(DateTime now)
{
  // Display the current date and time
  D_print("Date & Time: ");
  D_print(now.year(), DEC);
  D_print('/');
  D_print(now.month(), DEC);
  D_print('/');
  D_print(now.day(), DEC);
  D_print(" ");
  D_print(now.hour(), DEC);
  D_print(':');
  D_print(now.minute(), DEC);
  D_print(':');
  D_print(now.second(), DEC);
  D_println();
}

String getData();

void resetTrip(uint8_t tripNo)
{
  float _km = trip.getKm();
  float _ltr = trip.getLiters();
  trip.resetKm();
  trip.resetLiters();
  preferences.begin("zApp", false);
  for (uint8_t i = 0; i <= totalTrips; i++)
  {
    String fuelKey = "trip" + String(i) + "_fuel"; // Generate key dynamically
    String distanceKey = "trip" + String(i) + "_distance";
    if (tripNo == i)
    {
      preferences.putFloat(fuelKey.c_str(), 0);
      preferences.putFloat(distanceKey.c_str(), 0);
      trips[i].reset();
    }
    else
    {
      preferences.putFloat(fuelKey.c_str(), trips[i].getFuel());
      preferences.putFloat(distanceKey.c_str(), trips[i].getDistance());
      trips[i].savedDistance = trips[i].getDistance();
      trips[i].savedFuel = trips[i].getFuel();
      trips[i].fuel = 0;
      trips[i].distance = 0;
    }
  }
  preferences.end();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  return;
  D_print("\r\nLast Packet Send Status:\t");
  D_println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  memcpy(&dataFromLCD, incomingData, sizeof(dataFromLCD));
  D_print("Trip No: ");
  D_print(dataFromLCD.tripNo);
  D_print("   ,Button State: ");
  D_println(dataFromLCD.buttonState);
  if (currentTrip != dataFromLCD.tripNo)
  {
    currentTrip = dataFromLCD.tripNo;
    currentTrip = (currentTrip < 0 || currentTrip > totalTrips) ? 0 : currentTrip;
    preferences.begin("zApp", false);
    preferences.putInt("currentTrip", currentTrip);
    preferences.end();
  }
  if (currentTrip >= 0 && currentTrip <= totalTrips && dataFromLCD.buttonState == 8)
  {
    resetTrip(currentTrip);
  }
  // D_print("button pressed: ");
  // D_println(dataFromLCD.button);
}

void setup_vss_output()
{
  /*
  // Initialize MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, VSS_OUTPUT_PIN);

  // Configure MCPWM
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1; // Set frequency in Hz
  pwm_config.cmpr_a = 50.0; // Set duty cycle to 50%
  pwm_config.cmpr_b = 0;    // Not used in this example
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  */
}

void set_vss_output_frequency(int frequency)
{
  static bool is_pwm_running = false; // Track if PWM is currently running

  if (frequency < 1)
  {
    if (is_pwm_running)
    {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
      mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
      is_pwm_running = false;
    }
  }
  else
  {
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, frequency);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);

    if (!is_pwm_running)
    {
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
      is_pwm_running = true;
    }
  }
}

void setup_esp_now()
{
  // LCD_Data lcdData;
  lcdDataSize = sizeof(lcdData);
  doorDataSize = sizeof(doorData);
  if (esp_now_init() != ESP_OK)
  {
    D_println("Error initializing ESP-NOW");
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    D_println("Failed to add peer");
  }

  esp_now_register_recv_cb(OnDataRecv);
}

long lastRTC_Check = 0;
void send_data_to_lcd(unsigned long current_lcd_print = millis())
{
  // lcd_refresh_rate = 1;
  current_lcd_print = millis();
  if (current_lcd_print - last_lcd_print >= (1000 / lcd_refresh_rate))
  {
    last_lcd_print = current_lcd_print;
    esp_err_t result = ESP_OK;
    // currentTrip = (currentTrip < 1 || currentTrip > totalTrips) ? 0 : currentTrip;
    lcdData.dataType = TYPE_LCD;
    lcdData.gear = currentGear;
    lcdData.converter_lock = converterLockup;
    lcdData.inst_kml = inst_kml;
    lcdData.inst_lph = lph;
    lcdData.totalTrips = totalTrips;
    lcdData.currentTrip = currentTrip;
    lcdData.trip_distance = trips[currentTrip].getDistance();
    lcdData.trip_avg = trips[currentTrip].getAverage();
    lcdData.trip_fuel = trips[currentTrip].getFuel();
    lcdData.trip_price = trips[currentTrip].getFuelPrice();
    lcdData.speed = current_speed;
    lcdData.outSideTemp = outSideTempC;
    result = esp_now_send(broadcastAddress, (uint8_t *)&lcdData, lcdDataSize);

#if DEBUG
    if (result == ESP_OK)
    {
      // D_println("Sent with success");
    }
    else
    {
      D_print("Error sending the data ");
      D_println(esp_err_to_name(result));
    }
#endif
  }
}

bool send_lock_unlock_cmd(byte doorStatus)
{
  esp_err_t result = ESP_OK;
  doorData.dataType = TYPE_DOOR;
  doorData.doorStatus = doorStatus;
  result = esp_now_send(broadcastAddress, (uint8_t *)&doorData, doorDataSize);
  return (result == ESP_OK);
}

unsigned long last_dummy_data = 0;
void dummy_data()
{
  unsigned long current_millis = millis();
  if (current_millis - last_dummy_data >= (500 / lcd_refresh_rate))
  {
    last_dummy_data = current_millis;
    // Generate random data for variables
    current_speed++;

    if (current_speed > 220)
      current_speed = 0;

    lph = lph + 0.1;
    rpm = rpm + 10;

    if (lph > 40)
      lph = 0;
    if (current_speed <= 20)
    {
      currentGear = '1';
      converterLockup = false;
    }
    else if (current_speed <= 80)
    {
      currentGear = '2';
      converterLockup = true;
    }
    else if (current_speed <= 130)
    {
      currentGear = '3';
      converterLockup = false;
    }
    else if (current_speed <= 180)
    {
      currentGear = '4';
      converterLockup = true;
    }
    else
    {
      currentGear = '5';
      converterLockup = true;
    }
    for (int i = 0; i <= totalTrips; i++)
    {
      trips[i].distance = trips[i].distance + 0.01 + i;
      trips[i].fuel = trips[i].fuel + 0.001;
    }

    if (rpm > 5000)
      rpm = 0;
    inst_kml = current_speed / lph;
    return;
    // Output the generated values using custom debug print macros
    D_print("Speed: ");
    D_print(current_speed);
    D_print("    RPM: ");
    D_print(rpm);
    D_print("    currentGear: ");
    D_print(currentGear);
    D_print("   converterLockup: ");
    D_print(converterLockup);
    D_print("   inst_kml: ");
    D_print(inst_kml);
    D_print("   lph: ");
    D_println(lph);
  }
}

void handleUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  if (!index)
  {

#if DEBUG
    Serial.printf("Update Start: %s\n", filename.c_str());
#endif
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len)
  {
    Update.printError(Serial);
  }

  if (final)
  {
    if (Update.end(true))
    {
#if DEBUG
      Serial.printf("Update Success: %uB\n", index + len);
#endif
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
    }
  }
}

String milli2Time(unsigned long totMillis)
{
  // Convert milliseconds to minutes and seconds
  unsigned long minutes = totMillis / (60 * 1000);          // 60,000 milliseconds in a minute
  unsigned long seconds = (totMillis % (60 * 1000)) / 1000; // Remaining milliseconds converted to seconds

  // Create a String to store the result
  String tm = String(minutes);

  // Add a leading zero for single-digit minutes
  if (minutes < 10)
  {
    tm = "0" + tm;
  }

  // Add a colon between minutes and seconds
  tm += ":";

  // Add a leading zero for single-digit seconds
  if (seconds < 10)
  {
    tm += "0";
  }

  // Concatenate the seconds to the result String
  tm += String(seconds);

  // Return the formatted time
  return tm;
}

long lastAccelerationCheck = 0;
void AccelerationCheck(unsigned long currentMillis = millis())
{
#if not ACCELERATION_ON
  return;
#endif
  // unsigned long currentMillis = millis();
  if (currentMillis - lastAccelerationCheck >= 100)
  {
    lastAccelerationCheck = currentMillis;
    mpu.fetchData();
    float x = mpu.getAccX();
    float y = mpu.getAccY();
    float z = mpu.getAccZ();
    float t = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) - 1;
#if ACCELERATION_ON
    g_force = lp.filt(t);
#endif
    // D_println("Acceleration" + String(g_force));
    //  accelerationData.add(t);
    //  g_force = accelerationData.get();
  }
}

void checkCurrentGear(unsigned long currentMillis = millis())
{
  currentMillis = millis();
  if (currentMillis - lastGearCheck >= 200)
  {
    lastGearCheck = currentMillis;

    // a = (REG_READ(GPIO_IN_REG) & A_PIN);         // digitalRead(A_PIN);
    // b = (REG_READ(GPIO_IN_REG) & B_PIN);         // digitalRead(B_PIN);
    // c = (REG_READ(GPIO_IN1_REG) & (C_PIN - 32)); // digitalRead(C_PIN);
    // d = (REG_READ(GPIO_IN1_REG) & (D_PIN - 32)); // digitalRead(D_PIN);

    a = digitalRead(A_PIN);
    b = digitalRead(B_PIN);
    c = digitalRead(C_PIN);
    d = digitalRead(D_PIN);

    if (a == IS_HEIGH && b == IS_HEIGH && c == IS_HEIGH && speed > 5.0f)
    {
      gear_cnt = 0;
      currentGear = '4';
    }
    else if (a == IS_LOW && b == IS_HEIGH && c == IS_HEIGH && speed > 5.0f)
    {
      currentGear = '4'; //'4-5';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_LOW && c == IS_HEIGH && speed > 5.0f)
    {
      currentGear = '3';
      gear_cnt = 0;
    }
    else if (a == IS_HEIGH && b == IS_LOW && c == IS_HEIGH && speed > 5.0f)
    {
      currentGear = '3'; //'3-4';
      gear_cnt = 0;
    }
    else if (a == IS_HEIGH && b == IS_LOW && c == IS_LOW && speed > 5.0f)
    {
      currentGear = '2';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_LOW && c == IS_LOW && speed > 18.0f)
    {
      currentGear = '2'; //'2-3';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_HEIGH && c == IS_LOW && speed > 42.0f)
    {
      currentGear = '5';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_HEIGH && c == IS_LOW)
    {
      currentGear = '1';
      gear_cnt = 0;
    }
    else if (a == IS_HEIGH && b == IS_HEIGH && c == IS_LOW)
    {
      currentGear = '1'; //'1-2';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_LOW && c == IS_LOW && d == IS_LOW)
    {
      gear_cnt++;
      if (gear_cnt > 5)
      {
        currentGear = 'N';
        gear_cnt = 0;
      }
    }
    else if (a == IS_LOW && b == IS_LOW && c == IS_LOW && d == IS_HEIGH)
    {
      currentGear = 'P';
      gear_cnt = 0;
    }
    else if (a == IS_LOW && b == IS_HEIGH && c == IS_HEIGH && d == IS_HEIGH)
    {
      currentGear = 'R';
      gear_cnt = 0;
    }

    if (d == IS_HEIGH)
    {
      converterLockup = true;
    }
    else
    {
      converterLockup = false;
    }
    /*D_print("a: ");
    D_print(a);
    D_print("    b: ");
    D_print(b);
    D_print("    c: ");
    D_print(c);
    D_print("    d: ");
    D_print(d);
    D_print("    currentGear: ");
    D_println(currentGear);*/
  }
}

void checkConsumption(unsigned long currentMillis = millis())
{
  currentMillis = millis();
  if (currentMillis - lastConsumptionCheck >= dataUpdateInterval)
  {
    // float deltaTime = (currentMillis - lastConsumptionCheck) / 1000.0;
    lastConsumptionCheck = currentMillis;
    current_speed = trip.getKmh();
    rpm = trip.getRpm(); // trip.getRpmCrank();
    lph = trip.getLph();
    speed = round(current_speed);
    inst_kml = (speed > 0 && lph > 0) ? (speed / lph) : 0;

    /*float speedChange = current_speed - prvSpeed;
    prvSpeed = current_speed;
    acceleration = speedChange / deltaTime;
    if (acceleration == 0)
      g_force = 0;
    else
      g_force = acceleration / 9.81;*/
    float _km = trip.getKm();
    float _ltr = trip.getLiters();

    for (int i = 0; i <= totalTrips; i++)
    {
      trips[i].distance = _km;
      trips[i].fuel = _ltr;
    }
    vss1_pulses = trip.getVel();
    if (checkConsumptionWeb)
    {
      checkConsumptionWeb = false;
      injectorRPM = trip.getRpm();
      trip_totalInjectionMs = trip.getTotalInjectionMs();
      inj_timing = trip.getInjectionTime();
      inj_freq = trip.getInjFreq();
      duty_cycle = (trip.getDuty() * 100);
    }

    if (engineStartTime == 0 && rpm > 500)
    {
      engineStartTime = millis();
      isTripNeedToSave = true;
    }
    else if (engineStartTime != 0 && rpm < 300)
    {
      engineEndTime = millis();
      engineRunningTime = engineRunningTime + engineEndTime - engineStartTime;
      // engineRunningTime = engineEndTime - engineStartTime;
      engineStartTime = 0;

      if (isTripNeedToSave)
      {
        isTripNeedToSave = false;
        preferences.begin("zApp", false);
        for (int i = 0; i <= totalTrips; i++)
        {
          String fuelKey = "trip" + String(i) + "_fuel"; // Generate key dynamically
          String distanceKey = "trip" + String(i) + "_distance";
          preferences.putFloat(fuelKey.c_str(), trips[i].getFuel());
          preferences.putFloat(distanceKey.c_str(), trips[i].getDistance());
        }
        //  DateTime now = rtc.now();
        //  D_print("Saving last Time: ");
        //  D_println(current_time);
        preferences.putULong("last_timestamp", current_time);
        preferences.end();
      }
    }
  }
}

String getData()
{
  readings["gear"] = String(currentGear);
  readings["rpm2"] = String(injectorRPM);
  readings["rpm"] = String(rpm);
  readings["speed"] = String(speed);

  readings["inj"] = String(inj_timing);
  readings["duty_cycle"] = String(duty_cycle, 4);
  readings["inj_freq"] = String(inj_freq);
  readings["lph"] = String(lph, 2);
  readings["kml"] = String(inst_kml, 2);

  readings["con_loc"] = String(converterLockup);
  readings["vss1"] = String(vss1_pulses);
  readings["relay"] = String(relay_status);
  readings["o2_val"] = String(o2_val);
  readings["g_force"] = String(g_force, 2);
  readings["A"] = String(a);
  readings["B"] = String(b);
  readings["C"] = String(c);
  readings["D"] = String(d);
  readings["run_time"] = milli2Time(engineRunningTime);
  readings["vss_spoof"] = String(is_spoofing);
  readings["spoof_per"] = String(spoof_per);
  readings["totalInjectionUs"] = String(trip_totalInjectionMs);
  readings["last_timestamp"] = String(last_timestamp);

  for (int i = 0; i <= totalTrips; i++)
  {
    String fuelKey = "trip" + String(i) + "_fuel";
    String distanceKey = "trip" + String(i) + "_distance";
    String savedFuelKey = "trip" + String(i) + "_fuel";
    String savedDistanceKey = "trip" + String(i) + "_distance";

    readings["trip" + String(i) + "_distance"] = String(trips[i].getDistance(), 4);
    readings["trip" + String(i) + "_fuel"] = String(trips[i].getFuel(), 4);
    readings["trip" + String(i) + "_fuel_price"] = String(trips[i].getFuelPrice());
    readings["trip" + String(i) + "avg"] = String(trips[i].getAverage(), 2);

    readings["trip" + String(i) + "_sDistance"] = String(trips[i].savedDistance);
    readings["trip" + String(i) + "_sFuel"] = String(trips[i].savedFuel, 4);
  }
  jsonData = JSON.stringify(readings);

  return jsonData;
}

void SpoofSpeed(unsigned long currentMillis = millis())
{

  currentMillis = millis();
  if (currentMillis - lastSpeedSpoof > 4)
  {
    lastSpeedSpoof = currentMillis;

    if (!is_spoofing && vss_relay_status)
    {
      vss_relay_status = false;
      digitalWrite(VSS_RELAY_PIN, vss_relay_status);
      return;
    }

    if (vss1_pulses < 300 && !vss_relay_status)
    {
      vss_relay_status = false;
      digitalWrite(VSS_RELAY_PIN, vss_relay_status);
      return;
    }

    if (
        abs(spoof_per) > 25 &&
        ((spoof_gear == 1 && speed < 10) ||
         (spoof_gear == 2 && speed < 15) ||
         (spoof_gear == 3 && speed < 20) ||
         (spoof_gear == 4 && speed < 30) ||
         (spoof_gear == 5 && speed < 40)))
    {
      vss_relay_status = false;
      digitalWrite(VSS_RELAY_PIN, vss_relay_status);
      return;
    }

    int VSS_PLUS = vss1_pulses * (1 + spoof_per / 100.0);
    int delta = abs(oldSpoofValue - VSS_PLUS);
    // if (delta > 9 && delta < 10000)
    if (oldSpoofValue != VSS_PLUS)
    {
      oldSpoofValue = VSS_PLUS;
      set_vss_output_frequency(VSS_PLUS);
    }

    if (vss_relay_status != is_spoofing)
    {
      vss_relay_status = is_spoofing;
      digitalWrite(VSS_RELAY_PIN, is_spoofing);
    }
  }
}

void setupWIFI()
{
  const char *ap_ssid = "Reborn";
  const char *ap_password = "Z.12345678";
  IPAddress customIP(192, 168, 4, 1);

  // const char *ssid = "Logout";
  // const char *password = "ms120400427";

  WiFi.mode(WIFI_AP_STA);
  D_print("MAC Address");
  D_println(WiFi.macAddress());
  WiFi.softAPConfig(customIP, customIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ap_ssid, ap_password, 1);
  WiFi.hostname(String("Reborn").c_str());
  dnsServer.start(53, "*", customIP);
  // WiFi.begin(ssid, password);
}

void setupServer()
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_MAIN_PAGE, null); });
  server.on("/car_door", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_DOOR_PAGE, null); });

  server.on("/dash", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_DASH_BOARD, null); });

  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_JSON_PAGE, null); });
  server.on("/export", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_EXPORT_PAGE, null); });

  server.on("/getdata", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // String json = getData();
              updateJson = true;
              checkConsumptionWeb = true;
              request->send(200, "application/json", jsonData);
              // json = String();
            });

  server.on("/update_o2", HTTP_GET, [](AsyncWebServerRequest *request)
            {
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
      request->send(200, "text/html", String(o2_val)); });

  server.on("/update_spoof", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("spoof_val")) {
        int val = request->arg("spoof_val").toInt();
        if (val > 0 && val<= 100)
        {
          spoof_per = val;
          preferences.begin("zApp", false);
          preferences.putInt("spoof_per", spoof_per);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(spoof_per)); });

  server.on("/relay_on", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      relay_status = true;
      digitalWrite(RELAY_PIN, relay_status);
      preferences.begin("zApp", false);
      preferences.putBool("RELAY_STATUS", relay_status);
      preferences.end();
      request->send(200, "text/html", "OK"); });

  server.on("/relay_off", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      relay_status = false;
      digitalWrite(RELAY_PIN, relay_status);
      preferences.begin("zApp", false);
      preferences.putBool("RELAY_STATUS", relay_status);
      preferences.end();
      request->send(200, "text/html", "OK"); });

  server.on("/spoof_on", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      is_spoofing = true;
      preferences.begin("zApp", false);
      preferences.putBool("is_spoofing", is_spoofing);
      preferences.end();
      request->send(200, "text/html", "Spoof On"); });

  server.on("/spoof_off", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      is_spoofing = false;
      preferences.begin("zApp", false);
      preferences.putBool("is_spoofing", is_spoofing);
      preferences.end();
      request->send(200, "text/html", "Spoof Of"); });

  server.on("/update_speed", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        int Speed = request->arg("val").toInt();
        if (Speed > 0) {
          trip.VSS_PULSE_KM = 3600 / Speed * vss1_pulses;
          preferences.begin("zApp", false);
          preferences.putFloat("VSS_PULSE_KM", trip.VSS_PULSE_KM);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(trip.VSS_PULSE_KM, 2)); });

  server.on("/update_speed_raw", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        int raw = request->arg("val").toFloat();
        if (raw > 0) {
          trip.VSS_PULSE_KM = raw;
          preferences.begin("zApp", false);
          preferences.putFloat("VSS_PULSE_KM", trip.VSS_PULSE_KM);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(trip.VSS_PULSE_KM, 2)); });

  server.on("/update_fuel", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        float InstantLph = request->arg("val").toFloat();
        if (InstantLph > 0) {
          trip.INJ_USEC_LITER = 3600000000.0f / InstantLph * trip.getDuty();// /100;
          preferences.begin("zApp", false);
          preferences.putFloat("INJ_USEC_LITER", trip.INJ_USEC_LITER);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(trip.INJ_USEC_LITER, 2)); });

  server.on("/update_fuel_raw", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        float raw = request->arg("val").toFloat();
        if (raw > 0) {
          trip.INJ_USEC_LITER = raw;
          preferences.begin("zApp", false);
          preferences.putFloat("INJ_USEC_LITER", trip.INJ_USEC_LITER);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(trip.INJ_USEC_LITER, 2)); });

  server.on("/update_fuel_ltr", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        float fuelLtr = request->arg("val").toFloat();
        if (fuelLtr > 0)
        {
          float reportedFuel = trips[0].getFuel();
          if (reportedFuel > 0){
            // You know actual fuel used manually, hardcoded for one-time calibration
          trip.INJ_USEC_LITER = trip.INJ_USEC_LITER * (reportedFuel / fuelLtr);
          preferences.begin("zApp", false);
          preferences.putFloat("INJ_USEC_LITER", trip.INJ_USEC_LITER);
          preferences.end();
          }
        }
      }
      request->send(200, "text/html", String(trip.INJ_USEC_LITER, 2)); });

  server.on("/lcd_refresh_rate", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        int val = request->arg("val").toInt();
        if (val > 0) {
          lcd_refresh_rate = val;
          preferences.begin("zApp", false);
          preferences.putInt("lcd_ref_rate", lcd_refresh_rate);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(lcd_refresh_rate)); });

  server.on("/trip_reset_min", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        int val = request->arg("val").toInt();
        if (val >= 0) {
          trip_reset_min = val;
          preferences.begin("zApp", false);
          preferences.putInt("trip_reset_min", trip_reset_min);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(trip_reset_min)); });

  server.on("/fuel_price", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        int val = request->arg("val").toInt();
        if (val > 0) {
          global_fuel_price = val;
          preferences.begin("zApp", false);
          preferences.putInt("fuel_price", global_fuel_price);
          preferences.end();
        }
      }
      request->send(200, "text/html", String(global_fuel_price)); });

  server.on("/spoof_gear", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              request->send(200, "text/html", "0");
              return;
      if (request->hasArg("gear")) {
        int gear = request->arg("gear").toInt();

        if(!is_spoofing)return;
        if (gear > 0) {
          spoof_gear = gear;
          if(spoof_gear == 1){
            //ledcWriteTone(0, 300);
            set_vss_output_frequency(300);
          }else if(spoof_gear == 2){
  //ledcWriteTone(0, 750);
  set_vss_output_frequency(750);
          }
          else if(spoof_gear == 3){
  //ledcWriteTone(0, 1050 );
  set_vss_output_frequency(1050);
          }
          else if(spoof_gear == 4){
  //ledcWriteTone(0,1550);
  set_vss_output_frequency(1500);
          }
          else if(spoof_gear == 5){
  //ledcWriteTone(0, 2500);
  set_vss_output_frequency(2500);
          }
        } else{
          spoof_gear = 0;
          //ledcWriteTone(0, vss1_pulses);
          set_vss_output_frequency(vss1_pulses);
        }
      }
      request->send(200, "text/html", String(trip.VSS_PULSE_KM, 2)); });

  server.on("/reset_trip", HTTP_GET, [](AsyncWebServerRequest *request)
            {
          resetTrip(0);
          request->send(200, "text/html", "OK"); });

  server.on("/ota_update", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", serverIndex); });

  server.on(
      "/ota_update", HTTP_POST, [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", "Updated !!!"); },
      handleUpdate);

  server.on(
      "/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "image/x-icon", "X"); });

  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->redirect("/"); });

  server.on("/synchronize_time", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (request->hasArg("val")) {
        long val = request->arg("val").toDouble();
        
        if (val > 0) {
          DateTime dt = DateTime(val);
          if(isRTC_Connected){
             rtc.adjust(dt);
          }
        }
      }
      request->send(200, "text/html", "OK"); });

  server.on("/lock", HTTP_GET, [](AsyncWebServerRequest *request)
            { bool isOk = send_lock_unlock_cmd(1);
              if(isOk)
              request->send(200, "text/html", "OK"); 
              else
              request->send(200, "text/html", "Error"); });
  server.on("/unlock", HTTP_GET, [](AsyncWebServerRequest *request)
            { bool isOk = send_lock_unlock_cmd(2);
              if(isOk)
              request->send(200, "text/html", "OK"); 
              else
              request->send(200, "text/html", "Error"); });

  server.begin();
}

void setupRTC()
{
  isRTC_Connected = rtc.begin();
  if (!isRTC_Connected)
  {

    D_println("RTC module is NOT found");
  }
}

void dummySetting()
{
  preferences.begin("zApp", false);
  preferences.putFloat("trip2_fuel", 10.0);
  preferences.putFloat("trip2_distance", 100.0);
  preferences.putFloat("trip1_fuel", 5.0);
  preferences.putFloat("trip1_distance", 50.0);
  preferences.putFloat("INJ_USEC_LITER", 1000);
  preferences.putFloat("VSS_PULSE_KM", 500);
  preferences.putULong("last_timestamp", 234233);
  preferences.end();
  while (true)
  {
    delay(1000);
  }
}
void readSetting()
{
  D_println("readSetting");
  // dummySetting();
  preferences.begin("zApp", false);
  trip.INJ_USEC_LITER = preferences.getFloat("INJ_USEC_LITER", INJ_USEC_LITER_D);
  trip.VSS_PULSE_KM = preferences.getFloat("VSS_PULSE_KM", VSS_PULSE_KM_D);
  relay_status = preferences.getBool("RELAY_STATUS", true);
  o2_val = preferences.getInt("O2_VAL", 62);
  lcd_refresh_rate = preferences.getInt("lcd_ref_rate", 5);
  trip_reset_min = preferences.getInt("trip_reset_min", 0);
  spoof_per = preferences.getInt("spoof_per", 0);
  is_spoofing = preferences.getBool("is_spoofing", 0);
  last_timestamp = preferences.getULong("last_timestamp", 0);
  global_fuel_price = preferences.getInt("fuel_price", 0);
  totalTrips = preferences.getInt("total_trip", 2);
  currentTrip = preferences.getInt("currentTrip", 0);

  totalTrips = (totalTrips < 1 || totalTrips > maxTrips) ? 2 : totalTrips;
  currentTrip = (currentTrip < 1 || currentTrip > totalTrips) ? 0 : currentTrip;

  for (int i = 0; i <= totalTrips; i++)
  {
    String fuelKey = "trip" + String(i) + "_fuel"; // Generate key dynamically
    String distanceKey = "trip" + String(i) + "_distance";
    trips[i].savedFuel = preferences.getFloat(fuelKey.c_str(), 0);
    trips[i].savedDistance = preferences.getFloat(distanceKey.c_str(), 0);
    trips[i].distance = 0.0;
    trips[i].fuel = 0.0;
    D_print(i);
    D_print("   => saved_fuel: ");
    D_print(trips[i].savedFuel);
    D_print("   saved_distance: ");
    D_println(trips[i].savedDistance);
  }

  DateTime lastDateTime = DateTime(last_timestamp);
  D_print("last_timestamp: ");
  D_println(last_timestamp);
  D_print("lastDateTime: ");
  printTime(lastDateTime);

  D_print("isRTC_Connected: ");
  D_println(isRTC_Connected);
  int diffMinutes = 0;
  DateTime now;
  if (isRTC_Connected)
  {
    now = rtc.now();
    long diffSeconds = now.unixtime() - lastDateTime.unixtime();
    diffMinutes = diffSeconds / 60;
  }
  // D_print("last_timestamp: ");
  // D_println(last_timestamp);

  D_print("diffMinutes: ");
  D_println(diffMinutes);
  if (diffMinutes > trip_reset_min && trip_reset_min > 0 && isRTC_Connected)
  {
    D_print("Inside if");
    D_println(now.unixtime());
    trips[0].savedFuel = 0;
    trips[0].savedDistance = 0;

    preferences.putFloat("trip0_fuel", trips[0].savedFuel);
    preferences.putFloat("trip0_distance", trips[0].savedDistance);
    if (isRTC_Connected)
    {
      preferences.putULong("last_timestamp", now.unixtime());
    }
  }
  preferences.end();
  // trip2_avg = trip2_saved_distance / trip2_saved_fuel;
  // if (isnan(trip2_avg) || isinf(trip2_avg))
  //{
  //   trip2_avg = 0;
  // }
}

void setupAccelerationSensor()
{
#if not ACCELERATION_ON
  return;
#endif
  Wire.begin();
  isGyroConnected = mpu.begin();
  D_print(F("MPU6050 status: "));
  D_println(isGyroConnected);

  if (isGyroConnected)
  {
    mpu.setAccConfig(0);
    mpu.calcOffsets(true, true); // gyro and accelero
    // accelerationData.begin(SMOOTHED_AVERAGE, 10);
    // accelerationData.begin(SMOOTHED_EXPONENTIAL, 10);
    // accelerationData.clear();
  }
}

void setupPins()
{
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(C_PIN, INPUT_PULLUP);
  pinMode(D_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(VSS_RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, relay_status);
  digitalWrite(VSS_RELAY_PIN, vss_relay_status);

  dacWrite(DAC_CH1, o2_val);
}

void checkTime(long ms = millis())
{
  if (!isRTC_Connected)
    return;

  ms = millis();
  if (ms - lastRTC_Check > 999)
  {

    lastRTC_Check = ms;
    printTime(rtc.now());
    long _current_time = rtc.now().unixtime();
    if (_current_time > 0)
    {
      current_time = _current_time;
    }
  }
}
int f = 10;
void SpeedoMeterCheck()
{
  digitalWrite(VSS_RELAY_PIN, true);
  set_vss_output_frequency(f);
  f++;
  if (f > 8000)
  {
    f = 10;
  }
  delay(10);
}

void checkTemp(long ms = millis())
{
  static unsigned long lastRequestTime = 0;
  static bool waitingForConversion = false;
  const unsigned long conversionDelay = 1000; // ms for 12-bit resolution
  if (!waitingForConversion && (ms - lastRequestTime > 1500))
  {
    // Step 1: Request conversion without waiting
    tempSensor.setWaitForConversion(false);
    tempSensor.requestTemperatures();
    lastRequestTime = ms;
    waitingForConversion = true;
  }
  if (waitingForConversion && (ms - lastRequestTime >= conversionDelay))
  {
    /// if (tempSensor.getDeviceCount() > 0)
    {
      // Step 2: Enough time has passed, now read the temperature
      D_print("outSideTemp: ");
      outSideTempC = tempSensor.getTempCByIndex(0);
      D_print(outSideTempC);
      D_println(" C");
    }
    waitingForConversion = false; // Reset for next cycle
  }
}

void setup()
{
#if RESET_FLASH
  resetFlash();
#endif
  unsigned long startTime = millis();
  D_SerialBegin(115200);
  delay(10);
  mainLoopCount = 0;
  setupPins();
  setupRTC();
  delay(30);
  readSetting();
  setupWIFI();
  setupServer();
  setupAccelerationSensor();
  setup_esp_now();
  setup_vss_output();
  tempSensor.begin();
  // tempSensor.setResolution(0, 12); // 9-bit resolution for sensor at index 0
  trip.begin();
  /*
  while (1)
  {
    digitalWrite(VSS_RELAY_PIN, true);
    set_vss_output_frequency(1000); --35 meter
    delay(5000);
    set_vss_output_frequency(2000);--67 meter
    delay(5000);
    set_vss_output_frequency(3000);--100 meter
    delay(5000);
  }*/

  // randomSeed(analogRead(0));
  unsigned long endTime = millis();
  unsigned long setupTime = endTime - startTime;

  D_print("Setup done in: ");
  D_print(setupTime);
  D_println(" milliseconds");
}

void loop()
{
  // SpeedoMeterCheck();
  // return;
  mainLoopCount++;
  unsigned long ms = millis();
  dnsServer.processNextRequest();
  // #ifndef DUMMY_DATA
  checkCurrentGear(ms);
  checkConsumption(ms);
  AccelerationCheck(ms);
  checkTemp(ms);
  // #endif

  // #if DUMMY_DATA
  //  dummy_data();
  // #endif
  send_data_to_lcd(ms);
  checkTime(ms);
  // SpoofSpeed(ms);
  if (updateJson)
  {
    updateJson = false;
    jsonData = getData();
  }

  //   Allow the ESP32 to handle interrupts and other tasks
  delay(0);
}
