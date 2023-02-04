/***
 * ESP32 weather monitor application using a 3 colour Waveshare 4.2" e-ink display. Weather
 * data is obtained from Open Weather Map
 * 
 * Application was written using VSCode and platformio to manage the project.  Graphics library used
 * for the e-ink display is GxEPD2.
 * 
 * v1: Near copy of project by G6EJD.
 * v2: Modify display to remove weather person and re-arrange information.
 * 
 * This is a copy/update of the weather display written by G6EJD:
 *  https://github.com/G6EJD/ESP32-Revised-Weather-Display-42-E-Paper
*/
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "GxEPD2_GFX.h"
#include "GxEPD2_3C.h"                            // 3 colour screen
#include "GxEPD2_display_selection_new_style.h"   // For selecting screen

#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include "config.h"
#include "fonts.h"
#include "arrow.h"
#include "sunrise.h"
#include "sunset.h"

/* Constants/defines */
const uint SCREEN_WIDTH = 400;
const uint SCREEN_HEIGHT = 300;
const String VERSION = "v3";
const String Hemisphere = "north";
const int forecast_counter = 24;         // number of forecasts to get/show.

#define LARGE  10
#define SMALL  4

boolean large_icon = true;
boolean small_icon = false;

enum alignment {LEFT, RIGHT, CENTER};
// enum pressure_trend {LEVEL, UP, DOWN};
enum sun_direction {SUN_UP, SUN_DOWN};
enum star_size {SMALL_STAR, MEDIUM_STAR, LARGE_STAR};

/* Function prototypes */
bool getTodaysWeather(void);
bool getWeatherForecast(void);
bool getDailyWeatherForecast(void);
static void updateLocalTime(void);
void initialiseDisplay(void);
void displayErrorMessage(String message);
void displaySystemInfo(int x, int y);
void drawString(int x, int y, String text, alignment align);
void displayInformation(bool today_flag, bool forecast_flag);
void displayTemperature(int x, int y);
void displayCloudCover(int x, int y, int cover);
void addCloud(int x, int y, int scale, int linesize);
void displayHeader(void);
void displayWind(int x, int y, float angle, float windspeed, int radius);
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength, uint16_t colour);
String windDegToDirection(float winddirection);
void drawStringMaxWidth(int x, int y, unsigned int text_width, String text, alignment align);
String titleCase(String text);
void displayWeatherDescription(int x, int y);
void displayMoonPhase(int x, int y);
void displayRain(int x, int y);
String convertUnixTime(uint32_t unix_time);
double normalizedMoonPhase(int d, int m, int y);
void drawMoon(int x, int y, int dd, int mm, int yy, String hemisphere, const int diameter, bool surface);
void displaySunAndMoon(int x, int y);
void moonPhase(int x, int y, int day, int month, int year, String hemisphere);
int julianDate(int d, int m, int y);
void displayWeatherForecast(int x, int y);
void displaySingleForecast(int x, int y, int offset, int index);
void displayWeatherIcon(int x, int y, String icon, bool icon_size);
void addMoon (int x, int y, int scale);
void addSun(int x, int y, int scale, boolean icon_size, uint16_t icon_color);
void noData(int x, int y, bool large_size);
void sunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
void mostlySunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
void cloudyIcon(int x, int y, bool large_size, String icon_name);
void veryCloudyIcon(int x, int y, bool large_size, String icon_name);
void chanceOfRainIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
void rainIcon(int x, int y, bool large_size, String icon_name);
void thunderStormIcon(int x, int y, bool large_size, String icon_name);
void snowIcon(int x, int y, bool large_size, String icon_name);
void fogIcon(int x, int y, bool large_size, String icon_name);
void hazeIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
void sunRiseSetIcon(uint16_t x, uint16_t y, sun_direction direction);
void addCloud(int x, int y, int scale, int linesize);
void addRain(int x, int y, int scale, uint16_t colour);
void addSnow(int x, int y, int scale, uint16_t colour);
void addThunderStorm(int x, int y, int scale, uint16_t colour);
void addFog(int x, int y, int scale, int linesize, uint16_t colour);
void addStar(int x, int y, star_size starsize);
void drawGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h, float Data[], float Data2[], int len, String title);
void drawSingleGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h, float Data[], int len, String title);

/* Globals etc. */
WiFiClientSecure wifiClient;

String ipAddress = "0:0:0:0";
int rssi = 0;

// current
typedef struct WeatherStruct {
  // pressure_trend   trend = LEVEL;
  uint8_t  humidity = 0;
  uint8_t  clouds = 0;
  uint16_t wind_deg = 0;
  uint32_t dt = 0;
  uint32_t sunrise = 0;
  uint32_t sunset = 0;
  uint32_t visibility = 0;
  float    temperature = 0;
  float    high = 0;
  float    low = 0;
  float    feels_like = 0;
  float    pressure = 0;
  float    dew_point = 0;
  float    uvi = 0;
  float    wind_speed = 0;
  float    wind_gust = 0;
  float    rain = 0;
  float    snow = 0;
  String   main;
  String   description;
  String   icon;
  String   period;
} WeatherStruct;

WeatherStruct weather;
WeatherStruct forecast[forecast_counter];

static portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *My_timer = NULL;    // Hardware timer for updating clock etc.
volatile bool update = false;   // Tinmer update flag
char timeStringBuff[35];        // buffer for time on the display
char dateStringBuff[35];
char dayStringBuff[10];
// timer interrupt routine
void IRAM_ATTR onTimerInterrupt() {
  portENTER_CRITICAL_ISR(&myMux);
  update = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

void setup() {
  Serial.begin(115200);

  initialiseDisplay();

  Serial.println("\n##################################");
  Serial.println(F("ESP32 Information:"));
  Serial.printf("Internal Total Heap %d, Internal Used Heap %d, Internal Free Heap %d\n", ESP.getHeapSize(), ESP.getHeapSize()-ESP.getFreeHeap(), ESP.getFreeHeap());
  Serial.printf("Sketch Size %d, Free Sketch Space %d\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Serial.printf("Chip Model %s, ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  Serial.println("##################################\n");

  WiFi.begin(SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connecting to Wi-Fi...");
  ipAddress = WiFi.localIP().toString();
  rssi = WiFi.RSSI();
  Serial.println(ipAddress);

  Serial.println("Connecting to NTP Time Server...");
  configTime(0, 3600, SNTP_TIME_SERVER);
  updateLocalTime();

  Serial.println("Setting up hardware timer...");

  // Set up hardware timer
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimerInterrupt, true);
  timerAlarmWrite(My_timer, 1000000 * 60 * 30, true);            // every 30 minutes (* 60 * 30)  
//  timerAlarmWrite(My_timer, 1000000, true);                 // every second
  timerAlarmEnable(My_timer);

  Serial.println("All set up, display some information...");
  bool today_flag = getTodaysWeather();
  bool forecast_flag = getWeatherForecast();
  if (today_flag == true || forecast_flag == true) {
    displayInformation(today_flag, forecast_flag);
  }  
}



void loop() {
  // Timer to update weather information
  if (update) {
    // update flag
    portENTER_CRITICAL(&myMux);
    update = false;
    portEXIT_CRITICAL(&myMux);

    updateLocalTime();

    bool today_flag = getTodaysWeather();
    bool forecast_flag = getWeatherForecast();
    if (today_flag == true || forecast_flag == true) {
      displayInformation(today_flag, forecast_flag);
    } else {
      displayErrorMessage("Unable to retrieve the weather!");
    }
  }
}


static void updateLocalTime(void) 
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  // Update buffer with current time/date
  strftime(dateStringBuff, sizeof(dateStringBuff), "%d", &timeinfo);      // Sat 15-Jan-23
  strftime(dayStringBuff, sizeof(dayStringBuff), "%A", &timeinfo);      // Saturday
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);          // 15:15
}

bool getTodaysWeather(void)
{
  WiFiClientSecure client;
  bool retcode = true;
  int port = 443;
  const char* host = "api.openweathermap.org";

  uint32_t dt = millis();

  client.setInsecure();     // certificate is not checked

  if (!client.connect(host, port)) {
    Serial.println("HTTPS connection to OpenWeatherMap failed!");
    return false;
  } 
  
  uint32_t timeout = millis();
  char c = 0;

  // Send GET request
  Serial.printf("Sending GET request to %s, port %d\n", host, port);
  client.print(String("GET ") + WEATHER_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("Header end found.");
      break;
    }

    Serial.println(line);

    if ((millis() - timeout) > 5000UL)
    {
      Serial.println("HTTP header timeout");
      client.stop();
      return false;
    }
  }

//  Serial.print("JSON length: "); Serial.println(client.available());
  Serial.println("Parsing JSON...");

  // bool decode = false;
  DynamicJsonDocument doc(20*1024);

  Serial.println("Deserialization process starting...");
  // Parse JSON object
  DeserializationError err = deserializeJson(doc, client);
  if (err) { 
    Serial.print("deserializeJson() failed: ");
    Serial.println(err.c_str());
    retcode =  false;
  } else {
    Serial.print(F("Deserialization succeeded, decoding data..."));

    weather.main = doc["weather"][0]["main"].as<String>();
    weather.description = doc["weather"][0]["description"].as<String>();
    weather.icon = doc["weather"][0]["icon"].as<String>();
    weather.temperature = doc["main"]["temp"];
    weather.high = doc["main"]["temp_max"];
    weather.low = doc["main"]["temp_min"];
    weather.feels_like = doc["main"]["feels_like"];
    weather.pressure = doc["main"]["pressure"];
    weather.humidity = doc["main"]["humidity"];
    weather.wind_speed = doc["wind"]["speed"];
    weather.wind_deg = doc["wind"]["deg"];
    weather.wind_gust = doc["wind"]["gust"];
    weather.sunrise = doc["sys"]["sunrise"];
    weather.sunset = doc["sys"]["sunset"];
    weather.visibility = doc["visibility"];
    weather.clouds = doc["clouds"]["all"];

    Serial.print("\nDone in "); Serial.print(millis()-dt); Serial.println(" ms");
  }

  client.stop();    

  return retcode;
}

bool getWeatherForecast(void)
{
  WiFiClientSecure client;
  bool retcode = true;
  int port = 443;
  const char* host = "api.openweathermap.org";

  uint32_t dt = millis();

  client.setInsecure();     // certificate is not checked

  if (!client.connect(host, port)) {
    Serial.println("HTTPS connection to OpenWeatherMap failed!");
    return false;
  } 
  
  uint32_t timeout = millis();
  char c = 0;

  // Send GET request
  Serial.printf("Sending GET request to %s, port %d\n", host, port);
  client.print(String("GET ") + FORECAST_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("Header end found.");
      break;
    }

    if ((millis() - timeout) > 5000UL)
    {
      Serial.println("HTTP header timeout");
      client.stop();
      return false;
    }
  }

  Serial.println("Parsing Forecast JSON...");

  DynamicJsonDocument doc(24*1024);

  Serial.println("Deserialization process starting...");

  // Parse JSON object
  DeserializationError err = deserializeJson(doc, client);
  if (err) { 
    Serial.print("deserializeJson() failed: ");
    Serial.println(err.c_str());
    retcode =  false;
  } else {
    Serial.println(F("Deserialization succeeded, decoding forecast data..."));

    for (byte i = 0; i < forecast_counter; i++) {
      forecast[i].dt = doc["list"][i]["dt"];
      forecast[i].temperature = doc["list"][i]["main"]["temp"];
      forecast[i].feels_like = doc["list"][i]["main"]["feels_like"];
      forecast[i].icon = doc["list"][i]["weather"][0]["icon"].as<String>();
      forecast[i].main = doc["list"][i]["weather"][0]["main"].as<String>();
      forecast[i].description = doc["list"][i]["weather"][0]["description"].as<String>();
      forecast[i].low = doc["list"][i]["main"]["temp_min"];
      forecast[i].high = doc["list"][i]["main"]["temp_max"];
      forecast[i].pressure = doc["list"][i]["main"]["pressure"];
      forecast[i].humidity = doc["list"][i]["main"]["humidity"];
      forecast[i].clouds = doc["list"][i]["clouds"]["all"];
      forecast[i].wind_speed = doc["list"][i]["wind"]["speed"];
      forecast[i].wind_deg = doc["list"][i]["wind"]["deg"];
      forecast[i].rain = doc["list"][i]["rain"]["3h"];
      forecast[i].snow = doc["list"][i]["snow"]["3h"];
      forecast[i].period = doc["list"][i]["dt_txt"].as<String>();      
    }

    Serial.println("##Still need to check on rainfall and snowfall returns!##");
    Serial.print("\nDone in "); Serial.print(millis()-dt); Serial.println(" ms");
  }

  client.stop();    

  return retcode;
}

void initialiseDisplay() {
  display.init(115200, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse
  display.setRotation(0);
  display.setTextSize(0);
  display.setFont(&DejaVu_Sans_Bold_11);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  display.firstPage();
  display.hibernate();
  delay(1000);
}

void displayErrorMessage(String message) {
  display.setFullWindow();
  display.firstPage();

  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 60);
    display.printf("Error: %s", message);
   } while (display.nextPage());

   display.hibernate();
}

void displayInformation(bool today_flag, bool forecast_flag) {
  uint32_t dt = millis();

  display.setFullWindow();
  display.firstPage();
  do {
    displayHeader();
    displayWeatherIcon(120, -15, weather.icon, large_icon); // Weather icon
    displayWeatherDescription(185, 90);                    // Description of the weather now
    displayTemperature(270, 0);

    display.drawLine(125, 110, 395, 110, GxEPD_BLACK);
    display.drawLine(5, 180, 115, 180, GxEPD_BLACK);
    display.drawLine(125, 180, 395, 180, GxEPD_BLACK);

    displayWind(48, 225, weather.wind_deg, weather.wind_speed, 44); // Wind direction info
    displayWeatherForecast(118, 115);                       // Forecast

    displaySunAndMoon(0, 120);                            // Sunset and sunrise and moon state with icons
  } while (display.nextPage());
  
  display.hibernate();

  Serial.print("\nDisplay updated in "); Serial.print((millis()-dt)/1000); Serial.println(" secs");
}

void displayHeader(void) {
  display.fillRect(0, 0, 120, 120, GxEPD_BLACK);
  display.setTextColor(GxEPD_WHITE);

  display.setFont();
  drawString(3, -6, VERSION, LEFT);
  drawString(45, -6, timeStringBuff, CENTER);

  drawString(117,-6, String(rssi) + "dBm", RIGHT);
    
  display.setFont(&DSEG7_Classic_Bold_21);
  display.setTextSize(2);
  drawString(24, 30, dateStringBuff, LEFT);

  display.setTextSize(0);
  display.setFont(&FreeMonoBold9pt7b);
  drawString(60, 92, dayStringBuff, CENTER);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&DejaVu_Sans_Bold_11);
}

void displayTemperature(int x, int y) {
  display.setFont();
  drawString(x - 10, y, "Min", LEFT);
  drawString(x + 103, y, "Max", LEFT);
  drawString(x - 10, y + 46, "Humidity", LEFT);
  drawString(x + 88, y + 46, "Clouds", LEFT);
  drawString(x + 30, y + 75, "Pressure", LEFT);

  display.setFont(&DejaVu_Sans_Bold_11);
  display.setTextSize(1);
  drawString(x - 15, y + 20, String(weather.low, 0) + "'", LEFT);
  drawString(x + 123, y + 20, String(weather.high, 0) + "'", RIGHT);
  drawString(x - 9, y + 66, String(weather.humidity) + "%", LEFT);
  drawString(x + 123, y + 66, String(weather.clouds) + "%", RIGHT);
  drawString(x + 53, y + 93, String(weather.pressure, 0), CENTER);

  display.setFont(&DSEG7_Classic_Bold_21);
  display.setTextSize(2);
  if (weather.temperature < 0) {
    drawString(x + 18, y + 39, "-", LEFT); // Show temperature sign to compensate for non-proportional font spacing
    drawString(x + 35, y + 2, String(fabs(weather.temperature), 0), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 77, y + 10, "'", LEFT);    // Add-in ° symbol ' in this font 
  }
  else if (weather.temperature < 10) {
    drawString(x + 25, y + 2, String(fabs(weather.temperature), 0), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 67, y + 10, "'", LEFT);    // Add-in ° symbol ' in this font
  } else {
    drawString(x + 15, y + 2, String(fabs(weather.temperature), 0), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 82, y + 8, "'", LEFT);    // Add-in ° symbol ' in this font 
  }

  display.setFont(&DejaVu_Sans_Bold_11);
}

void displayWeatherDescription(int x, int y) {
  drawString(x, y, titleCase(weather.description), CENTER);
}

void displaySystemInfo(int x, int y) {
  int wifi_rssi = 0;
  int xpos = 1;

  display.drawRect(x, y, 135, 62, GxEPD_BLACK);

  int rssi_x = x + 33;
  int rssi_y = y + 38;
  for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) {
    if (_rssi <= -20)  wifi_rssi = 20; //  <-20dbm displays 5-bars
    if (_rssi <= -40)  wifi_rssi = 16; //  -40dbm to  -21dbm displays 4-bars
    if (_rssi <= -60)  wifi_rssi = 12; //  -60dbm to  -41dbm displays 3-bars
    if (_rssi <= -80)  wifi_rssi = 8;  //  -80dbm to  -61dbm displays 2-bars
    if (_rssi <= -100) wifi_rssi = 4;  // -100dbm to  -81dbm displays 1-bar
    display.fillRect(rssi_x + xpos * 5 + 60, rssi_y - wifi_rssi, 4, wifi_rssi, GxEPD_BLACK);
    xpos++;
  }
    
  display.fillRect(rssi_x + 60, rssi_y - 1, 4, 1, GxEPD_BLACK);  
  drawString(rssi_x, rssi_y - 9, String(rssi) + "dBm", LEFT);
  
  drawString(x + 68, y + 45, WiFi.localIP().toString(), CENTER);
}

void displayCloudCover(int x, int y, int cover) {
  addCloud(x, y,     SMALL * 0.5,  1);      // Main cloud
  addCloud(x + 5, y - 5, SMALL * 0.35, 1);  // Cloud top right
  addCloud(x - 8, y - 5, SMALL * 0.35, 1);  // Cloud top left
  drawString(x + 30, y - 5, String(cover) + "%", CENTER);
}

/**
 * @brief Title case the first word in the passed string.
 * 
 * @param text    String to title case.
 * @return String Returned title cased string or empty string if text length is 0.
 */
String titleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); // Title-case the string
  }
  return "";
}

void displayWind(int x, int y, float angle, float windspeed, int radius) {
  int offset = 16;
  int dxo;
  int dyo;
  int dxi;
  int dyi;

  arrow(x + offset, y + offset, radius - 11, angle, 15, 22, GxEPD_RED); // Show wind direction on outer circle of width and length
  display.setTextSize(0);
  
  display.drawCircle(x + offset, y + offset, radius, GxEPD_BLACK); // Draw compass circle
  display.drawCircle(x + offset, y + offset, radius + 1, GxEPD_BLACK); // Draw compass circle
  display.drawCircle(x + offset, y + offset, radius * 0.7, GxEPD_BLACK); // Draw compass inner circle
  for (float a = 0; a < 360; a = a + 22.5) {
    dxo = radius * cos((a - 90) * PI / 180);
    dyo = radius * sin((a - 90) * PI / 180);
    if (a == 45)  drawString(dxo + x + 10 + offset, dyo + y - 10 + offset, "NE", CENTER);
    if (a == 135) drawString(dxo + x + 12 + offset, dyo + y + offset,   "SE", CENTER);
    if (a == 225) drawString(dxo + x - 16 + offset, dyo + y + offset,    "SW", CENTER);
    if (a == 315) drawString(dxo + x - 12 + offset, dyo + y - 10 + offset, "NW", CENTER);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    display.drawLine(dxo + x + offset, dyo + y + offset, dxi + x + offset, dyi + y + offset, GxEPD_BLACK);
    dxo = dxo * 0.7;
    dyo = dyo * 0.7;
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    display.drawLine(dxo + x + offset, dyo + y + offset, dxi + x + offset, dyi + y + offset, GxEPD_BLACK);
  }

  display.setTextColor(GxEPD_RED);
  drawString(x + offset, y - radius - 11 + offset, "N", CENTER);
  display.setTextColor(GxEPD_BLACK);

  drawString(x + offset, y + 4 + offset + radius, "S", CENTER);
  drawString(x - radius - 10 + offset, y - 3 + offset, "W", CENTER);
  drawString(x + radius + offset + 7, y - 4 + offset, "E", CENTER);

  drawString(x + offset, y - 16 + offset, String(windspeed, 1), CENTER);

  display.setFont();  // use default 6x8 font
  drawString(x + offset + 3, y - 15 + offset, "mph", CENTER);

  display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x + offset, y + 10 + offset, String(angle, 0) +  "'", CENTER);
}

void arrow(int x, int y, int asize, float aangle, int pwidth, int plength, uint16_t colour) {
  float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
  float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  display.fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, colour);
}

String windDegToDirection(float winddirection) {
  
  if (winddirection >= 348.75 || winddirection < 11.25) {
    return "N";
  }

  if (winddirection >=  11.25 && winddirection < 33.75) {
    return "NNE";
  }

  if (winddirection >=  33.75 && winddirection < 56.25) {
    return "NE";
  }

  if (winddirection >=  56.25 && winddirection < 78.75) {
    return "ENE";
  }

  if (winddirection >=  78.75 && winddirection < 101.25) {
    return "E";
  }

  if (winddirection >= 101.25 && winddirection < 123.75) {
    return "ESE";
  }

  if (winddirection >= 123.75 && winddirection < 146.25) {
    return "SE";
  }

  if (winddirection >= 146.25 && winddirection < 168.75) {
    return "SSE";
  }

  if (winddirection >= 168.75 && winddirection < 191.25) {
    return "S";
  }

  if (winddirection >= 191.25 && winddirection < 213.75) {
    return "SSW";
  }

  if (winddirection >= 213.75 && winddirection < 236.25) {
    return "SW";
  }

  if (winddirection >= 236.25 && winddirection < 258.75) {
    return "WSW";
  }

  if (winddirection >= 258.75 && winddirection < 281.25) {
    return "W";
  }

  if (winddirection >= 281.25 && winddirection < 303.75) {
    return "WNW";
  }

  if (winddirection >= 303.75 && winddirection < 326.25) {
    return "NW";
  }

  if (winddirection >= 326.25 && winddirection < 348.75) {
    return "NNW";
  }

  return "?";
}

void displaySunAndMoon(int x, int y) {
  time_t now = time(NULL);
  struct tm * now_utc  = gmtime(&now);
  const int day_utc = now_utc->tm_mday;
  const int month_utc = now_utc->tm_mon + 1;
  const int year_utc = now_utc->tm_year + 1900;

  sunRiseSetIcon(x + 20, y + 20, SUN_UP);
  sunRiseSetIcon(x + 20, y + 45, SUN_DOWN);

  drawString(x + 40, y + 15, convertUnixTime(weather.sunrise).substring(0, 8), LEFT);   // 08:00 AM
  drawString(x + 40, y + 40, convertUnixTime(weather.sunset).substring(0, 8), LEFT);    // 19:00 PM
}

void displayWeatherForecast(int x, int y) {
  int offset = 57;

  for (byte i = 0; i < 5; i++) {
    displaySingleForecast(x + offset * i, y, offset, i);
  }

  float temp [forecast_counter] = {0};
  float pressure [forecast_counter] = {0};

  for(byte i = 0; i < forecast_counter; i++ )
  {
    temp[i] = forecast[i].temperature;  
    pressure[i] = forecast[i].pressure;
  }

  //(x, y, w, h, Data[], Range, len, title)
  drawSingleGraph(155, 205, 96, 75, temp, forecast_counter, "Temperature");
  drawSingleGraph(295, 205, 96, 75, pressure, forecast_counter, "Pressure (hPa)");
}

void displaySingleForecast(int x, int y, int offset, int index) {
  displayWeatherIcon(x + offset / 2 + 1, y + 35, forecast[index].icon, small_icon);

  drawString(x + offset / 2, y + 3, String(forecast[index].period.substring(11, 16)), CENTER);
  drawString(x + offset / 2, y + 50, String(forecast[index].high, 0) + "/" + String(forecast[index].low, 0), CENTER);
}

String convertUnixTime(uint32_t unix_time) {
  // Returns '21:12 PM'
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[12];
    
  strftime(output, sizeof(output), "%H:%M %p", now_tm);
  
  return output;
}


void drawMoon(int x, int y, int dd, int mm, int yy, String hemisphere, const int diameter, bool surface) {
//  const int diameter = 38;
  double phase = normalizedMoonPhase(dd, mm, yy);

  if (hemisphere == "south") {
    phase = 1 - phase;
  }

  // Draw dark part of moon
  display.fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, GxEPD_BLACK);
  const int number_of_lines = 90;
  for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++) {
    double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
    // Determine the edges of the lighted part of the moon
    double Rpos = 2 * Xpos;
    double Xpos1, Xpos2;
    if (phase < 0.5) {
      Xpos1 = -Xpos;
      Xpos2 = Rpos - 2 * phase * Rpos - Xpos;
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = Xpos - 2 * phase * Rpos + Rpos;
    }
    // Draw light part of moon
    double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW1y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW2y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW3y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW4y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    display.drawLine(pW1x, pW1y, pW2x, pW2y, GxEPD_WHITE);
    display.drawLine(pW3x, pW3y, pW4x, pW4y, GxEPD_WHITE);
  }

  display.drawCircle(x + diameter - 1, y + diameter, diameter / 2, GxEPD_BLACK);
}

double normalizedMoonPhase(int d, int m, int y) {
  int j = julianDate(d, m, y);
  //Calculate the approximate phase of the moon
  double Phase = (j + 4.867) / 29.53059;

  return (Phase - (int) Phase);
}

int julianDate(int d, int m, int y) {
  int mm, yy, k1, k2, k3, j;
  yy = y - (int)((12 - m) / 10);
  mm = m + 9;
  if (mm >= 12) mm = mm - 12;
  k1 = (int)(365.25 * (yy + 4712));
  k2 = (int)(30.6001 * mm + 0.5);
  k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
  // 'j' for dates in Julian calendar:
  j = k1 + k2 + d + 59 + 1;
  if (j > 2299160) {
    j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
  }

  return j;
}

void moonPhase(int x, int y, int day, int month, int year, String hemisphere) {
  int c, e;
  double jd;
  int b;
  int yoffset = y + 13;   // offset down for second word of moon type
  int xoffset = x + 4;    // offset across for shorter top word of moon type

  if (month < 3) {
    year--;
    month += 12;
  }

  ++month;
  c   = 365.25 * year;
  e   = 30.6 * month;
  jd  = c + e + day - 694039.09;     /* jd is total days elapsed */
  jd /= 29.53059;                        /* divide by the moon cycle (29.53 days) */
  b   = jd;                              /* int(jd) -> b, take integer part of jd */
  jd -= b;                               /* subtract integer part to leave fractional part of original jd */
  b   = jd * 8 + 0.5;                /* scale fraction from 0-8 and round by adding 0.5 */
  b   = b & 7;                           /* 0 and 8 are the same phase so modulo 8 for 0 */

  if (hemisphere == "south") {
    b = 7 - b;
  }

  
  if (b == 0) {         // New; 0% illuminated
    drawString(xoffset, y, "New", LEFT);
    drawString(x, yoffset, "Moon", LEFT);         
  } else if (b == 1) {  // Waxing crescent; 25% illuminated
    drawString(x, y, " Waxing", LEFT);
    drawString(x, yoffset, "Crescent", LEFT);            
  } else if (b == 2) {  // First quarter; 50% illuminated
    drawString(x, y, " First", LEFT);
    drawString(x, yoffset, "Quarter", LEFT);         
  } else if (b == 3) {  // Waxing gibbous; 75% illuminated
    drawString(xoffset, y, "Waxing", LEFT);
    drawString(x, yoffset, "Gibbous", LEFT);         
  } else if (b == 4) {  // Full; 100% illuminated
    drawString(xoffset, y, "Full", LEFT);
    drawString(x, yoffset, "Moon", LEFT);         
  } else if (b == 5) {  // Waning gibbous; 75% illuminated
    drawString(xoffset, y, "Waning", LEFT);
    drawString(x, yoffset, "Gibbous", LEFT);         
  } else if (b == 6) {     // Last quarter; 50% illuminated
    drawString(x, y, " Third", LEFT);
    drawString(x, yoffset, "Quarter", LEFT);         
  } else if (b == 7) {  // Waning crescent; 25% illuminated
    drawString(x, y, " Waning", LEFT);
    drawString(x, yoffset, "Crescent", LEFT);         
  }
}

void displayWeatherIcon(int x, int y, String icon, bool large_icon) {
  if (large_icon) { // == large icon, TODO: need to change this logic, variable name!
    x = x + 65;
    y = y + 65;
  }

  if (icon == "01d") {                                      // sun
    sunnyIcon(x, y, large_icon, icon, GxEPD_RED);           
  } else if (icon == "01n") {
    sunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
  } else if (icon == "02d") {                               // few clouds (clouds and sun)
    mostlySunnyIcon(x, y, large_icon, icon, GxEPD_RED);
  } else if (icon == "02n") {
    mostlySunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
  } else if (icon == "03d" || icon == "03n") {            // scattered clouds, no sun
    cloudyIcon(x, y, large_icon, icon);
  } else if (icon == "04d" || icon == "04n") {            // broken clouds, more clouds than scatterred!
    veryCloudyIcon(x, y, large_icon, icon);
  } else if (icon == "09d") {
    chanceOfRainIcon(x, y, large_icon, icon, GxEPD_RED);
  } else if (icon == "09n") {
    chanceOfRainIcon(x, y, large_icon, icon, GxEPD_BLACK);
  } else if (icon == "10d" || icon == "10n") {
    rainIcon(x, y, large_icon, icon);
  } else if (icon == "11d" || icon == "11n") {
    thunderStormIcon(x, y, large_icon, icon);
  } else if (icon == "13d" || icon == "13n") {
    snowIcon(x, y, large_icon, icon);
  } else if (icon == "50d") {
    hazeIcon(x, y, large_icon, icon, GxEPD_RED);
  } else if (icon == "50n") {
    fogIcon(x, y, large_icon, icon);
  } else {
    noData(x, y, large_icon);
  }
}

void displayRain(int x, int y) {
  if (forecast[1].rain > 0) {

    drawString(x, y, String(forecast[0].rain, (forecast[0].rain > 0.5 ? 2 : 3)) + "mm Rain", CENTER); // Only display rainfall if > 0
  }
}

void mostlySunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 10;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {    // Night time, add stars
    addMoon(x, y + offset, scale);
  } else {                          // Day time, add sun
    addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
  }

  addCloud(x, y + offset, scale, linesize);
}

void sunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color) {
  int scale = SMALL;
  int offset = 0;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (icon_name.endsWith("n")) {    // Night time, show stars
    addMoon(x, y + offset, scale);

    if (!large_size) {
      // Small stars
      addStar(x, y, SMALL_STAR);
      addStar(x - 18, y + 3, SMALL_STAR);
      addStar(x + 17, y - 10, SMALL_STAR);

      // Medium stars
      addStar(x - 1, y - 14, MEDIUM_STAR);    // top left star
      addStar(x + 10, y, MEDIUM_STAR);    // bottom right star
    } else {
      // Fill area with random stars
      int horizontal = 0;
      int vertical = 0;
      int left = 150;
      int top = 45;
      int width = 115;
      int height = 90;
      display.setTextColor(GxEPD_WHITE);
      for (int i = 0; i <= 41; i++) { 
        horizontal = (int) ( ((rand() / (RAND_MAX * 1.0f)) * width) + left);
        vertical = (int) ( ((rand() / (RAND_MAX * 1.0f)) * height) + top);            
        drawString(horizontal, vertical, ".", LEFT);
      }
      display.setTextColor(GxEPD_BLACK);
    }
  } else {                        // Day time, show sun
    scale = scale * 1.5;
    addSun(x, y + offset, scale, large_size, icon_color);
  }
}

void cloudyIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    if (icon_name.endsWith("n")) 
      addMoon(x, y + offset, scale);

    linesize = 1;
    addCloud(x, y + offset, scale, linesize);
  }
  else {
    if (icon_name.endsWith("n")) {
      addMoon(x, y + offset, scale);
    }

    addCloud(x, y + offset, scale, linesize); // Main cloud
  }
}

void veryCloudyIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    if (icon_name.endsWith("n")) 
      addMoon(x, y + offset, scale);

    linesize = 1;
    addCloud(x - 7, y - 7 + offset, 2, linesize); // Left v.small
    addCloud(x + 8, y - 10 + offset, 2, linesize); // Right v.small
    addCloud(x, y + offset, scale, linesize);     // Main cloud
  }
  else {
    if (icon_name.endsWith("n")) {
      addMoon(x, y + offset, scale);
    }

    addCloud(x + 28, y - 18 + offset, 4, linesize); // Cloud top right
    addCloud(x - 20, y - 8 + offset, 6, linesize); // Cloud top left
    addCloud(x, y + offset + 9, scale, linesize); // Main cloud
  }
}

void chanceOfRainIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addRain(x, y + offset, scale, GxEPD_WHITE);
  } else {
    addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
    addRain(x, y + offset, scale, GxEPD_BLACK);
  }

  addCloud(x, y + offset, scale, linesize);
}

void rainIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addRain(x, y + offset, scale, GxEPD_WHITE);
  } else {
    addRain(x, y + offset, scale, GxEPD_BLACK);
  }

  addCloud(x, y + offset, scale, linesize);
}

void thunderStormIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }
  
  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addThunderStorm(x, y + offset, scale, GxEPD_WHITE);
  } else {
    addThunderStorm(x, y + offset, scale, GxEPD_BLACK);
  }

  addCloud(x, y + offset, scale, linesize);
}

void snowIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addSnow(x, y + offset, scale, GxEPD_WHITE);
  } else {
    addSnow(x, y + offset, scale, GxEPD_BLACK);
  }

  addCloud(x, y + offset, scale, linesize);
}

void fogIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addFog(x, y + offset, scale, linesize, GxEPD_WHITE);
  } else {
    addFog(x, y + offset, scale, linesize, GxEPD_BLACK);
    addCloud(x, y + offset, scale, linesize);
  }
}

void hazeIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color) {
  int scale = SMALL;
  int offset = 0;
  int linesize = 3;

  if (large_size) {
    scale = LARGE;
    offset = 0;
  }

  if (scale == SMALL) {
    linesize = 1;
  }

  if (icon_name.endsWith("n")) {
    addMoon(x, y + offset, scale);
    addFog(x, y + offset, scale * 1.4, linesize, GxEPD_WHITE);
  } else {
    addSun(x, y + offset, scale * 1.4, large_size, icon_color);
    addFog(x, y + offset, scale * 1.4, linesize, GxEPD_BLACK);
  }
}

void sunRiseSetIcon(uint16_t x, uint16_t y, sun_direction direction) {
  uint16_t r = 7;

  // Horizontal
  display.drawLine(x - r*2 + 2, y, x + r*2 - 2, y, GxEPD_BLACK);
  // Vertical
  display.drawLine(x, y - r*2 + 2, x, y, GxEPD_BLACK);
  // Angle Top right
  display.drawLine(x - r*2 + 5, y - r*2 + 5, x, y, GxEPD_BLACK);
  // Angle Top left
  display.drawLine(x, y, x + r*2 - 5, y - r*2 + 5, GxEPD_BLACK);
  // Remove lines inside
  display.fillCircle(x, y, r + 1, GxEPD_WHITE);
  // Empty inside
  display.fillCircle(x, y, r-1, GxEPD_RED);
  display.drawCircle(x, y, r-1, GxEPD_BLACK);
  // Overwrite the bottom
  display.fillRect(x - r, y + 4, r*2, r, GxEPD_WHITE);
  
  // Arrow up
  if(direction == SUN_UP) {
    display.fillTriangle( x-r/2-1, y+r-2, x, y+r - 7, x+r/2+1, y+r-2, GxEPD_WHITE);
    display.drawLine( x-r/2, y+r-2, x, y+r - 6, GxEPD_BLACK);
    display.drawLine( x, y+r - 6, x+r/2, y+r-2, GxEPD_BLACK);
  } else {
    // Arrow DOWN
    display.drawLine( x-r/2, y+r-2, x, y+r + 2, GxEPD_BLACK);
    display.drawLine( x, y+r + 2, x+r/2, y+r-2, GxEPD_BLACK);
  }

  // Horizon line
  display.drawLine( x-r, y+r-2, x-r/2, y+r-2, GxEPD_BLACK);
  display.drawLine( x+r/2, y+r-2, x+r, y+r-2, GxEPD_BLACK);
}

void addMoon (int x, int y, int scale) {
  if (scale == LARGE) {
    display.fillCircle(x - 37, y - 30, scale, GxEPD_BLACK);
    display.fillCircle(x - 24, y - 30, scale * 1.6, GxEPD_WHITE);  
  }
  else
  {
    display.fillCircle(x - 20, y - 15, scale, GxEPD_BLACK);
    display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_WHITE);
  }
}

void addSun(int x, int y, int scale, boolean icon_size, uint16_t icon_color) {
  int linesize = 3;
  int dxo, dyo, dxi, dyi;

  if (icon_size == small_icon) {
    linesize = 1;
  }

  display.fillCircle(x, y, scale, icon_color);
  if (icon_color != GxEPD_RED) {        // not day time or 2 colour display
    display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
  } 

  for (float i = 0; i < 360; i = i + 45) {
    dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.6;
    dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.6;
    if (i == 0   || i == 180) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (icon_size == large_icon) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
      }
    }
    if (i == 90  || i == 270) {
      display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1, GxEPD_BLACK);
      if (icon_size == large_icon) {
        display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0, GxEPD_BLACK);
        display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1, GxEPD_BLACK);
      }
    }
    if (i == 45  || i == 135 || i == 225 || i == 315) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (icon_size == large_icon) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
      }
    }
  }
}

void addCloud(int x, int y, int scale, int linesize) {
  //Draw cloud outer
  display.fillCircle(x - scale * 3, y, scale, GxEPD_BLACK);                  // Left most circle
  display.fillCircle(x + scale * 3, y, scale, GxEPD_BLACK);                  // Right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4, GxEPD_BLACK);      // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, GxEPD_BLACK); // Right middle upper circle
  display.fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, GxEPD_BLACK); // Upper and lower lines
  //Clear cloud inner
  display.fillCircle(x - scale * 3, y, scale - linesize, GxEPD_WHITE);     // Clear left most circle
  display.fillCircle(x + scale * 3, y, scale - linesize, GxEPD_WHITE);     // Clear right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4 - linesize, GxEPD_WHITE); // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, GxEPD_WHITE); // Right middle upper circle
  display.fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, GxEPD_WHITE); // Upper and lower lines
}

void addRain(int x, int y, int scale, uint16_t colour) {
  for (byte i = 0; i < 6; i++) {
    display.fillCircle(x - scale * 4 + scale * i * 1.3, y + scale * 1.9 + (scale == SMALL ? 3 : 0), scale / 3, colour);
    arrow(x - scale * 4 + scale * i * 1.3 + (scale == SMALL ? 6 : 4), y + scale * 1.6 + (scale == SMALL ? -3 : -1), scale / 6, 40, scale / 1.6, scale * 1.2, colour);
  }
}

void addSnow(int x, int y, int scale, uint16_t colour) {
  int dxo, dyo, dxi, dyi;

  for (byte flakes = 0; flakes < 5; flakes++) {
    for (int i = 0; i < 360; i = i + 45) {
      dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.1;
      dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.1;
      display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3, dyo + y + scale * 2, dxi + x + 0 + flakes * 1.5 * scale - scale * 3, dyi + y + scale * 2, colour);
    }
  }
}

void addThunderStorm(int x, int y, int scale, uint16_t colour) {
  y = y + scale / 2;

  for (byte i = 0; i < 5; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, colour);
    if (scale != SMALL) {
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, colour);
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, colour);
    }
    display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, colour);
    if (scale != SMALL) {
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, colour);
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, colour);
    }
    display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, colour);
    if (scale != SMALL) {
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, colour);
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, colour);
    }
  }
}

void addFog(int x, int y, int scale, int linesize, uint16_t colour) {
  if (scale == SMALL) {
    y -= 10;
  }

  for (byte i = 0; i < 6; i++) {
    display.fillRect((x + 8) - scale * 3, y + scale * -1, scale * 4, linesize, colour);
    display.fillRect((x + 10) - scale * 3, y + scale * -0.5, scale * 5, linesize, colour);
    display.fillRect((x + 5) - scale * 3, y + scale * 0, scale * 3, linesize, colour);
    display.fillRect((x + 12) - scale * 3, y + scale * 0.5, scale * 2, linesize, colour);
    display.fillRect((x + 8) - scale * 3, y + scale * 1, scale * 4, linesize, colour);

    display.fillRect(x - scale * 3, y + scale * 1.5, scale * 4, linesize, colour);
    display.fillRect((x + 5) - scale * 3, y + scale * 2.0, scale * 4, linesize, colour);
    display.fillRect((x + 10) - scale * 3, y + scale * 2.5, scale * 3, linesize, colour);
  }
}

void noData(int x, int y, bool large_size) {
  int scale = SMALL;
  int offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 7;
  }

  if (scale == LARGE) {
    display.setFont(&FreeMonoBold12pt7b); 
  } else {
    display.setFont(&DejaVu_Sans_Bold_11);
  }

  drawString(x - 20, y - 10 + offset, "N/A", LEFT);
}

void addStar(int x, int y, star_size starsize) {
  if (starsize == SMALL_STAR) {
    display.drawTriangle(x, y, x - 2, y + 3, x + 2, y + 3, GxEPD_WHITE);
    display.drawTriangle(x, y + 4, x - 2, y + 1, x + 2, y + 1, GxEPD_WHITE);
  } else if (starsize == MEDIUM_STAR) {
    display.drawTriangle(x, y, x - 4, y + 6, x + 4, y + 6, GxEPD_WHITE);
    display.drawTriangle(x, y + 8, x - 4, y + 2, x + 4, y + 2, GxEPD_WHITE);
  }
}

/**
 * @brief Draw graph
 * 
 * @param x     x coordinates
 * @param y     y coordinates
 * @param w     graph width
 * @param h     graph height
 * @param Data  Data array one
 * @param Data2 Data array two (null == empty array)
 * @param len   Length of data array
 * @param title graph title 
 */
void drawGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h, float Data[], float Data2[], int len, String title) {
    float ymin;
    float ymax;
    int ticklines = 5;
    int auto_scale_margin = 0;

    int maxYscale = -10000;
    int minYscale =  10000;

    for (int i = 1; i < len; i++ ) {
      if (Data[i] >= maxYscale) {
        maxYscale = Data[i];
      }
      if (Data[i] <= minYscale) {
        minYscale = Data[i];
      }

      if (Data2 != NULL) {
        if (Data2[i] >= maxYscale) {
          maxYscale = Data2[i];
        }
        if (Data2[i] <= minYscale) {
          minYscale = Data2[i];
        }
      }
    }
    maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
    ymax = round(maxYscale + 0.5);
    if (minYscale != 0) {
      minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
    }
    ymin = round(minYscale);

    float steps = (ymax-ymin)/(ticklines);

    // Title
    display.setFont();
    drawString(x + w / 2, y - 24, title, CENTER);
    
    // Draw y-axis tick markers and dashed lines
    for (byte i =0; i<ticklines+1; i++){
      drawString(x-2,y+((h/ticklines)*i-12), String( lrint(ymin+steps*(ticklines-i)) ), RIGHT);
      if(i == 0) {
        continue;
      }
      display.drawLine(x+1,y+((h/ticklines)*i),x+w-1,y+((h/ticklines)*i),GxEPD_RED);
      bool blank = true;
      for(byte r = 0; r<w; r = r+3 ){
        if(blank) {
          display.drawLine(x+r,y+((h/ticklines)*i),x+r+3,y+((h/ticklines)*i),GxEPD_WHITE);
          blank = false;
        } else {
          blank = true;
        }
      }
    }

    // x-Axis
    display.drawLine(x,y+h,x+w,y+h,GxEPD_BLACK);

    // y-Axis
    display.drawLine(x,y,x,y+h,GxEPD_BLACK);

    // Draw data line 1
    float x1 = x + 1;
    float y1 = y + (ymax - constrain(Data[0], ymin, ymax)) / (ymax - ymin) * h;
    float y3 = y + (ymax - constrain(Data2[0], ymin, ymax)) / (ymax - ymin) * h;

    if(y1 > y+h-1)
      y1 = y+h-1;
    if(y3 > y+h-1)
      y3 = y+h-1;
    float x2, y2, y4;

    for (int i = 1; i < len; i++ ) 
    {
      x2 = x + i * w / (len-1) - 1;
      y2 = y + (ymax - constrain(Data[i], ymin, ymax)) / (ymax - ymin) * h + 1;
      y4 = y + (ymax - constrain(Data2[i], ymin, ymax)) / (ymax - ymin) * h + 1;
      if(y2 > y+h-1)
        y2 = y+h-1;
      if(y4 > y+h-1)
        y4 = y+h-1;

      display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

      x1 = x2;
      y1 = y2;
      y3 = y4;
    }

    // x-Axis ticks
    struct tm *lt;
    char buff[32];
    for (int i = 0; i <= 4; i++) {
      display.setCursor(x-4+(w/4)*i ,y+h+6);
      display.print(String(12*i)+"h");
    }

  // Reset font
  display.setFont(&DejaVu_Sans_Bold_11);
}

/**
 * @brief Draw graph with 1 set of data
 * 
 * @param x     x coordinates
 * @param y     y coordinates
 * @param w     graph width
 * @param h     graph height
 * @param Data  Data array
 * @param len   Length of data array
 * @param title graph title 
 */
void drawSingleGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h, float Data[], int len, String title) {
    float ymin;
    float ymax;
    int ticklines = 5;
    int auto_scale_margin = 0;

    int maxYscale = -10000;
    int minYscale =  10000;

    for (int i = 1; i < len; i++ ) {
      if (Data[i] >= maxYscale) {
        maxYscale = Data[i];
      }
      if (Data[i] <= minYscale) {
        minYscale = Data[i];
      }
    }

    maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
    ymax = round(maxYscale + 0.5);
    if (minYscale != 0) {
      minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
    }
    ymin = round(minYscale);

    float steps = (ymax-ymin)/(ticklines);

    // Title
    display.setFont();
    drawString(x + w / 2, y - 24, title, CENTER);
    
    // Draw y-axis tick markers and dashed lines
    for (byte i =0; i<ticklines+1; i++){
      // Due to space constraints only show values under 10 with a decimal point
      if (ymin < 1 && ymax < 10) {
        drawString(x-2, y+((h/ticklines)*i-12), String(ymin+steps*(ticklines-i), 1), RIGHT);
      } else {
        drawString(x-2,y+((h/ticklines)*i-12), String(lrint(ymin+steps*(ticklines-i))), RIGHT);
      }

      if(i == 0) {
        continue;
      }

      display.drawLine(x+1,y+((h/ticklines)*i),x+w-1,y+((h/ticklines)*i),GxEPD_RED);
      bool blank = true;
      for(byte r = 0; r<w; r = r+3 ){
        if(blank) {
          display.drawLine(x+r,y+((h/ticklines)*i),x+r+3,y+((h/ticklines)*i),GxEPD_WHITE);
          blank = false;
        } else {
          blank = true;
        }
      }
    }

    // x-Axis
    display.drawLine(x,y+h,x+w,y+h,GxEPD_BLACK);

    // y-Axis
    display.drawLine(x,y,x,y+h,GxEPD_BLACK);

    // Draw data line 1
    float x1 = x + 1;
    float y1 = y + (ymax - constrain(Data[0], ymin, ymax)) / (ymax - ymin) * h;
    float x2, y2;

    if(y1 > y+h-1) {
      y1 = y+h-1;
    }

    // Draw data on display
    for (int i = 1; i < len; i++ ) 
    {
      x2 = x + i * w / (len-1) - 1;
      y2 = y + (ymax - constrain(Data[i], ymin, ymax)) / (ymax - ymin) * h + 1;
      if(y2 > y+h-1) {
        y2 = y+h-1;
      }
      // More solid line using 2 lines 1 pixel apart.
      display.drawLine(x1, y1-1, x2, y2-1, GxEPD_BLACK);
      display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

      x1 = x2;
      y1 = y2;
    }

    // x-Axis ticks
    for (int i = 0; i <= 6; i++) {
      display.setCursor(x-7+(w/6)*i ,y+h+6);
      //display.print(String(12*i)+"h");
      if (i == 0) {   // day 0 (now)
        display.print("0");
      } else if ((12 * i) == 24) {   // day 1
        display.print("24");
      } else if ((12 * i) == 48) {  // day 2
        display.print("48");
      } else if ((12 * i) == 72) {  // day 3
        display.print("72");
      }
    }

  // Reset font
  display.setFont(&DejaVu_Sans_Bold_11);
}

/**
 * @brief Draw text to the display.
 * 
 * @param x     x coordinate.
 * @param y     y coordinate.
 * @param text  Text to draw to the display.
 * @param align Text alignment, left, right, or center.
 */
void drawString(int x, int y, String text, alignment align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;

  display.setTextWrap(false);
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT) {
    x = x - w;
  }

  if (align == CENTER) {
    x = x - w / 2;
  }

  display.setCursor(x, y + h);
  display.print(text);
}