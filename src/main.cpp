/***
 * ESP32 weather monitor application using a 3 colour Waveshare 4.2" e-ink display. Weather
 * data is obtained from Open Weather Map
 * 
 * Application was written using VSCode and platformio to manage the project.  Graphics library used
 * for the e-ink display is GxEPD2.
 * 
 * This is a copy/update of the weather display written by G6EJD:
 *  https://github.com/G6EJD/ESP32-Revised-Weather-Display-42-E-Paper
*/
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "GxEPD2_GFX.h"
#include "GxEPD2_3C.h"    // 3 colour screen
#include "GxEPD2_display_selection_new_style.h"

#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

#include "main.h"
#include "config.h"
#include "fonts.h"
#include "weather.h"

/* Constants/defines */
const uint SCREEN_WIDTH = 400;
const uint SCREEN_HEIGHT = 300;
const String VERSION = "v1";
const String Hemisphere = "north";

#define LARGE  10
#define SMALL  4

boolean large_icon = true;
boolean small_icon = false;

/* Globals etc. */
WiFiClientSecure wifiClient;

String ipAddress = "0:0:0:0";
int rssi = 0;

float pressure_readings[5]    = {0};
float temperature_readings[5] = {0};
float rain_readings[5]        = {0};

// current
typedef struct WeatherStruct {
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
  String   trend;
  String   period;
} WeatherStruct;

WeatherStruct weather;
WeatherStruct forecast[5];

static portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;


hw_timer_t *My_timer = NULL;    // Hardware timer for updating clock etc.
volatile bool update = false;   // Tinmer update flag
char timeStringBuff[35];        // buffer for time on the display
char dateStringBuff[35];

// timer interrupt routine
void IRAM_ATTR onTimerInterrupt() {
  portENTER_CRITICAL_ISR(&myMux);
  update = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

void setup() {
  Serial.begin(115200);

  initialiseDisplay();

  Serial.printf("Display Width %d, Display Height %d\n", display.width(), display.height());

  // #ifdef DEBUG
  //   #warning "DEBUG is defined in platformio.ini"
  //        build_type = debug
  //        debug_build_flags = -D DEBUG
    Serial.println("\n##################################");
    Serial.println(F("ESP32 Information:"));
    Serial.printf("Internal Total Heap %d, Internal Used Heap %d, Internal Free Heap %d\n", ESP.getHeapSize(), ESP.getHeapSize()-ESP.getFreeHeap(), ESP.getFreeHeap());
    Serial.printf("Sketch Size %d, Free Sketch Space %d\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
    Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("Chip Model %s, ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
    Serial.printf("Sizeof byte:%d  uint: %d  uint8: %d  uint16: %d  uint32: %d  int: %d\n", sizeof(byte), sizeof(uint), sizeof(uint8_t), sizeof(uint16_t), sizeof(uint32_t), sizeof(int));
    Serial.println("##################################\n");
  // #else
  //   #warning "DEBUG is not defined in platformio.ini"
  // #endif

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

  // Update buffer with current time
//  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S %a %b %d %Y", &timeinfo);
//  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M %a %b %d %Y", &timeinfo);
//  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);

  strftime(dateStringBuff, sizeof(dateStringBuff), "%a  %d-%b-%y", &timeinfo);      // Sat 15-Jan-23
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);          // 15:15:23
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

//#ifdef SHOW_JSON
  int ccount = 0;
//#endif

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
//    JsonObject obj = doc.as<JsonObject>(); // Convert to JSON object

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

//#ifdef SHOW_JSON
  int ccount = 0;
//#endif

  // Send GET request
  Serial.printf("Sending GET request to %s, port %d\n", host, port);
  client.print(String("GET ") + FORECAST_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

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
  Serial.println("Parsing Forecast JSON...");

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
    Serial.println(F("Deserialization succeeded, decoding forecast data..."));

    for (byte i = 0; i < 5; i++) {
      forecast[i].dt = doc["list"][i]["dt"];   
      forecast[i].temperature = doc["list"][i]["main"]["temp"];
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
//      forecast[i].rain = doc["list"][i][];
//      forecast[i].snow = doc["list"][i][];
      forecast[i].period = doc["list"][i]["dt_txt"].as<String>();      
    }

    float pressure_trend = forecast[0].pressure - forecast[1].pressure; // Measure pressure slope between ~now and later
    pressure_trend = ((int)(pressure_trend * 10)) / 10.0;               // Remove any small variations less than 0.1
    weather.trend = "0";

    if (pressure_trend > 0)  
      weather.trend = "+";
    else if (pressure_trend < 0)  
      weather.trend = "-";

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
    display.printf("Error: ", message);
   } while (display.nextPage());

   display.hibernate();
}

void displayInformation(bool today_flag, bool forecast_flag) {
  uint32_t dt = millis();

  display.setFullWindow();
  display.firstPage();
  do {
    displayHeader();
    displayTemperature(0, 15);
    displayWeatherPerson(145, 15, weather.icon);          // Weather person depiction of weather
    displayWeatherIcon(276, 15, weather.icon, large_icon); // Weather icon
    displayWeatherForcast(131, 172);
    displayMoonPhase(131, 174);                       // Astronomy section Sun rise/set, Moon phase and Moon icon
    displayWeatherDescription(0, 148); 
    displayWind(50, 220, weather.wind_deg, weather.wind_speed, 50); // Wind direction info
    displaySystemInfo(293, 238);
  } while (display.nextPage());
  
  display.hibernate();

  Serial.print("\nDisplay updated in "); Serial.print((millis()-dt)/1000); Serial.println(" secs");
}

void displayHeader(void) {
  drawString(SCREEN_WIDTH / 2, 2, LOCATION, CENTER);
  drawString(SCREEN_WIDTH, 2, dateStringBuff, RIGHT);
  drawString(2, 5, timeStringBuff, LEFT);
  drawString(115, 5, VERSION, CENTER);
  display.drawLine(0, 15, SCREEN_WIDTH, 15, GxEPD_BLACK);
}

void displayTemperature(int x, int y) {
  display.drawRect(x, y, 144, 130, GxEPD_BLACK);
  display.setFont(&DSEG7_Classic_Bold_21);
  display.setTextSize(2);

  if (weather.temperature < 0) {
    drawString(x, y + 61, "-", LEFT); // Show temperature sign to compensate for non-proportional font spacing
    drawString(x + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 95, y + 25, "'C", LEFT);    // Add-in ° symbol ' in this font plus units
  }
  else if (weather.temperature < 10) {
    drawString(x + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 95, y + 25, "'C", LEFT);    // Add-in ° symbol ' in this font plus units
  } else {
    drawString(x, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
    display.setTextSize(1);
    drawString(x + 105, y + 25, "'C", LEFT);    // Add-in ° symbol ' in this font plus units
  }


  drawString(x + 25,  y + 89, String(weather.low, 0) + "'/" + String(weather.high, 0) + "'", LEFT); // Show forecast high and Low, in the font ' is a °
  display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x + 72, y + 115, String(weather.humidity) + "% RH", CENTER);
  if (weather.clouds > 0) 
    displayCloudCover(x + 60, y + 10, weather.clouds);
}

void displaySystemInfo(int x, int y) {
  int wifi_rssi = 0;
  int xpos = 1;

  display.drawRect(x, y, 107, 62, GxEPD_BLACK);

  int rssi_x = x + 15;
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
  
  drawString(x + 5, y + 45, WiFi.localIP().toString(), LEFT);
}

void displayCloudCover(int x, int y, int cover) {
  addCloud(x, y,     SMALL * 0.5,  1);      // Main cloud
  addCloud(x + 5, y - 5, SMALL * 0.35, 1);  // Cloud top right
  addCloud(x - 8, y - 5, SMALL * 0.35, 1);  // Cloud top left
  drawString(x + 30, y - 5, String(cover) + "%", CENTER);
}

void displayWeatherDescription(int x, int y) {
  display.drawRect(x, y - 4, SCREEN_WIDTH, 27, GxEPD_BLACK);
  String description = weather.description + ", " + weather.main;
  
  display.setFont(&FreeMonoBold12pt7b);
  unsigned int MsgWidth = 28;
  if (description.length() > MsgWidth) {
    display.setFont(&DejaVu_Sans_Bold_11); // Drop to smaller font to allow all weather description to be displayed
    MsgWidth = 52;
    y = y - 7;
  }
  drawStringMaxWidth(x + 3, y + 15, MsgWidth, titleCase(description), LEFT); // 28 character screen width at this font size
  display.setFont(&DejaVu_Sans_Bold_11);
}

String titleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); // Title-case the string
  }
  return "";
}

void drawStringMaxWidth(int x, int y, unsigned int text_width, String text, alignment align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;

  if (text.length() > text_width * 2) 
    text = text.substring(0, text_width * 2); // Truncate if too long for 2 rows of text
  
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)  
    x = x - w;
  
  if (align == CENTER) 
    x = x - w / 2;

  display.setCursor(x, y);
  display.println(text.substring(0, text_width));
  if (text.length() > text_width) {
    display.setCursor(x, y + h);
    display.println(text.substring(text_width));
  }
}

void displayWind(int x, int y, float angle, float windspeed, int radius) {
  int offset = 15;
  arrow(x + offset, y + offset, radius - 11, angle, 15, 22); // Show wind direction on outer circle of width and length
  display.setTextSize(0);
  display.drawRect(x - radius, y - radius, 130, 130, GxEPD_BLACK);
  int dxo, dyo, dxi, dyi;
  display.drawCircle(x + offset, y + offset, radius, GxEPD_BLACK); // Draw compass circle
  display.drawCircle(x + offset, y + offset, radius + 1, GxEPD_BLACK); // Draw compass circle
  display.drawCircle(x + offset, y + offset, radius * 0.7, GxEPD_BLACK); // Draw compass inner circle
  for (float a = 0; a < 360; a = a + 22.5) {
    dxo = radius * cos((a - 90) * PI / 180);
    dyo = radius * sin((a - 90) * PI / 180);
    if (a == 45)  drawString(dxo + x + 10 + offset, dyo + y - 10 + offset, "NE", CENTER);
    if (a == 135) drawString(dxo + x + 5 + offset, dyo + y + 5 + offset,   "SE", CENTER);
    if (a == 225) drawString(dxo + x - 10 + offset, dyo + y + offset,    "SW", CENTER);
    if (a == 315) drawString(dxo + x - 10 + offset, dyo + y - 10 + offset, "NW", CENTER);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    display.drawLine(dxo + x + offset, dyo + y + offset, dxi + x + offset, dyi + y + offset, GxEPD_BLACK);
    dxo = dxo * 0.7;
    dyo = dyo * 0.7;
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    display.drawLine(dxo + x + offset, dyo + y + offset, dxi + x + offset, dyi + y + offset, GxEPD_BLACK);
  }
  drawString(x + offset, y - radius - 11 + offset, "N", CENTER);
  drawString(x + offset, y + 3 + offset + radius, "S", CENTER);
  drawString(x - radius - 8 + offset, y - 5 + offset, "W", CENTER);
  drawString(x + radius + offset + 7, y - 3 + offset, "E", CENTER);
  drawString(x + offset, y - 23 + offset, windDegToDirection(angle), CENTER);
  drawString(x + offset, y + 12 + offset, String(angle, 0) + "°", CENTER);
  drawString(x + offset, y - 5 + offset, String(windspeed, 1) + " mph", CENTER);
}

void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
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
  display.fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, GxEPD_BLACK);
}

String windDegToDirection(float winddirection) {
  if (winddirection >= 348.75 || winddirection < 11.25)  return "N";
  if (winddirection >=  11.25 && winddirection < 33.75)  return "NNE";
  if (winddirection >=  33.75 && winddirection < 56.25)  return "NE";
  if (winddirection >=  56.25 && winddirection < 78.75)  return "ENE";
  if (winddirection >=  78.75 && winddirection < 101.25) return "E";
  if (winddirection >= 101.25 && winddirection < 123.75) return "ESE";
  if (winddirection >= 123.75 && winddirection < 146.25) return "SE";
  if (winddirection >= 146.25 && winddirection < 168.75) return "SSE";
  if (winddirection >= 168.75 && winddirection < 191.25) return "S";
  if (winddirection >= 191.25 && winddirection < 213.75) return "SSW";
  if (winddirection >= 213.75 && winddirection < 236.25) return "SW";
  if (winddirection >= 236.25 && winddirection < 258.75) return "WSW";
  if (winddirection >= 258.75 && winddirection < 281.25) return "W";
  if (winddirection >= 281.25 && winddirection < 303.75) return "WNW";
  if (winddirection >= 303.75 && winddirection < 326.25) return "NW";
  if (winddirection >= 326.25 && winddirection < 348.75) return "NNW";
  return "?";
}

void displayWeatherPerson(int x, int y, String icon) {
  display.drawRect(x, y, 130, 130, GxEPD_BLACK);

  // NOTE: Using 'drawInvertedBitmap' and not 'drawBitmap' so that images are WYSIWYG, otherwise all images need to be inverted
  if      (icon == "01d" || icon == "01n")  display.drawInvertedBitmap(x, y, WX_Sunny,       128, 128, GxEPD_BLACK);
  else if (icon == "02d" || icon == "02n")  display.drawInvertedBitmap(x, y, WX_MostlySunny, 128, 128, GxEPD_BLACK);
  else if (icon == "03d" || icon == "03n")  display.drawInvertedBitmap(x, y, WX_Cloudy,      128, 128, GxEPD_BLACK);
  else if (icon == "04d" || icon == "04n")  display.drawInvertedBitmap(x, y, WX_MostlySunny, 128, 128, GxEPD_BLACK);
  else if (icon == "09d" || icon == "09n")  display.drawInvertedBitmap(x, y, WX_ChanceRain,  128, 128, GxEPD_BLACK);
  else if (icon == "10d" || icon == "10n")  display.drawInvertedBitmap(x, y, WX_Rain,        128, 128, GxEPD_BLACK);
  else if (icon == "11d" || icon == "11n")  display.drawInvertedBitmap(x, y, WX_TStorms,     128, 128, GxEPD_BLACK);
  else if (icon == "13d" || icon == "13n")  display.drawInvertedBitmap(x+8, y, WX_Snow,      128, 128, GxEPD_BLACK);
  else if (icon == "50d")                   display.drawInvertedBitmap(x, y, WX_Haze,        128, 128, GxEPD_BLACK);
  else if (icon == "50n")                   display.drawInvertedBitmap(x+8, y, WX_Fog,         128, 128, GxEPD_BLACK);
  else                                      display.drawInvertedBitmap(x, y, WX_Nodata,      128, 128, GxEPD_BLACK);
}

void displayMoonPhase(int x, int y) {
  display.drawRect(x, y + 64, 161, 62, GxEPD_BLACK);
  drawString(x + 4,  y + 67, "Sun Rise/Set", LEFT);
  drawString(x + 20, y + 82, convertUnixTime(weather.sunrise).substring(0, 5), LEFT);
  drawString(x + 20, y + 96, convertUnixTime(weather.sunset).substring(0, 5), LEFT);
  time_t now = time(NULL);
  struct tm * now_utc  = gmtime(&now);
  const int day_utc = now_utc->tm_mday;
  const int month_utc = now_utc->tm_mon + 1;
  const int year_utc = now_utc->tm_year + 1900;
  drawString(x + 4,  y + 109, moonPhase(day_utc, month_utc, year_utc, Hemisphere), LEFT);
  drawMoon(x + 95,   y + 56, day_utc, month_utc, year_utc, Hemisphere);
}


void displayWeatherForcast(int x, int y) {
  int offset = 54;

  for (byte i = 0; i < 5; i++) {
    displaySingleForecast(x + offset * i, y, offset, i);
  }

  for (byte i = 0; i < 5; i++) {
    pressure_readings[i] = forecast[i].pressure;    
    temperature_readings[i] = forecast[i].temperature;
    rain_readings[i] = forecast[i].rain;
  }
}

void displaySingleForecast(int x, int y, int offset, int index) {
  display.drawRect(x, y, offset - 1, 65, GxEPD_BLACK);
  display.drawLine(x, y + 13, x + offset - 2, y + 13, GxEPD_BLACK);
  displayWeatherIcon(x + offset / 2 + 1, y + 35, forecast[index].icon, small_icon);
  drawString(x + offset / 2, y + 3, String(forecast[index].period.substring(11, 16)), CENTER);
  drawString(x + offset / 2, y + 50, String(forecast[index].high, 0) + "/" + String(forecast[index].low, 0), CENTER); //+ "*", LEFT); if you want the ° symbol in this font
}

String convertUnixTime(int unix_time) {
  // Returns '21:12  '
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
    
  strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
  
  return output;
}

void drawMoon(int x, int y, int dd, int mm, int yy, String hemisphere) {
  const int diameter = 38;
  double Phase = normalizedMoonPhase(dd, mm, yy);
  if (hemisphere == "south") Phase = 1 - Phase;
  // Draw dark part of moon
  display.fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, GxEPD_BLACK);
  const int number_of_lines = 90;
  for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++) {
    double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
    // Determine the edges of the lighted part of the moon
    double Rpos = 2 * Xpos;
    double Xpos1, Xpos2;
    if (Phase < 0.5) {
      Xpos1 = -Xpos;
      Xpos2 = Rpos - 2 * Phase * Rpos - Xpos;
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = Xpos - 2 * Phase * Rpos + Rpos;
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
  if (j > 2299160) j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
  return j;
}

String moonPhase(int d, int m, int y, String hemisphere) {
  int c, e;
  double jd;
  int b;
  if (m < 3) {
    y--;
    m += 12;
  }
  ++m;
  c   = 365.25 * y;
  e   = 30.6 * m;
  jd  = c + e + d - 694039.09;     /* jd is total days elapsed */
  jd /= 29.53059;                        /* divide by the moon cycle (29.53 days) */
  b   = jd;                              /* int(jd) -> b, take integer part of jd */
  jd -= b;                               /* subtract integer part to leave fractional part of original jd */
  b   = jd * 8 + 0.5;                /* scale fraction from 0-8 and round by adding 0.5 */
  b   = b & 7;                           /* 0 and 8 are the same phase so modulo 8 for 0 */
  if (hemisphere == "south") b = 7 - b;
  if (b == 0) return "New moon";         // New; 0% illuminated
  if (b == 1) return "Waxing crescent";  // Waxing crescent; 25% illuminated
  if (b == 2) return "First quarter";    // First quarter; 50% illuminated
  if (b == 3) return "Waxing gibbous";   // Waxing gibbous; 75% illuminated
  if (b == 4) return "Full";             // Full; 100% illuminated
  if (b == 5) return "Waning gibbous";   // Waning gibbous; 75% illuminated
  if (b == 6) return "Third quarter";    // Last quarter; 50% illuminated
  if (b == 7) return "Waning crescent";  // Waning crescent; 25% illuminated
  return "";
}

void displayWeatherIcon(int x, int y, String icon, bool large_size) {
  if (large_size) {
    display.drawRect(x, y, 124, 130, GxEPD_BLACK);
    displayPressureAndTrend(x + 45, y + 100, weather.pressure, weather.trend);
    displayRain(x + 60, y + 115);
    x = x + 65;
    y = y + 65;
  }

  if      (icon == "01d" || icon == "01n")      sunnyIcon(x, y,         large_size, icon);
  else if (icon == "02d" || icon == "02n")      mostlySunnyIcon(x, y,   large_size, icon);
  else if (icon == "03d" || icon == "03n")      cloudyIcon(x, y,        large_size, icon);
  else if (icon == "04d" || icon == "04n")      mostlySunnyIcon(x, y,   large_size, icon);
  else if (icon == "09d" || icon == "09n")      chanceOfRainIcon(x, y,  large_size, icon);
  else if (icon == "10d" || icon == "10n")      rainIcon(x, y,          large_size, icon);
  else if (icon == "11d" || icon == "11n")      thunderStormIcon(x, y,  large_size, icon);
  else if (icon == "13d" || icon == "13n")      snowIcon(x, y,          large_size, icon);
  else if (icon == "50d")                       hazeIcon(x, y,          large_size, icon);
  else if (icon == "50n")                       fogIcon(x, y,           large_size, icon);
  else                                          noData(x, y,            large_size);
}

void displayPressureAndTrend(int x, int y, float pressure, String slope) {
  display.setFont(&DSEG7_Classic_Bold_21);
  drawString(x - 35, y - 95, String(pressure, 0), LEFT);    // metric
  display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x + 36, y - 90, "hPa", LEFT);
  if (slope == "0") display.drawInvertedBitmap(x + 60, y - 96, FL_Arrow, 18, 18, GxEPD_BLACK); // Steady
  if (slope == "-") display.drawInvertedBitmap(x + 60, y - 96, DN_Arrow, 18, 18, GxEPD_BLACK); // Falling
  if (slope == "+") display.drawInvertedBitmap(x + 60, y - 96, UP_Arrow, 18, 18, GxEPD_BLACK); // Rising
}

void displayRain(int x, int y) {
  if (forecast[1].rain > 0) drawString(x, y, String(forecast[1].rain, (forecast[1].rain > 0.5 ? 2 : 3)) + "mm Rain", CENTER); // Only display rainfall if > 0
}

void mostlySunnyIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 17;
  }
  int linesize = 3;
  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addCloud(x, y + offset, scale, linesize);
  addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size);
}

void sunnyIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 10;
  }
  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  scale = scale * 1.5;
  addSun(x, y + offset, scale, large_size);
}

void cloudyIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 12;
  }
  int linesize = 3;
  if (scale == SMALL) {
    if (icon_name.endsWith("n")) 
      addMoon(x, y + offset, scale);

    linesize = 1;
    addCloud(x, y + offset, scale, linesize);
  }
  else {
    if (icon_name.endsWith("n")) 
      addMoon(x, y + offset, scale);

    addCloud(x + 30, y - 20 + offset, 4, linesize); // Cloud top right
    addCloud(x - 20, y - 10 + offset, 6, linesize); // Cloud top left
    addCloud(x, y + offset + 15, scale, linesize); // Main cloud
  }
}

void chanceOfRainIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 12;
  }
  int linesize = 3;
  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);
  
  addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size);
  addCloud(x, y + offset, scale, linesize);
  addRain(x, y + offset, scale);
}

void rainIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 12;
  }
  int linesize = 3;
  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addCloud(x, y + offset, scale, linesize);
  addRain(x, y + offset, scale);
}

void thunderStormIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 12;
  }
  int linesize = 3;
  if (scale == SMALL) 
    linesize = 1;
  
  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addCloud(x, y + offset, scale, linesize);
  addThunderStorm(x, y + offset, scale);
}

void snowIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 12;
  }

  int linesize = 3;

  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addCloud(x, y + offset, scale, linesize);
  addSnow(x, y + offset, scale);
}

void fogIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;

  if (large_size) {
    scale = LARGE;
    offset = 12;
  }

  int linesize = 3;

  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addCloud(x, y + offset, scale, linesize);
  addFog(x, y + offset, scale, linesize);
}

void hazeIcon(int x, int y, bool large_size, String icon_name) {
  int scale = SMALL, offset = 0;
  
  if (large_size) {
    scale = LARGE;
    offset = 7;
  }

  int linesize = 3;

  if (scale == SMALL) 
    linesize = 1;

  if (icon_name.endsWith("n")) 
    addMoon(x, y + offset, scale);

  addSun(x, y + offset, scale * 1.4, large_size);
  addFog(x, y + offset, scale * 1.4, linesize);
}

void addMoon (int x, int y, int scale) {
  if (scale == LARGE) {
    display.fillCircle(x - 37, y - 33, scale, GxEPD_BLACK);
    display.fillCircle(x - 27, y - 33, scale * 1.6, GxEPD_WHITE);
  }
  else
  {
    display.fillCircle(x - 20, y - 15, scale, GxEPD_BLACK);
    display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_WHITE);
  }
}

void addSun(int x, int y, int scale, boolean icon_size) {
  int linesize = 3;
  if (icon_size == small_icon) linesize = 1;
  int dxo, dyo, dxi, dyi;
  display.fillCircle(x, y, scale, GxEPD_BLACK);
  display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
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

void addRain(int x, int y, int scale) {
  for (byte i = 0; i < 6; i++) {
    display.fillCircle(x - scale * 4 + scale * i * 1.3, y + scale * 1.9 + (scale == SMALL ? 3 : 0), scale / 3, GxEPD_BLACK);
    arrow(x - scale * 4 + scale * i * 1.3 + (scale == SMALL ? 6 : 4), y + scale * 1.6 + (scale == SMALL ? -3 : -1), scale / 6, 40, scale / 1.6, scale * 1.2);
  }
}

void addSnow(int x, int y, int scale) {
  int dxo, dyo, dxi, dyi;
  for (byte flakes = 0; flakes < 5; flakes++) {
    for (int i = 0; i < 360; i = i + 45) {
      dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.1;
      dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.1;
      display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3, dyo + y + scale * 2, dxi + x + 0 + flakes * 1.5 * scale - scale * 3, dyi + y + scale * 2, GxEPD_BLACK);
    }
  }
}

void addThunderStorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (byte i = 0; i < 5; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, GxEPD_BLACK);
    if (scale != SMALL) {
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, GxEPD_BLACK);
    }
    display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, GxEPD_BLACK);
    if (scale != SMALL) {
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, GxEPD_BLACK);
    }
    display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, GxEPD_BLACK);
    if (scale != SMALL) {
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, GxEPD_BLACK);
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, GxEPD_BLACK);
    }
  }
}

void addFog(int x, int y, int scale, int linesize) {
  if (scale == SMALL) 
    y -= 10;

  if (scale == SMALL) 
    linesize = 1;
  
  for (byte i = 0; i < 6; i++) {
    display.fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, GxEPD_BLACK);
  }
}

void noData(int x, int y, bool large_size) {
  int scale = SMALL, offset = 0;
  if (large_size) {
    scale = LARGE;
    offset = 7;
  }
  if (scale == LARGE)  
    display.setFont(&FreeMonoBold12pt7b); 
  else 
    display.setFont(&DejaVu_Sans_Bold_11);

  drawString(x - 20, y - 10 + offset, "N/A", LEFT);
}

void drawString(int x, int y, String text, alignment align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  display.setTextWrap(false);
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  display.setCursor(x, y + h);
  display.print(text);
}