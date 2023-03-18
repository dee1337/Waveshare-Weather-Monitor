/***
 * ESP32 weather monitor application using a 3 colour Waveshare 4.2" e-ink display. Weather
 * data is obtained from Open Weather Map
 *
 * Application was written using VSCode and platformio to manage the project.  Graphics library used
 * for the e-ink display is GxEPD2.
 *
 * v1: Near copy of project by G6EJD.
 * v2: Modify display to remove weather person and re-arrange information.
 * v3: Yet another different display setup.
 * 
 * v3.1: Version using Lilygo ESP32S3 T7-S3
 * Waveshare        ESP32S3
 *   DIN               11 (SPI MOSI)
 *   CLK               12 (SPI SCK)
 *   CS                10 (SPI chip selection)
 *   DC                18
 *   RST               16
 *   BUSY              15
 */
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_adc_cal.h"    // So we can read the battery voltage

#include "GxEPD2_GFX.h"
#include "GxEPD2_3C.h"                          // 3 colour screen
#include "GxEPD2_display_selection_new_style.h" // For selecting screen

#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "config.h"
#include "fonts.h"
#include "arrow.h"
#include "sunrise.h"
#include "sunset.h"
#include "OpenSans_Regular24pt7b.h"
#include "OpenSans_Regular18pt7b.h"

// CaptureLog setup
#define CLOG_ENABLE false                        // this must be defined before cLog.h is included 
#include "cLog.h"

#if CLOG_ENABLE
const uint16_t maxEntries = 20;
const uint16_t maxEntryChars = 50;
CLOG_NEW myLog1(maxEntries, maxEntryChars, NO_TRIGGER, NO_WRAP);
#endif

// T7-S3 power LED pin which we can turn off to save power
#define LED_PIN 17
//#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

// Battery voltage pin
#define BAT_ADC 2
const float LOW_BATTERY_VOLTAGE = 3.40; // warn user battery low!

/* Constants/defines */
const uint SCREEN_WIDTH = 400;
const uint SCREEN_HEIGHT = 300;
const String VERSION = "v3.1";
const String Hemisphere = "north";
const int forecast_counter = 16; // Number of forecasts to get/show.

const long sleep_duration = 30; // Number of minutes to go to sleep for
const int sleep_hour = 23;      // Start power saving at 23:00
const int wakeup_hour = 8;      // Stop power saving at 08:00

#define LARGE 10
#define SMALL 4

float battery_voltage = 0.0;

boolean large_icon = true;
boolean small_icon = false;

enum alignment {
    LEFT,
    RIGHT,
    CENTER
};
// enum pressure_trend {LEVEL, UP, DOWN};
enum sun_direction {
    SUN_UP,
    SUN_DOWN
};
enum star_size {
    SMALL_STAR,
    MEDIUM_STAR,
    LARGE_STAR
};

/* Function prototypes */
bool getTodaysWeather(void);
bool getWeatherForecast(void);
bool getDailyWeatherForecast(void);
static void updateLocalTime(void);
void initialiseDisplay(void);
void goToSleep(void);
static uint32_t readADC_Cal(const int adc_raw);
uint32_t getBatteryVoltage(void);
int calculateBatteryPercentage(double v);
void displayBattery(int x, int y);
void logWakeupReason(void);
void displayErrorMessage(String message);
void displayWifiErrorMessage(void);
void drawString(int x, int y, String text, alignment align);
void displayInformation(void);
void displayTemperature(int x, int y);
void displayCloudCover(int x, int y, int cover);
void addCloud(int x, int y, int scale, int linesize);
void displaySystemInfo(int x, int y);
void displayWind(int x, int y, float angle, float windspeed, int radius);
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength, uint16_t colour);
String windDegToDirection(float winddirection);
String titleCase(String text);
void displayWeatherDescription(int x, int y);
String convertUnixTime(uint32_t unix_time);
void displaySunAndMoon(int x, int y);
//int julianDate(int d, int m, int y);
void displayWeatherForecast(int x, int y);
void displaySingleForecast(int x, int y, int offset, int index);
void displayWeatherIcon(int x, int y, String icon, bool icon_size);
void addMoon(int x, int y, int scale);
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
void mistIcon(int x, int y, bool large_size, String icon_name);
// void fogIcon(int x, int y, bool large_size, String icon_name);
// void hazeIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
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
    uint8_t humidity = 0;
    uint8_t clouds = 0;
    uint16_t wind_deg = 0;
    uint32_t dt = 0;
    uint32_t sunrise = 0;
    uint32_t sunset = 0;
    uint32_t visibility = 0;
    float temperature = 0;
    float high = 0;
    float low = 0;
    float feels_like = 0;
    float pressure = 0;
    float dew_point = 0;
    float uvi = 0;
    float wind_speed = 0;
    float wind_gust = 0;
    float rain = 0;
    float snow = 0;
    String main;
    String description;
    String icon;
    String period;
} WeatherStruct;

WeatherStruct weather;
WeatherStruct forecast[forecast_counter];

char timeStringBuff[7]; // buffer for time on the display
char dateStringBuff[4];
char dayStringBuff[10];

void setup() {
    int wifi_connect_counter = 0;
    bool wifi_connected = true;
    
	// Ensure power LED is off to save power.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    #if CLOG_ENABLE
    Serial.begin(115200);
    delay(5000); // delay for serial to begin, T7-S3 is very slow to start serial output!
	
	logWakeupReason();
    #endif

    initialiseDisplay();

    // Serial.println("\n##################################");
    // Serial.println(F("ESP32 Information:"));
    // Serial.printf("Internal Total Heap %d, Internal Used Heap %d, Internal Free Heap %d\n", ESP.getHeapSize(), ESP.getHeapSize()-ESP.getFreeHeap(), ESP.getFreeHeap());
    // Serial.printf("Sketch Size %d, Free Sketch Space %d\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
    // Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    // Serial.printf("Chip Model %s, ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    // Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
    // Serial.println("##################################\n");

    WiFi.disconnect();

    WiFi.mode(WIFI_STA); // switch off AP
    WiFi.setAutoReconnect(true);

    WiFi.begin(SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        // Serial.print(".");

        wifi_connect_counter++;

        // Give the wifi a few seconds to connect
        if (wifi_connect_counter > 30) {
            wifi_connected = false;
            break;
        }
    }
    
    if (wifi_connected) {
        // Serial.println("");
        // Serial.println("Connecting to Wi-Fi...");

        ipAddress = WiFi.localIP().toString();
        rssi = WiFi.RSSI();
        //Serial.println(ipAddress);
        CLOG(myLog1.add(), "IP Address: %s", ipAddress);

        //Serial.println("Connecting to NTP Time Server...");
        configTime(0, 0, SNTP_TIME_SERVER);
        updateLocalTime();

        //Serial.println("All set up, display some information...");
        CLOG(myLog1.add(), "Setup complete...");

        bool today_flag = getTodaysWeather();
        bool forecast_flag = getWeatherForecast();

        // Turn off wifi to save power
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);

        battery_voltage = getBatteryVoltage();

        if (today_flag == true && forecast_flag == true)
        {
            CLOG(myLog1.add(), "All data retrieved successfully.");
            displayInformation();
        }
        else
        {
            CLOG(myLog1.add(), "Battery: %.2fV", battery_voltage);
            CLOG(myLog1.add(), "Unable to retrieve data!");
            displayErrorMessage("Unable to retrieve data, contact support!");
        }
    } else {
        CLOG(myLog1.add(), "Unable to connect to wifi!");
        displayWifiErrorMessage();
    }

    goToSleep();
}

/**
 * @brief Main loop which we will never enter as we enter deep sleep once we've updated
 * the display.
 * 
 */
void loop() {
    while (true) {
        /*
         * Should never get here - using deep sleep
         */
    }
}


/**
 * @brief Put the chip to sleep
 * 
 */
void goToSleep(void) {
    long sleep_timer = sleep_duration * 60;

    struct tm timeinfo;

    display.powerOff(); // should be in hibernate but no harm tuning it off

    getLocalTime(&timeinfo);

    if (timeinfo.tm_hour >= sleep_hour || timeinfo.tm_hour < wakeup_hour)
    {
        sleep_timer = 7200;

        if (timeinfo.tm_hour < wakeup_hour && (timeinfo.tm_hour + (sleep_timer / 3600)) > wakeup_hour)
        {
            sleep_timer = (wakeup_hour * 3600) - ((timeinfo.tm_hour * 3600) + (timeinfo.tm_min * 60) + timeinfo.tm_sec);
            if (sleep_timer < (sleep_duration * 60))
            {
                sleep_timer = sleep_timer + sleep_duration * 60;
            }
        }
    }

    if (sleep_timer < sleep_duration * 60)
    {
        sleep_timer = sleep_duration * 60;
    }
    esp_sleep_enable_timer_wakeup(sleep_timer * 1000000LL);

    CLOG(myLog1.add(), "Off to deep-sleep for %ld minutes", sleep_timer/60);

    #if CLOG_ENABLE
    Serial.println("");
    Serial.println("## This is myLog1 ##");
    for (uint16_t i = 0; i < myLog1.numEntries; i++) {
        Serial.println(myLog1.get(i));
    }
    delay(3000);  // serial output seems to be slow!
    #endif

    esp_deep_sleep_start();
}

/**
 * @brief Log (cLog) why we've woken up.
 * 
 */
void logWakeupReason(void) {
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : CLOG(myLog1.add(), "Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : CLOG(myLog1.add(), "Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : CLOG(myLog1.add(), "Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : CLOG(myLog1.add(), "Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : CLOG(myLog1.add(), "Wakeup caused by ULP program"); break;
    default : CLOG(myLog1.add(), "Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
    }
}

/**
 * @brief Get the battery voltage
 * 
 * @return uint32_t Battery voltage, e.g. 3.9v
 */
uint32_t getBatteryVoltage(void)
{
    float v = 0.0;
    v = (readADC_Cal(analogRead(BAT_ADC))) * 2;
    CLOG(myLog1.add(), "getBatteryVoltage: %f", v);
    return v;
}

/**
 * @brief Get the battery voltage via the battery pin
 * 
 * @param adc_raw Raw battery voltage from an adc read.
 * @return uint32_t Battery voltage, e.g. 3999.0
 */
static uint32_t readADC_Cal(const int adc_raw)
{
    esp_adc_cal_characteristics_t adc_chars;

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    return (esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars));
}

/**
 * @brief Calculate the appromimate battery life percentage remaining. Returns a value 
 * between 0-100% rounded to the nearest integer.
 * 
 * @param v Voltage reading of the battery.
 * @return int Percentage remaining
 */
int calculateBatteryPercentage(double v)
{
  // this formula was calculated using samples collected from a lipo battery
  double y = -  144.9390 * v * v * v
             + 1655.8629 * v * v
             - 6158.8520 * v
             + 7501.3202;

  // enforce bounds, 0-100
  y = max(y, 0.0);
  y = min(y, 100.0);
  
  y = round(y);
  return static_cast<int>(y);
} 

/**
 * @brief Update global buffers with various times/date values.
 * 
 */
static void updateLocalTime(void)
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        //Serial.println("Failed to obtain time");
        CLOG(myLog1.add(), "Failed to obtain time");
        return;
    }

    // Update buffer with current time/date
    strftime(dateStringBuff, sizeof(dateStringBuff), "%e", &timeinfo);    // 1-9. 10-31
    strftime(dayStringBuff, sizeof(dayStringBuff), "%A", &timeinfo);      // Saturday
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo); // 15:15
}

/**
 * @brief Get the Todays Weather from openweathermaps.org
 * 
 * @return true If we successfully retrieved the weather
 * @return false If we failed to retrieve the weather
 */
bool getTodaysWeather(void)
{
    WiFiClientSecure client;
    bool retcode = true;
    int port = 443;
    const char *host = "api.openweathermap.org";

    uint32_t dt = millis();

    client.setInsecure(); // certificate is not checked

    if (!client.connect(host, port)) {
        CLOG(myLog1.add(), "HTTPS[1] connection to OpenWeatherMap failed!");
        return false;
    }

    uint32_t timeout = millis();
    char c = 0;

    // Send GET request
    client.print(String("GET ") + WEATHER_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

    while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
            //Serial.println("Header end found.");
            break;
        }

        Serial.println(line);

        if ((millis() - timeout) > 5000UL) {
            //Serial.println("HTTP header timeout");
            client.stop();
            return false;
        }
    }

    //Serial.print("JSON length: "); Serial.println(client.available());
    //Serial.println("Parsing JSON...");

    // bool decode = false;
    DynamicJsonDocument doc(20 * 1024);

    //Serial.println("Deserialization process starting...");

    // Parse JSON object
    DeserializationError err = deserializeJson(doc, client);
    if (err) {

        CLOG(myLog1.add(), "deserializeJson(1) failed: %s", err.c_str());
        retcode = false;
    }
    else {
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

        CLOG(myLog1.add(), "Deserialized today's weather in %ld ms", millis() - dt);
    }

    client.stop();

    return retcode;
}

/**
 * @brief Get the Weather Forecast for the next 'n' readings. Readings are for every 3 hours
 * and the number to retrieve is set in a global variable 'forecast_counter'.
 * 
 * @return true 
 * @return false 
 */
bool getWeatherForecast(void)
{
    WiFiClientSecure client;
    bool retcode = true;
    int port = 443;
    const char *host = "api.openweathermap.org";

    uint32_t dt = millis();

    client.setInsecure(); // certificate is not checked

    if (!client.connect(host, port)) {
        CLOG(myLog1.add(), "HTTPS[2] connection to OpenWeatherMap failed!");

        return false;
    }

    uint32_t timeout = millis();
    char c = 0;

    // Send GET request
    client.print(String("GET ") + FORECAST_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

    while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
            //Serial.println("Header end found.");
            break;
        }

        if ((millis() - timeout) > 5000UL) {
            //Serial.println("HTTP header timeout");
            client.stop();
            return false;
        }
    }

    //Serial.println("Parsing Forecast JSON...");

    DynamicJsonDocument doc(24 * 1024);

    //Serial.println("Deserialization process starting...");

    // Parse JSON object
    DeserializationError err = deserializeJson(doc, client);
    if (err) {
        CLOG(myLog1.add(), "deserializeJson(2) failed: %s", err.c_str());

        retcode = false;
    }
    else {
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

        CLOG(myLog1.add(), "Deserialized [%d] forecasts in %ld ms", forecast_counter, millis() - dt);
    }

    client.stop();

    return retcode;
}

/**
 * @brief Set up the display, serial connection, rotation, font, colour, and size.
 * 
 */
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

/**
 * @brief Display an error message on the display to the user.
 * 
 * @param message Message to display
 */
void displayErrorMessage(String message) {
    display.setFullWindow();
    display.firstPage();

    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(10, 60);
        drawString(200, 150, "Error: " + message, CENTER);
    } while (display.nextPage());

    display.hibernate();
}

/**
 * @brief Display an error message on the display to the user.
 * 
 * @param message Message to display
 */
void displayWifiErrorMessage(void)
{
    display.setFullWindow();
    display.firstPage();

    do
    {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(10, 60);
        drawString(200, 60, "Error: Unable to connect to wifi network.", CENTER);
        display.setTextColor(GxEPD_RED);
        drawString(200, 85, SSID, CENTER);
        display.setTextColor(GxEPD_BLACK);
        drawString(30, 130, "a) Check wifi network is on.", LEFT);
        drawString(30, 150, "b) Reboot display via on/off or reset button.", LEFT);
        drawString(30, 170, "c) Move display closer to the router.", LEFT);
        drawString(30, 190, "d) Contact support!", LEFT);

        battery_voltage = getBatteryVoltage(); 
        displayBattery(304, 279);
    } while (display.nextPage());

    display.hibernate();
}

/**
 * @brief Set up the display with all of the information we're going to show, e.g. temperature,
 * forecast, sun rise and set times...
 * 
 */
void displayInformation(void)
{
    uint32_t dt = millis();

    display.setFullWindow();
    display.firstPage();
//    do {
        display.fillScreen(GxEPD_WHITE);

        // draw box lines
        // top
        display.drawLine(0, 0, 145, 0, GxEPD_BLACK);
        display.drawLine(147, 0, 276, 0, GxEPD_BLACK);
        display.drawLine(278, 0, 399, 0, GxEPD_BLACK);

        // right
        display.drawLine(399, 0, 399, 150, GxEPD_BLACK);
        display.drawLine(399, 152, 399, 180, GxEPD_BLACK);
        display.drawLine(399, 182, 399, 299, GxEPD_BLACK);

        // bottom
        display.drawLine(0, 299, 262, 299, GxEPD_BLACK);
        display.drawLine(264, 299, 399, 299, GxEPD_BLACK);

        // left
        display.drawLine(0, 0, 0, 150, GxEPD_BLACK);
        display.drawLine(0, 152, 0, 180, GxEPD_BLACK);
        display.drawLine(0, 182, 0, 299, GxEPD_BLACK);

        // lines between temp/icon/wind
        display.drawLine(145, 0, 145, 109, GxEPD_BLACK);
        display.drawLine(147, 0, 147, 109, GxEPD_BLACK);
        display.drawLine(276, 0, 276, 109, GxEPD_BLACK);
        display.drawLine(278, 0, 278, 109, GxEPD_BLACK);

        // line after the two graphs
        display.drawLine(262, 182, 262, 299, GxEPD_BLACK);
        display.drawLine(264, 182, 264, 299, GxEPD_BLACK);

        // top middle lines
        display.drawLine(0, 110, 145, 110, GxEPD_BLACK); //x=125
        display.drawLine(147, 110, 399, 110, GxEPD_BLACK); //x=125
        display.drawLine(0, 112, 119, 112, GxEPD_BLACK); //x=125
        display.drawLine(121, 112, 399, 112, GxEPD_BLACK); //x=125

        // lines between sun and forecasts
        display.drawLine(119, 112, 119, 180, GxEPD_BLACK);
        display.drawLine(121, 112, 121, 180, GxEPD_BLACK);
        
        // bottom middle lines
        display.drawLine(0, 180, 119, 180, GxEPD_BLACK);  // x=125
        display.drawLine(121, 180, 399, 180, GxEPD_BLACK);  // x=125


        display.drawLine(0, 182, 262, 182, GxEPD_BLACK);  // x=125
        display.drawLine(264, 182, 399, 182, GxEPD_BLACK);  // x=125

        displayTemperature(0, 0);
        displayWeatherIcon(146, -13, weather.icon, large_icon); // Weather icon
        if (weather.clouds > 0)
        {
            displayCloudCover(196, 11, weather.clouds);
        }
        displayWeatherDescription(212, 92);                     // Description of the weather now
        displayWind(325, 39, weather.wind_deg, weather.wind_speed, 40); // Wind direction info
        displaySystemInfo(295, 185);
        displaySunAndMoon(2, 114); // Sunset and sunrise and moon state with icons
        displayWeatherForecast(118, 115);                               // Forecast

//    } while (display.nextPage());
    display.display(false);  // works instead of do:while loop
    delay(500);
    
    display.hibernate();

    CLOG(myLog1.add(), "Display updated in %ld seconds", (millis() - dt) / 1000);
}

/**
 * @brief Display the sytem information;
 * date, time of reading the weather, version, rssi etc.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displaySystemInfo(int x, int y) {
    int wifi_rssi = 0;
    int xpos = 1;
    
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(0);

    display.setFont(&OpenSans_Regular18pt7b);
    drawString(x + 95, y + 5, dateStringBuff, RIGHT);

    display.setFont(&DejaVu_Sans_Bold_11);

    drawString(x - 26, y + 5, dayStringBuff, LEFT);
    
    drawString(x - 26, y + 18, "@", LEFT);
    drawString(x - 14, y + 21, timeStringBuff, LEFT);

    int rssi_x = x - 5;
    int rssi_y = y + 70;
    for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20)
    {
        if (_rssi <= -20)
            wifi_rssi = 20; //  <-20dbm displays 5-bars
        if (_rssi <= -40)
            wifi_rssi = 16; //  -40dbm to  -21dbm displays 4-bars
        if (_rssi <= -60)
            wifi_rssi = 12; //  -60dbm to  -41dbm displays 3-bars
        if (_rssi <= -80)
            wifi_rssi = 8; //  -80dbm to  -61dbm displays 2-bars
        if (_rssi <= -100)
            wifi_rssi = 4; // -100dbm to  -81dbm displays 1-bar
        display.fillRect(rssi_x + xpos * 5 + 60, rssi_y - wifi_rssi, 4, wifi_rssi, GxEPD_BLACK);
        xpos++;
    }

    display.fillRect(rssi_x + 60, rssi_y - 1, 4, 1, GxEPD_BLACK);
    drawString(rssi_x, rssi_y - 9, String(rssi) + "dBm", LEFT);

    drawString(x + 37, y + 80, ipAddress, CENTER);

    displayBattery(x + 7, y + 93);
}

/**
 * @brief Display the current temperature, cloud cover and humidity.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayTemperature(int x, int y) {
     int x_offset = 8;

    display.setFont(&DSEG7_Classic_Bold_21);
    display.setTextSize(2);

    // Center the tempearature in the weather box area
    if (weather.temperature < 0)
    {
        drawString(x + x_offset, y + 61, "-", LEFT);                                       // Show temperature sign to compensate for non-proportional font spacing
        drawString(x + x_offset + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + x_offset + 95, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }
    else if (weather.temperature < 10)
    {
        drawString(x + x_offset + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + x_offset + 95, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }
    else if (weather.temperature < 20)
    {
        drawString(x, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + 105, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }
    else
    {
        drawString(x + x_offset + 5, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + x_offset + 110, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }

    String buffer = String(weather.low, 0) + "'/" + String(weather.high, 0) + "'";
    drawString(x + 70, y + 82, buffer, CENTER); // Show forecast high and Low, in the font ' is a °

    display.setFont(&DejaVu_Sans_Bold_11);

    drawString(x + 73, y + 4, String(weather.humidity) + "% RH", CENTER);
}

/**
 * @brief Current weather description as returned by openweathermap.org
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayWeatherDescription(int x, int y) {
    drawString(x, y, titleCase(weather.description), CENTER);
}

// /**
//  * @brief Display system information which could include battery voltage, wi-fi signal
//  * strength and the IP address we've been assigned depending on screen space
//  * 
//  * @param x Display x coordinates
//  * @param y Display y coordinates
//  */
// void displaySystemInfo(int x, int y) {
//     int wifi_rssi = 0;
//     int xpos = 1;

//     display.drawRect(x, y, 135, 62, GxEPD_BLACK);

//     int rssi_x = x + 33;
//     int rssi_y = y + 38;
//     for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) {
//         if (_rssi <= -20)
//             wifi_rssi = 20; //  <-20dbm displays 5-bars
//         if (_rssi <= -40)
//             wifi_rssi = 16; //  -40dbm to  -21dbm displays 4-bars
//         if (_rssi <= -60)
//             wifi_rssi = 12; //  -60dbm to  -41dbm displays 3-bars
//         if (_rssi <= -80)
//             wifi_rssi = 8; //  -80dbm to  -61dbm displays 2-bars
//         if (_rssi <= -100)
//             wifi_rssi = 4; // -100dbm to  -81dbm displays 1-bar
//         display.fillRect(rssi_x + xpos * 5 + 60, rssi_y - wifi_rssi, 4, wifi_rssi, GxEPD_BLACK);
//         xpos++;
//     }

//     display.fillRect(rssi_x + 60, rssi_y - 1, 4, 1, GxEPD_BLACK);
//     drawString(rssi_x, rssi_y - 9, String(rssi) + "dBm", LEFT);

//     drawString(x + 68, y + 45, WiFi.localIP().toString(), CENTER);
// }

/**
 * @brief Display the battery voltage and percentage - we need 3.3v to drive 
 * the waveshare e-ink display so percentage will display 0% when we get that
 * low even though the ESP32S3 is capable of being run at 2.2v.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayBattery(int x, int y) {
    int percentage = 0;
    int p = 0;
    int colour = GxEPD_BLACK;
    float bv = battery_voltage/1000;

    if (bv >= 3 ) { 
        //p = 2836.9625 * pow(battery_voltage, 4) - 43987.4889 * pow(battery_voltage, 3) + 255233.8134 * pow(battery_voltage, 2) - 656689.7123 * battery_voltage + 632041.7303;
        percentage = calculateBatteryPercentage(bv);
        CLOG(myLog1.add(), "Battery voltage: %.2f, percentage: %d", bv, percentage);

        int offset = 6;
        display.drawRect(x + 9 + offset, y + 5, 34, 10, GxEPD_BLACK);
        display.fillRect(x + 43 + offset, y + 7, 2, 6, GxEPD_BLACK);

        if (bv <= LOW_BATTERY_VOLTAGE || percentage < 10) {
            display.setTextColor(GxEPD_RED);
            display.fillRect(x + 11 + offset, y + 7, 31 * percentage / 100.0, 6, GxEPD_RED);
        } else {
            display.fillRect(x + 11 + offset, y + 7, 31 * percentage / 100.0, 6, GxEPD_BLACK);
        }

        // draw lines to give a better battery icon
        // 25% = 7
        // 50% = 15
        // 75% = 23
        // 100% = 
        display.fillRect((x + 11 + offset) + 7, y + 6, 1, 8, GxEPD_WHITE);  // 25% across
        display.fillRect((x + 11 + offset) + 15, y + 6, 1, 8, GxEPD_WHITE);  // 50% across
        display.fillRect((x + 11 + offset) + 23, y + 6, 1, 8, GxEPD_WHITE);  // 75% across
        display.fillRect((x + 11 + offset) + 30, y + 6, 1, 8, GxEPD_WHITE);  // 100% across

        // display.setTextColor(colour);
        drawString(x + 55, y + 6, String(percentage) + "%", LEFT);
        drawString(x - 29, y + 6,  String(bv, 2) + "v", LEFT);
    } 
    else
    {
        CLOG(myLog1.add(), "Battery voltage: %.2f, recharge now!", bv);
        display.setTextColor(GxEPD_RED);
        drawString(x + 4, y - 1, "Recharge Battery", LEFT);
        display.setTextColor(GxEPD_BLACK);
    }
}


/**
 * @brief Display some small clouds and the percentage value of the cloud cover. 0% is
 * no cloud cover, 100% is total cover.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param cover The percentage of the current cloud cover to display.
 */
void displayCloudCover(int x, int y, int cover) {
    addCloud(x, y, SMALL * 0.5, 1);          // Main cloud
    addCloud(x + 5, y - 5, SMALL * 0.35, 1); // Cloud top right
    addCloud(x - 8, y - 5, SMALL * 0.35, 1); // Cloud top left
    drawString(x + 30, y - 5, String(cover) + "%", CENTER);
}

/**
 * @brief Title case the first word in the passed string.
 *
 * @param text    String to title case.
 * @return String Returned title cased or empty string if text length is 0.
 */
String titleCase(String text) {
    if (text.length() > 0)
    {
        String temp_text = text.substring(0, 1);
        temp_text.toUpperCase();
        return temp_text + text.substring(1); // Title-case the string
    }
    return "";
}

/**
 * @brief Display the wind direction and force using a compass display.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param angle Angle of the wind
 * @param windspeed Wind speed in mph
 * @param radius Radius of the compass in pixels
 */
void displayWind(int x, int y, float angle, float windspeed, int radius) {
    int offset = 16;
    int dxo;
    int dyo;
    int dxi;
    int dyi;

    arrow(x + offset, y + offset, radius - 11, angle, 15, 22, GxEPD_RED); // Show wind direction on outer circle of width and length
    display.setTextSize(0);

    display.drawCircle(x + offset, y + offset, radius, GxEPD_BLACK);       // Draw compass circle
    display.drawCircle(x + offset, y + offset, radius + 1, GxEPD_BLACK);   // Draw compass circle
    display.drawCircle(x + offset, y + offset, radius * 0.7, GxEPD_BLACK); // Draw compass inner circle
    for (float a = 0; a < 360; a = a + 22.5) {
        dxo = radius * cos((a - 90) * PI / 180);
        dyo = radius * sin((a - 90) * PI / 180);
        if (a == 45)
            drawString(dxo + x + 10 + offset, dyo + y - 10 + offset, "NE", CENTER);
        if (a == 135)
            drawString(dxo + x + 12 + offset, dyo + y + offset, "SE", CENTER);
        if (a == 225)
            drawString(dxo + x - 16 + offset, dyo + y + offset, "SW", CENTER);
        if (a == 315)
            drawString(dxo + x - 12 + offset, dyo + y - 10 + offset, "NW", CENTER);
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

    display.setFont(); // use default 6x8 font
    drawString(x + offset + 3, y - 15 + offset, "mph", CENTER);

    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(x + offset, y + 10 + offset, String(angle, 0) + "'", CENTER);
}

/**
 * @brief Draw the arrow of the compass used in displaying the wind direction.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param asize Arrow size in pixels
 * @param aangle Arrow angle
 * @param pwidth Arrow width
 * @param plength Arrow length
 * @param colour Colour to draw/fill the arrow
 */
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength, uint16_t colour) {
    float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
    float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
    float x1 = 0;
    float y1 = plength;
    float x2 = pwidth / 2;
    float y2 = pwidth / 2;
    float x3 = -pwidth / 2;
    float y3 = pwidth / 2;
    float angle = aangle * PI / 180 - 135;
    float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
    float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
    float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
    float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
    float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
    float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
    display.fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, colour);
}

/**
 * @brief Return the wind direction as a compass bearing, e.g. 0 = North.
 * 
 * @param winddirection Wind direction in degress
 * @return String Wind direction as a compass bearing
 */
String windDegToDirection(float winddirection) {
    if (winddirection >= 348.75 || winddirection < 11.25)
        return "N";

    if (winddirection >= 11.25 && winddirection < 33.75)
        return "NNE";

    if (winddirection >= 33.75 && winddirection < 56.25)
        return "NE";

    if (winddirection >= 56.25 && winddirection < 78.75)
        return "ENE";

    if (winddirection >= 78.75 && winddirection < 101.25)
        return "E";

    if (winddirection >= 101.25 && winddirection < 123.75)
        return "ESE";

    if (winddirection >= 123.75 && winddirection < 146.25)
        return "SE";

    if (winddirection >= 146.25 && winddirection < 168.75)
        return "SSE";

    if (winddirection >= 168.75 && winddirection < 191.25)
        return "S";

    if (winddirection >= 191.25 && winddirection < 213.75)
        return "SSW";

    if (winddirection >= 213.75 && winddirection < 236.25)
        return "SW";

    if (winddirection >= 236.25 && winddirection < 258.75)
        return "WSW";

    if (winddirection >= 258.75 && winddirection < 281.25)
        return "W";

    if (winddirection >= 281.25 && winddirection < 303.75)
        return "WNW";

    if (winddirection >= 303.75 && winddirection < 326.25)
        return "NW";

    if (winddirection >= 326.25 && winddirection < 348.75)
        return "NNW";

    return "?";
}

/**
 * @brief Display the time when the sun rises and sets.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displaySunAndMoon(int x, int y) {
    time_t now = time(NULL);
    struct tm *now_utc = gmtime(&now);
    const int day_utc = now_utc->tm_mday;
    const int month_utc = now_utc->tm_mon + 1;
    const int year_utc = now_utc->tm_year + 1900;

    sunRiseSetIcon(x + 20, y + 20, SUN_UP);
    sunRiseSetIcon(x + 20, y + 47, SUN_DOWN);

    drawString(x + 40, y + 15, convertUnixTime(weather.sunrise).substring(0, 8), LEFT); // 08:00 AM
    drawString(x + 40, y + 42, convertUnixTime(weather.sunset).substring(0, 8), LEFT);  // 19:00 PM
}

/**
 * @brief Display the weather forcast for the next 'forecast_counter' hours.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayWeatherForecast(int x, int y) {
    int offset = 57;

    for (byte i = 0; i < 5; i++) {
        displaySingleForecast(x + offset * i, y, offset, i);
    }

    float temperature[forecast_counter] = {0};
    float pressure[forecast_counter] = {0};
    float feels_like[forecast_counter] = {0};
    // float humidity [forecast_counter] = {0};
    // float rainfall [forecast_counter] = {0};

    for (byte i = 0; i < forecast_counter; i++) {
        temperature[i] = forecast[i].temperature;
        pressure[i] = forecast[i].pressure;
        feels_like[i] = forecast[i].feels_like;
        // humidity [i] = forecast[i].humidity;
        // rainfall [i] = forecast[i].rain;
    }

    //(x, y, w, h, Data[], lengthofdata, title)
    // drawSingleGraph(155, 205, 96, 75, temperature, forecast_counter, "Temperature");

    drawSingleGraph(155, 205, 96, 75, pressure, forecast_counter, "Pressure (hPa)"); // x=295

    // drawSingleGraph(295, 205, 96, 75, humidity, forecast_counter, "Humidity (%)");
    //(x, y, w, h, Data1[], Data2[], lengthofdata, title)

    drawGraph(20, 205, 96, 75, temperature, feels_like, forecast_counter, "Temp & Feels"); //x = 155

    // drawGraph(295, 205, 96, 75, humidity, rainfall, forecast_counter, "Hum. & Rain (mm)");
}

/**
 * @brief Display a single 3 hourly forcast
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param offset Offset to apply as this is called for large and small boxes on the display
 * @param index The index of the forecast to display
 */
void displaySingleForecast(int x, int y, int offset, int index) {
    displayWeatherIcon(x + offset / 2 + 1, y + 35, forecast[index].icon, small_icon);

    drawString(x + offset / 2, y + 3, String(forecast[index].period.substring(11, 16)), CENTER);
    drawString(x + offset / 2, y + 50, String(forecast[index].high, 0) + "/" + String(forecast[index].low, 0), CENTER);
}

/**
 * @brief Convert unix time to the current time
 * 
 * @param unix_time Unix time value to convert
 * @return String Current time hh:mm AM/PM
 */
String convertUnixTime(uint32_t unix_time)
{
    // E.G. Returns '21:12 PM'
    time_t tm = unix_time;
    char output[10];

    strftime(output, sizeof(output), "%H:%M %p", gmtime(&tm));

    return output;
}

/**
 * @brief Return the julian date of a date passed in.
 * 
 * @param d Day
 * @param m Month
 * @param y Year
 * @return int Julian date (0-366)
 */
// int julianDate(int d, int m, int y) {
//     int mm, yy, k1, k2, k3, j;

//     yy = y - (int)((12 - m) / 10);
//     mm = m + 9;

//     if (mm >= 12)
//         mm = mm - 12;

//     k1 = (int)(365.25 * (yy + 4712));
//     k2 = (int)(30.6001 * mm + 0.5);
//     k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
//     // 'j' for dates in Julian calendar:
//     j = k1 + k2 + d + 59 + 1;

//     if (j > 2299160) {
//         j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
//     }

//     return j;
// }


/**
 * @brief Display the current weather as an icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param icon Icon to display as set by openweathermap.org
 * @param large_icon If this is a large icon or not
 */
void displayWeatherIcon(int x, int y, String icon, bool large_icon) {
    if (large_icon) { // == large icon, TODO: need to change this logic, variable name!
        x = x + 65;
        y = y + 65;
    }

    if (icon == "01d") { // sun
        sunnyIcon(x, y, large_icon, icon, GxEPD_RED);
    } else if (icon == "01n") {
        sunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
    } else if (icon == "02d") { // few clouds (clouds and sun)
        mostlySunnyIcon(x, y, large_icon, icon, GxEPD_RED);
    } else if (icon == "02n") {
        mostlySunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
    } else if (icon == "03d" || icon == "03n") { // scattered clouds, no sun
        cloudyIcon(x, y, large_icon, icon);
    } else if (icon == "04d" || icon == "04n") { // broken clouds, more clouds than scatterred!
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
    } else if (icon == "50d" || icon == "50n") {
        mistIcon(x, y, large_icon, icon);
    } else {
        noData(x, y, large_icon);
    }
}

/**
 * @brief Display the sunny icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 * @param icon_color Icon colour
 */
void sunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color) {
    int scale = SMALL;
    int offset = 0;

    if (large_size) {
        scale = LARGE;
        offset = 0;
    }

    if (icon_name.endsWith("n")) { // Night time, show stars
        addMoon(x, y + offset, scale);

        if (!large_size) {
            // Small stars
            addStar(x, y, SMALL_STAR);
            addStar(x - 18, y + 3, SMALL_STAR);
            addStar(x + 17, y - 10, SMALL_STAR);

            // Medium stars
            addStar(x - 1, y - 14, MEDIUM_STAR); // top left star
            addStar(x + 10, y, MEDIUM_STAR);     // bottom right star
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
                horizontal = (int)(((rand() / (RAND_MAX * 1.0f)) * width) + left);
                vertical = (int)(((rand() / (RAND_MAX * 1.0f)) * height) + top);
                drawString(horizontal, vertical, ".", LEFT);
            }
            display.setTextColor(GxEPD_BLACK);
        }
    } else { // Day time, show sun
        scale = scale * 1.5;
        addSun(x, y + offset, scale, large_size, icon_color);
    }
}

/**
 * @brief Display the mostly sunny icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 * @param icon_color Icon colour
 */
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

    if (icon_name.endsWith("n")) { // Night time, add stars
        addMoon(x, y + offset, scale);
    } else { // Day time, add sun
        addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
    }

    if (scale == SMALL) {
        addCloud(x, y + offset, 2, linesize);
    } else {
        addCloud(x + 28, y - 18 + offset, 4, linesize); // Cloud top right
        addCloud(x - 20, y - 6 + offset, 4, linesize);  // Cloud top left  }
    }
}

/**
 * @brief Display the cloudy icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
void cloudyIcon(int x, int y, bool large_size, String icon_name) {
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size) {
        scale = LARGE;
        offset = 0;
    }

    if (scale == SMALL) {
        if (icon_name.endsWith("n")) {
            addMoon(x, y + offset, scale);
        } else {
            addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, GxEPD_RED);
        }
        linesize = 1;
        addCloud(x, y + offset, scale, linesize);
    } else {
        if (icon_name.endsWith("n")) {
            addMoon(x, y + offset, scale);
        } else { // Day time, add sun
            addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, GxEPD_RED);
        }
    }

    addCloud(x, y + offset, scale, linesize); // Main cloud
}

/**
 * @brief Display the very cloudy icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
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
        addCloud(x - 7, y - 7 + offset, 2, linesize);  // Left v.small
        addCloud(x + 8, y - 10 + offset, 2, linesize); // Right v.small
        addCloud(x, y + offset, scale, linesize);      // Main cloud
    } else {
        if (icon_name.endsWith("n")) {
            addMoon(x, y + offset, scale);
        }

        addCloud(x + 28, y - 18 + offset, 4, linesize); // Cloud top right
        addCloud(x - 20, y - 8 + offset, 6, linesize);  // Cloud top left
        addCloud(x, y + offset + 9, scale, linesize);   // Main cloud
    }
}

/**
 * @brief Display the chance of rain icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 * @param icon_color Icon colour
 */
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
    } else {
        addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
    }

    addRain(x, y + offset, scale, GxEPD_BLACK);
    addCloud(x, y + offset, scale, linesize);
}

/**
 * @brief Display the rain icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
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
    }
    
    addRain(x, y + offset, scale, GxEPD_BLACK);
    addCloud(x, y + offset, scale, linesize);
}

/**
 * @brief Display the thunderstorm icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
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
        addThunderStorm(x, y + offset, scale, GxEPD_BLACK);
    } else {
        addThunderStorm(x, y + offset, scale, GxEPD_RED);
    }

    addCloud(x, y + offset, scale, linesize);
}

/**
 * @brief Display the snow icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
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
    }
    
    addSnow(x, y + offset, scale, GxEPD_BLACK);
    addCloud(x, y + offset, scale, linesize);
}

/**
 * @brief Display the misty icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
void mistIcon(int x, int y, bool large_size, String icon_name) {
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
    } 
    
    addFog(x, y + offset, scale, linesize, GxEPD_BLACK);
}

/**
 * @brief Display the sun icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param direction Is the sun rising (SUN_UP) or setting (SUN_DOWN)
 */
void sunRiseSetIcon(uint16_t x, uint16_t y, sun_direction direction) {
    uint16_t r = 7;

    // Horizontal
    display.drawLine(x - r * 2 + 2, y, x + r * 2 - 2, y, GxEPD_BLACK);
    // Vertical
    display.drawLine(x, y - r * 2 + 2, x, y, GxEPD_BLACK);
    // Angle Top right
    display.drawLine(x - r * 2 + 5, y - r * 2 + 5, x, y, GxEPD_BLACK);
    // Angle Top left
    display.drawLine(x, y, x + r * 2 - 5, y - r * 2 + 5, GxEPD_BLACK);
    // Remove lines inside
    display.fillCircle(x, y, r + 1, GxEPD_WHITE);
    // Empty inside
    display.fillCircle(x, y, r - 1, GxEPD_RED);
    display.drawCircle(x, y, r - 1, GxEPD_BLACK);
    // Overwrite the bottom
    display.fillRect(x - r, y + 4, r * 2, r, GxEPD_WHITE);

    // Arrow up
    if (direction == SUN_UP) {
        display.fillTriangle(x - r / 2 - 1, y + r - 2, x, y + r - 7, x + r / 2 + 1, y + r - 2, GxEPD_WHITE);
        display.drawLine(x - r / 2, y + r - 2, x, y + r - 6, GxEPD_BLACK);
        display.drawLine(x, y + r - 6, x + r / 2, y + r - 2, GxEPD_BLACK);
    } else {
        // Arrow DOWN
        display.drawLine(x - r / 2, y + r - 2, x, y + r + 2, GxEPD_BLACK);
        display.drawLine(x, y + r + 2, x + r / 2, y + r - 2, GxEPD_BLACK);
    }

    // Horizon line
    display.drawLine(x - r, y + r - 2, x - r / 2, y + r - 2, GxEPD_BLACK);
    display.drawLine(x + r / 2, y + r - 2, x + r, y + r - 2, GxEPD_BLACK);
}

/**
 * @brief Draw a moon on the display to indicate this is a night time reading
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Large or small icon
 */
void addMoon(int x, int y, int scale) {
    if (scale == LARGE) {
        display.fillCircle(x - 37, y - 30, scale, GxEPD_BLACK);
        display.fillCircle(x - 24, y - 30, scale * 1.6, GxEPD_WHITE);
    } else {
        display.fillCircle(x - 20, y - 15, scale, GxEPD_BLACK);
        display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_WHITE);
    }
}

/**
 * @brief Display the sun
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius of the circle of the sun
 * @param icon_size Large or small icon
 * @param icon_color Icon colour, red during the day time
 */
void addSun(int x, int y, int scale, boolean icon_size, uint16_t icon_color) {
    int linesize = 3;
    int dxo, dyo, dxi, dyi;

    if (icon_size == small_icon) {
        linesize = 1;
    }

    display.fillCircle(x, y, scale, icon_color);
    if (icon_color != GxEPD_RED) { // not day time or 2 colour display
        display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
    }

    for (float i = 0; i < 360; i = i + 45) {
        dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180);
        dxi = dxo * 0.6;
        dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180);
        dyi = dyo * 0.6;
        if (i == 0 || i == 180) {
            display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
            if (icon_size == large_icon) {
                display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
                display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
            }
        }
        if (i == 90 || i == 270) {
            display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1, GxEPD_BLACK);
            if (icon_size == large_icon) {
                display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0, GxEPD_BLACK);
                display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1, GxEPD_BLACK);
            }
        }
        if (i == 45 || i == 135 || i == 225 || i == 315) {
            display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
            if (icon_size == large_icon) {
                display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
                display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
            }
        }
    }
}

/**
 * @brief Add some clouds to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius/size of the cloud
 * @param linesize Used in the calculation of the size of the cloud
 */
void addCloud(int x, int y, int scale, int linesize) {
    // Draw cloud outer
    display.fillCircle(x - scale * 3, y, scale, GxEPD_BLACK);                              // Left most circle
    display.fillCircle(x + scale * 3, y, scale, GxEPD_BLACK);                              // Right most circle
    display.fillCircle(x - scale, y - scale, scale * 1.4, GxEPD_BLACK);                    // left middle upper circle
    display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, GxEPD_BLACK);       // Right middle upper circle
    display.fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, GxEPD_BLACK); // Upper and lower lines
    // Clear cloud inner
    display.fillCircle(x - scale * 3, y, scale - linesize, GxEPD_WHITE);                                                   // Clear left most circle
    display.fillCircle(x + scale * 3, y, scale - linesize, GxEPD_WHITE);                                                   // Clear right most circle
    display.fillCircle(x - scale, y - scale, scale * 1.4 - linesize, GxEPD_WHITE);                                         // left middle upper circle
    display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, GxEPD_WHITE);                            // Right middle upper circle
    display.fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, GxEPD_WHITE); // Upper and lower lines
}

/**
 * @brief Add some rain to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius/size of the rain drops
 * @param colour Colour of the rain drops
 */
void addRain(int x, int y, int scale, uint16_t colour) {
    for (byte i = 0; i < 6; i++) {
        display.fillCircle(x - scale * 4 + scale * i * 1.3, y + scale * 1.9 + (scale == SMALL ? 3 : 0), scale / 3, colour);
        arrow(x - scale * 4 + scale * i * 1.3 + (scale == SMALL ? 6 : 4), y + scale * 1.6 + (scale == SMALL ? -3 : -1), scale / 6, 40, scale / 1.6, scale * 1.2, colour);
    }
}

/**
 * @brief Add some snow flakes to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius/size of the snow flakes
 * @param colour Colour of the snow flakes
 */
void addSnow(int x, int y, int scale, uint16_t colour) {
    int dxo, dyo, dxi, dyi;

    for (byte flakes = 0; flakes < 5; flakes++) {
        for (int i = 0; i < 360; i = i + 45) {
            dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180);
            dxi = dxo * 0.1;
            dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180);
            dyi = dyo * 0.1;
            display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3, dyo + y + scale * 2, dxi + x + 0 + flakes * 1.5 * scale - scale * 3, dyi + y + scale * 2, colour);
        }
    }
}

/**
 * @brief Add the lightening for the thunder storm
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius/size of the lightening arrows
 * @param colour Colour of the lightening
 */
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

/**
 * @brief Add some lines to give the impression of mist/haze/fog
 *
 * @param x Across
 * @param y Down
 * @param scale Large or small scale
 * @param linesize Width of line
 * @param colour Colour of line
 */
void addFog(int x, int y, int scale, int linesize, uint16_t colour) {
    int offset = 10;
    
    if (scale == SMALL) {
        offset = 5;
    }

    for (byte i = 0; i < 6; i++) {
        display.fillRect(((x + 5) - scale * 3) + offset, y - (scale * 3), scale * 3, linesize, colour);

        display.fillRect(((x - scale) - scale * 2) + offset, y - (scale * 2), scale * 5, linesize, colour);

        display.fillRect(((x - scale) - scale * 3) + offset, y - scale, scale * 4, linesize, colour);

        display.fillRect((x - scale * 3) + offset, y, scale * 4, linesize, colour);

        display.fillRect(((x + 5) - scale * 3) + offset, y + scale, scale * 3, linesize, colour); // bottom line
    }
}

/**
 * @brief Display the fact that we don't have any data to display!
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Used to set the font size to use 
 */
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

/**
 * @brief Add a star to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param starsize Size of the star to add, small or medium 
 */
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
 * @brief Draw graph with 2 sets of data
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
    int minYscale = 10000;

    for (int i = 1; i < len; i++) {
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

    float steps = (ymax - ymin) / (ticklines);

    // Title
    display.setFont();
    drawString(x + w / 2, y - 24, title, CENTER);

    // Draw y-axis tick markers and dashed lines
    for (byte i = 0; i < ticklines + 1; i++) {
        drawString(x - 2, y + ((h / ticklines) * i - 12), String(lrint(ymin + steps * (ticklines - i))), RIGHT);
        if (i == 0) {
            continue;
        }
        display.drawLine(x + 1, y + ((h / ticklines) * i), x + w - 1, y + ((h / ticklines) * i), GxEPD_RED);
        bool blank = true;
        for (byte r = 0; r < w; r = r + 3) {
            if (blank) {
                display.drawLine(x + r, y + ((h / ticklines) * i), x + r + 3, y + ((h / ticklines) * i), GxEPD_WHITE);
                blank = false;
            } else {
                blank = true;
            }
        }
    }

    // x-Axis
    display.drawLine(x, y + h, x + w, y + h, GxEPD_BLACK);

    // y-Axis
    display.drawLine(x, y, x, y + h, GxEPD_BLACK);

    // Draw data line 1
    float x1 = x + 1;
    float y1 = y + (ymax - constrain(Data[0], ymin, ymax)) / (ymax - ymin) * h;
    float y3 = y + (ymax - constrain(Data2[0], ymin, ymax)) / (ymax - ymin) * h;

    if (y1 > y + h - 1)
        y1 = y + h - 1;
    if (y3 > y + h - 1)
        y3 = y + h - 1;
    float x2, y2, y4;

    for (int i = 1; i < len; i++) {
        x2 = x + i * w / (len - 1) - 1;
        y2 = y + (ymax - constrain(Data[i], ymin, ymax)) / (ymax - ymin) * h + 1;
        y4 = y + (ymax - constrain(Data2[i], ymin, ymax)) / (ymax - ymin) * h + 1;
        
        if (y2 > y + h - 1)
            y2 = y + h - 1;
        if (y4 > y + h - 1)
            y4 = y + h - 1;

        display.drawLine(x1, y3, x2, y4, GxEPD_RED);

        for (byte r = 0; r < ceil(w / len) + 1; r++) {
            float m = (y4 - y3) / (x2 - x1);
            float b = y3 - m * x1;
            display.drawLine(x1 + r, y + h - 1, x1 + r, m * (x1 + r) + b, GxEPD_RED);
        }

        display.drawLine(x1, y1 - 1, x2, y2 - 1, GxEPD_BLACK); // thicker line on display
        display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

        x1 = x2;
        y1 = y2;
        y3 = y4;
    }

    // x-Axis ticks
    for (int i = 0; i <= 4; i++) {
        if (i == 0) {
            display.setCursor(x - 5 + (w / 4) * i, y + h + 6);
        } else {
            display.setCursor(x - 7 + (w / 4) * i, y + h + 6);
        }
        display.print(String(12 * i));
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
    int minYscale = 10000;

    for (int i = 1; i < len; i++) {
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

    float steps = (ymax - ymin) / (ticklines);

    // Title
    display.setFont();
    drawString(x + w / 2, y - 24, title, CENTER);

    // Draw y-axis tick markers and dashed lines
    for (byte i = 0; i < ticklines + 1; i++) {
        // Due to space constraints only show values under 10 with a decimal point
        if (ymin < 1 && ymax < 10) {
            drawString(x - 2, y + ((h / ticklines) * i - 12), String(ymin + steps * (ticklines - i), 1), RIGHT);
        } else {
            drawString(x - 2, y + ((h / ticklines) * i - 12), String(lrint(ymin + steps * (ticklines - i))), RIGHT);
        }

        if (i == 0) {
            continue;
        }

        display.drawLine(x + 1, y + ((h / ticklines) * i), x + w - 1, y + ((h / ticklines) * i), GxEPD_RED);
        bool blank = true;
        for (byte r = 0; r < w; r = r + 3) {
            if (blank) {
                display.drawLine(x + r, y + ((h / ticklines) * i), x + r + 3, y + ((h / ticklines) * i), GxEPD_WHITE);
                blank = false;
            } else {
                blank = true;
            }
        }
    }

    // x-Axis
    display.drawLine(x, y + h, x + w, y + h, GxEPD_BLACK);

    // y-Axis
    display.drawLine(x, y, x, y + h, GxEPD_BLACK);

    // Draw data line 1
    float x1 = x + 1;
    float y1 = y + (ymax - constrain(Data[0], ymin, ymax)) / (ymax - ymin) * h;
    float x2, y2;

    if (y1 > y + h - 1) {
        y1 = y + h - 1;
    }

    // Draw data on display
    for (int i = 1; i < len; i++) {
        x2 = x + i * w / (len - 1) - 1;
        y2 = y + (ymax - constrain(Data[i], ymin, ymax)) / (ymax - ymin) * h + 1;
        if (y2 > y + h - 1) {
            y2 = y + h - 1;
        }
        // More solid line using 2 lines 1 pixel apart.
        display.drawLine(x1, y1 - 1, x2, y2 - 1, GxEPD_BLACK);
        display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

        x1 = x2;
        y1 = y2;
    }

    // x-Axis ticks
    // for (int i = 0; i <= 6; i++) {             // 72 hours
    //   display.setCursor(x-7+(w/6)*i ,y+h+6);
    for (int i = 0; i <= 4; i++) {
        if (i == 0) {
            display.setCursor(x - 5 + (w / 4) * i, y + h + 6);
        } else {
            display.setCursor(x - 7 + (w / 4) * i, y + h + 6);
        }
        display.print(String(12 * i));
    }
    // if (i == 0) {   // day 0 (now)
    //   display.print("0");
    // } else if ((12 * i) == 24) {   // day 1
    //   display.print("24");
    // } else if ((12 * i) == 48) {  // day 2
    //   display.print("48");
    // } else if ((12 * i) == 72) {  // day 3
    //   display.print("72");
    // }

    // Reset font
    display.setFont(&DejaVu_Sans_Bold_11);
}

/**
 * @brief Draw a string to the screen - main function for all text written to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param text Test to display
 * @param align Text alignment on the screen
 */
void drawString(int x, int y, String text, alignment align) {
    int16_t x1, y1; // the bounds of x,y and w and h of the variable 'text' in pixels.
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