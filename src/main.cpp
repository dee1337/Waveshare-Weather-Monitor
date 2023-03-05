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
 * Version using Lilygo ESP32S3 T7-S3
 * Waveshare        ESP32S3
 *   DIN               11 (SPI MOSI)
 *   CLK               12 (SPI SCK)
 *   CS                10 (SPI chip selection)
 *   DC                18
 *   RST               16
 *   BUSY              15
 * 
 * This is a copy/update of the weather display written by G6EJD:
 *  https://github.com/G6EJD/ESP32-Revised-Weather-Display-42-E-Paper
 */
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "GxEPD2_GFX.h"
#include "GxEPD2_3C.h"                          // 3 colour screen
#include "GxEPD2_display_selection_new_style.h" // For selecting screen

#include <Fonts/FreeMonoBold12pt7b.h>

#include "config.h"
#include "battery.h"
#include "fonts.h"
#include "arrow.h"
#include "sunrise.h"
#include "sunset.h"

// CaptureLog setup
#define CLOG_ENABLE false                        // this must be defined before cLog.h is included 
#include "cLog.h"

#if CLOG_ENABLE
const uint16_t maxEntries = 20;
const uint16_t maxEntryChars = 50;
CLOG_NEW myLog1(maxEntries, maxEntryChars, NO_TRIGGER, NO_WRAP);
#endif

/* Constants/defines */
const uint SCREEN_WIDTH = 400;
const uint SCREEN_HEIGHT = 300;
const String VERSION = "vSRT";  //v2.2
const String Hemisphere = "north";
const int forecast_counter = 7; // number of forecasts to get/show.

const long sleep_duration = 30; // Number of minutes to go to sleep for
const int sleep_hour = 22;      // Start power saving at 23:00
const int wakeup_hour = 7;      // Stop power saving at 07:00

boolean large_icon = true;
boolean small_icon = false;

#define LARGE 10
#define SMALL 4

// T7-S3 power LED pin which we can turn off to save power
#define LED_PIN 17
//#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

#define DEBUG false

enum alignment
{
    LEFT,
    RIGHT,
    CENTER
};
enum pressure_trend
{
    LEVEL,
    UP,
    DOWN
};
enum sun_direction
{
    SUN_UP,
    SUN_DOWN
};
enum star_size
{
    SMALL_STAR,
    MEDIUM_STAR,
    LARGE_STAR
};

/* Function prototypes */
bool getTodaysWeather(void);
bool getWeatherForecast(void);
static void updateLocalTime(void);
void initialiseDisplay(void);
void logWakeupReason(void);
void displayBattery(int x, int y);
void goToSleep(void);
void displayErrorMessage(String message);
void displaySystemInfo(int x, int y);
void drawString(int x, int y, String text, alignment align);
void displayInformation(void);
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
void displayPressureAndTrend(int x, int y, float pressure, pressure_trend slope, uint16_t colour);
void displayRain(int x, int y);
String convertUnixTime(int unix_time);
double normalizedMoonPhase(int d, int m, int y);
void drawMoon(int x, int y, int dd, int mm, int yy, String hemisphere, const int diameter, bool surface);
void displaySunAndMoon(int x, int y);
void moonPhase(int x, int y, int day, int month, int year, String hemisphere);
int julianDate(int d, int m, int y);
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
void fogIcon(int x, int y, bool large_size, String icon_name);
void hazeIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color);
void sunRiseSetIcon(uint16_t x, uint16_t y, sun_direction direction);
void addCloud(int x, int y, int scale, int linesize);
void addRain(int x, int y, int scale, uint16_t colour);
void addSnow(int x, int y, int scale, uint16_t colour);
void addThunderStorm(int x, int y, int scale, uint16_t colour);
void addFog(int x, int y, int scale, int linesize, uint16_t colour);
void addStar(int x, int y, star_size starsize);

/* Globals etc. */
WiFiClientSecure wifiClient;

String ipAddress = "0:0:0:0";
int rssi = 0;
float battery_voltage = 0.0;

float pressure_readings[forecast_counter] = {0};
float temperature_readings[forecast_counter] = {0};
float rain_readings[forecast_counter] = {0};

// current
typedef struct WeatherStruct
{
    pressure_trend trend = LEVEL;
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

char timeStringBuff[35];      // buffer for time on the display
char dateStringBuff[35];

void setup()
{
    // Ensure power LED is off to save power.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    #if CLOG_ENABLE
    Serial.begin(115200);
    delay(5000);    // T7-S3 serial slow to initiate!

    Serial.println("\n##################################");
    Serial.println(F("ESP32 Information:"));
    Serial.printf("Internal Total Heap %d, Internal Used Heap %d, Internal Free Heap %d\n", ESP.getHeapSize(), ESP.getHeapSize() - ESP.getFreeHeap(), ESP.getFreeHeap());
    Serial.printf("Sketch Size %d, Free Sketch Space %d\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
    Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("Chip Model %s, ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
    Serial.println("##################################\n");

    // Why did we wake up
    logWakeupReason();
    #endif

    initialiseDisplay();

    WiFi.disconnect();

    WiFi.mode(WIFI_STA); // switch off AP
    WiFi.setAutoReconnect(true);

    WiFi.begin(SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        #if CLOG_ENABLE
        Serial.print(".");
        #endif
    }

    #if CLOG_ENABLE
    Serial.println("");
    Serial.println("Connecting to Wi-Fi...");
    #endif

    ipAddress = WiFi.localIP().toString();
    rssi = WiFi.RSSI();

    CLOG(myLog1.add(), "IP Address: %s", ipAddress);
    #if CLOG_ENABLE
    Serial.println("Connecting to NTP Time Server...");
    #endif

    configTime(0, 3600, SNTP_TIME_SERVER);
    updateLocalTime();

    #if CLOG_ENABLE
    Serial.println("All set up, time to get weather information and display it...");
    #endif

    bool today_flag = getTodaysWeather();
    bool forecast_flag = getWeatherForecast();

    // Turn off wifi to save power
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    battery_voltage = (float) getBatteryVoltage(); 

    if (today_flag == true || forecast_flag == true)
    {
        CLOG(myLog1.add(), "All data retrieved successfully.");
        displayInformation();
    }
    else
    {
        CLOG(myLog1.add(), "Unable to retrieve data!");
        displayErrorMessage("Unable to retrieve data, contact support ;-)");
    }

    goToSleep();
}

/**
 * @brief Main loop which we will never enter as we enter deep sleep once we've updated
 * the display.
 * 
 */
void loop()
{
    while (true) {
        /*
         * Should never get here - using deep sleep after displaying weather.
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
    Serial.println("Program log... ");
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
void logWakeupReason(void)
{
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
 * @brief Update global buffers with various times/date values.
 * 
 */
static void updateLocalTime(void)
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        CLOG(myLog1.add(), "Failed to obtain time");
        return;
    }

    // Update buffer with current time/date
    strftime(dateStringBuff, sizeof(dateStringBuff), "%a  %d-%b-%y", &timeinfo); // Sat 15-Jan-23
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);     // 15:15:23
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

    if (!client.connect(host, port))
    {
        CLOG(myLog1.add(), "HTTPS[1] connection to OpenWeatherMap failed!");
        return false;
    }

    uint32_t timeout = millis();
    char c = 0;

    // Send GET request
    #if DEBUG
    Serial.printf("Sending GET request to %s, port %d\n", host, port);
    #endif
    client.print(String("GET ") + WEATHER_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

    while (client.connected())
    {
        String line = client.readStringUntil('\n');
        if (line == "\r")
        {
            #if DEBUG
            Serial.println("Header end found.");
            #endif
            break;
        }

        #if DEBUG
        Serial.println(line);
        #endif

        if ((millis() - timeout) > 5000UL)
        {
            #if DEBUG
            Serial.println("HTTP header timeout");
            #endif

            client.stop();
            return false;
        }
    }

    #if DEBUG
    //  Serial.print("JSON length: "); Serial.println(client.available());
    Serial.println("Parsing JSON...");
    #endif

    // bool decode = false;
    DynamicJsonDocument doc(20 * 1024);

    #if DEBUG
    Serial.println("Deserialization process starting...");
    #endif

    // Parse JSON object
    DeserializationError err = deserializeJson(doc, client);
    if (err)
    {
        CLOG(myLog1.add(), "deserializeJson(1) failed: %s", err.c_str());
        retcode = false;
    }
    else
    {
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

    if (!client.connect(host, port))
    {
        CLOG(myLog1.add(), "HTTPS[2] connection to OpenWeatherMap failed!");
        return false;
    }

    uint32_t timeout = millis();
    char c = 0;

    // Send GET request
    #if DEBUG
    Serial.printf("Sending GET request to %s, port %d\n", host, port);
    #endif
    client.print(String("GET ") + FORECAST_URL + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

    while (client.connected())
    {
        String line = client.readStringUntil('\n');
        if (line == "\r")
        {
            #if DEBUG
            Serial.println("Header end found.");
            #endif

            break;
        }

        if ((millis() - timeout) > 5000UL)
        {
            #if DEBUG
            Serial.println("HTTP header timeout");
            #endif

            client.stop();
            return false;
        }
    }

    #if DEBUG
    Serial.println("Parsing Forecast JSON...");
    #endif

    DynamicJsonDocument doc(20 * 1024);

    #if DEBUG
    Serial.println("Deserialization process starting...");
    #endif

    // Parse JSON object
    DeserializationError err = deserializeJson(doc, client);
    if (err)
    {
        CLOG(myLog1.add(), "deserializeJson(2) failed: %s", err.c_str());
        retcode = false;
    }
    else
    {
        #if DEBUG
        Serial.println(F("Deserialization succeeded, decoding forecast data..."));
        #endif

        for (byte i = 0; i < forecast_counter; i++)
        {
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
            forecast[i].rain = doc["list"][i]["rain"]["3h"];
            forecast[i].snow = doc["list"][i]["snow"]["3h"];
            forecast[i].period = doc["list"][i]["dt_txt"].as<String>();
        }

        float ptrend = forecast[1].pressure - forecast[0].pressure; // Measure pressure slope between ~now and later
        ptrend = ((int)(ptrend * 10)) / 10.0;                       // Remove any small variations less than 0.1
        weather.trend = LEVEL;

        if (ptrend > 0)
            weather.trend = UP;
        else if (ptrend < 0)
            weather.trend = DOWN;

        CLOG(myLog1.add(), "Deserialized [%d] forecasts in %ld ms", forecast_counter, millis() - dt);
    }

    client.stop();

    return retcode;
}

/**
 * @brief Set up the display, serial connection, rotation, font, colour, and size.
 * 
 */
void initialiseDisplay()
{
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
void displayErrorMessage(String message)
{
    display.setFullWindow();
    display.firstPage();

    do
    {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(10, 60);
        drawString(200, 150, "Error: " + message, CENTER);
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
    do
    {
        displayHeader();
        displayTemperature(0, 15);
        displayWeatherIcon(140, 15, weather.icon, large_icon);          // Weather icon
        displayWind(319, 64, weather.wind_deg, weather.wind_speed, 49); // Wind direction info
        displayWeatherForecast(0, 172);                                 // Forecast
        displaySunAndMoon(0, 238);                                      // Sunset and sunrise and moon state with icons
        displayWeatherDescription(0, 148);                              // Description of the weather now
        displaySystemInfo(263, 238);
    } while (display.nextPage());

    display.hibernate();

    CLOG(myLog1.add(), "Display updated in %ld seconds", (millis() - dt) / 1000);
}

/**
 * @brief Display the header information;
 * location of weather data, date, time of reading the weather, version.
 * 
 */
void displayHeader(void)
{
    drawString((SCREEN_WIDTH / 2) + 8, 1, LOCATION, CENTER);
    drawString(SCREEN_WIDTH - 5, 3, dateStringBuff, RIGHT);
    drawString(1, 1, "@", LEFT);
    drawString(13, 4, timeStringBuff, LEFT);
    drawString(115, 4, VERSION, CENTER);
    display.drawLine(0, 15, SCREEN_WIDTH-2, 15, GxEPD_BLACK);
}

/**
 * @brief Display the current temperature, cloud cover and humidity.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayTemperature(int x, int y)
{
    display.drawRect(x, y, 144, 128, GxEPD_BLACK);
    display.setFont(&DSEG7_Classic_Bold_21);
    display.setTextSize(2);

    if (weather.temperature < 0)
    {
        drawString(x, y + 61, "-", LEFT);                                       // Show temperature sign to compensate for non-proportional font spacing
        drawString(x + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + 95, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }
    else if (weather.temperature < 10)
    {
        drawString(x + 25, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + 95, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }
    else
    {
        drawString(x + 8, y + 25, String(fabs(weather.temperature), 1), LEFT); // Show current Temperature without a '-' minus sign
        display.setTextSize(1);
        drawString(x + 110, y + 25, "'C", LEFT); // Add-in ° symbol ' in this font plus units
    }

    drawString(x + 32, y + 85, String(weather.low, 0) + "'/" + String(weather.high, 0) + "'", LEFT); // Show forecast high and Low, in the font ' is a °
    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(x + 72, y + 114, String(weather.humidity) + "% RH", CENTER);
    if (weather.clouds > 0)
    {
        displayCloudCover(x + 60, y + 10, weather.clouds);
    }
}

/**
 * @brief Display system information which includes battery voltage, wi-fi signal
 * strength and the IP address we've been assigned. 
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displaySystemInfo(int x, int y)
{
    int wifi_rssi = 0;
    int xpos = 1;

    display.drawRect(x, y, 135, 62, GxEPD_BLACK);

    // display battery voltage
    displayBattery(x + 43, y + 5);

    int rssi_x = x + 33;
    int rssi_y = y + 36;
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

    drawString(x + 68, y + 45, ipAddress, CENTER);
}

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
    int colour = GxEPD_BLACK;

    if (battery_voltage >= LOW_BATTERY_VOLTAGE) { 
        //p = 2836.9625 * pow(battery_voltage, 4) - 43987.4889 * pow(battery_voltage, 3) + 255233.8134 * pow(battery_voltage, 2) - 656689.7123 * battery_voltage + 632041.7303;
        percentage = calculateBatteryPercentage(battery_voltage);
        CLOG(myLog1.add(), "Battery voltage: %.2f, percentage: %d", battery_voltage, percentage);

        if (percentage <= 25) {
            colour = GxEPD_RED;
        }

        display.drawRect(x + 9, y + 3, 34, 10, GxEPD_BLACK);
        display.fillRect(x + 43, y + 5, 2, 6, GxEPD_BLACK);
        display.fillRect(x + 11, y + 5, 31 * percentage / 100.0, 6, colour);

        // draw lines to give a better battery icon
        // 25% = 7
        // 50% = 15
        // 75% = 23
        // 100% = 
        display.fillRect((x + 11) + 7, y + 4, 1, 8, GxEPD_WHITE);  // 25% across
        display.fillRect((x + 11) + 15, y + 4, 1, 8, GxEPD_WHITE);  // 50% across
        display.fillRect((x + 11) + 23, y + 4, 1, 8, GxEPD_WHITE);  // 75% across
        display.fillRect((x + 11) + 30, y + 4, 1, 8, GxEPD_WHITE);  // 100% across

        display.setTextColor(colour);
        drawString(x + 48, y + 4, String(percentage) + "%", LEFT);
        display.setTextColor(GxEPD_BLACK);
        drawString(x - 36, y + 4,  String(battery_voltage, 2) + "v", LEFT);
    } 
    else
    {
        CLOG(myLog1.add(), "Battery voltage: %.2f, recharge now!", battery_voltage);
        display.setTextColor(GxEPD_RED);
        drawString(x - 39, y + 4, "Recharge Battery", LEFT);
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
void displayCloudCover(int x, int y, int cover)
{
    addCloud(x, y, SMALL * 0.5, 1);          // Main cloud
    addCloud(x + 5, y - 5, SMALL * 0.35, 1); // Cloud top right
    addCloud(x - 8, y - 5, SMALL * 0.35, 1); // Cloud top left
    drawString(x + 30, y - 5, String(cover) + "%", CENTER);
}

/**
 * @brief Current weather description as returned by openweathermap.org
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayWeatherDescription(int x, int y)
{
    display.drawRect(x, y - 4, SCREEN_WIDTH - 1, 27, GxEPD_BLACK);
    String description = weather.description; // + ", " + weather.main;

    display.setFont(&FreeMonoBold12pt7b);
    unsigned int MsgWidth = 28;
    if (description.length() > MsgWidth)
    {
        display.setFont(&DejaVu_Sans_Bold_11); // Drop to smaller font to allow all weather description to be displayed
        MsgWidth = 52;
        y = y - 7;
    }
    drawStringMaxWidth(x + 3, y + 15, MsgWidth, titleCase(description), CENTER); // 28 character screen width at this font size
    display.setFont(&DejaVu_Sans_Bold_11);
}

/**
 * @brief Title case the first letter of the text supplied, e.g. 'current' and 'Current' will be returned.
 * 
 * @param text Text to capitalise the first letter.
 * @return String Capitalised text.
 */
String titleCase(String text)
{
    if (text.length() > 0)
    {
        String temp_text = text.substring(0, 1);
        temp_text.toUpperCase();
        return temp_text + text.substring(1); // Title-case the string
    }
    return "";
}

/**
 * @brief Draw a string ensuring it doesn't exceed the max width of the display.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param text_width Number of characters to display if possible
 * @param text Text to display
 * @param align Text alignment, LEFT, CENTER or RIGHT aligned
 */
void drawStringMaxWidth(int x, int y, unsigned int text_width, String text, alignment align)
{
    int16_t x1, y1; // the bounds of x,y and w and h of the variable 'text' in pixels.
    uint16_t w, h;
    int font_width = 10; // 10 pixels per character - guess at the moment!

    if (text.length() > text_width * 2)
    {
        text = text.substring(0, text_width * 2); // Truncate if too long for 2 rows of text
    }

    display.getTextBounds(text, x, y, &x1, &y1, &w, &h);

    if (align == RIGHT)
    {
        x = x - w;
    }

    if (align == CENTER)
    {
        // x = x - (w / 2);
        x = (394 - w) / 2;
    }

    display.setCursor(x, y);
    display.println(text.substring(0, text_width));
    if (text.length() > text_width)
    {
        display.setCursor(x, y + h);
        display.println(text.substring(text_width));
    }
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
void displayWind(int x, int y, float angle, float windspeed, int radius)
{
    int offset = 16;
    int dxo;
    int dyo;
    int dxi;
    int dyi;

    arrow(x + offset, y + offset, radius - 11, angle, 15, 22, GxEPD_RED); // Show wind direction on outer circle of width and length
    display.setTextSize(0);
    display.drawRect(x - radius, y - radius, 129, 128, GxEPD_BLACK);

    display.drawCircle(x + offset, y + offset, radius, GxEPD_BLACK);       // Draw compass circle
    display.drawCircle(x + offset, y + offset, radius + 1, GxEPD_BLACK);   // Draw compass circle
    display.drawCircle(x + offset, y + offset, radius * 0.7, GxEPD_BLACK); // Draw compass inner circle
    for (float a = 0; a < 360; a = a + 22.5)
    {
        dxo = radius * cos((a - 90) * PI / 180);
        dyo = radius * sin((a - 90) * PI / 180);
        if (a == 45)
            drawString(dxo + x + 10 + offset, dyo + y - 10 + offset, "NE", CENTER);
        if (a == 135)
            drawString(dxo + x + 12 + offset, dyo + y  + offset, "SE", CENTER);
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

    drawString(x + offset, y + 3 + offset + radius, "S", CENTER);
    drawString(x - radius - 9 + offset, y - 4 + offset, "W", CENTER);
    drawString(x + radius + offset + 7, y - 4 + offset, "E", CENTER);

    drawString(x + offset, y - 23 + offset, windDegToDirection(angle), CENTER);

    drawString(x + offset, y - 6 + offset, String(windspeed, 1), CENTER);

    display.setFont(); // use default 6x8 font
    drawString(x + offset, y - 5 + offset, "mph", CENTER);

    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(x + offset, y + 16 + offset, String(angle, 0) + "'", CENTER);
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
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength, uint16_t colour)
{
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
String windDegToDirection(float winddirection)
{

    if (winddirection >= 348.75 || winddirection < 11.25)
    {
        return "N";
    }

    if (winddirection >= 11.25 && winddirection < 33.75)
    {
        return "NNE";
    }

    if (winddirection >= 33.75 && winddirection < 56.25)
    {
        return "NE";
    }

    if (winddirection >= 56.25 && winddirection < 78.75)
    {
        return "ENE";
    }

    if (winddirection >= 78.75 && winddirection < 101.25)
    {
        return "E";
    }

    if (winddirection >= 101.25 && winddirection < 123.75)
    {
        return "ESE";
    }

    if (winddirection >= 123.75 && winddirection < 146.25)
    {
        return "SE";
    }

    if (winddirection >= 146.25 && winddirection < 168.75)
    {
        return "SSE";
    }

    if (winddirection >= 168.75 && winddirection < 191.25)
    {
        return "S";
    }

    if (winddirection >= 191.25 && winddirection < 213.75)
    {
        return "SSW";
    }

    if (winddirection >= 213.75 && winddirection < 236.25)
    {
        return "SW";
    }

    if (winddirection >= 236.25 && winddirection < 258.75)
    {
        return "WSW";
    }

    if (winddirection >= 258.75 && winddirection < 281.25)
    {
        return "W";
    }

    if (winddirection >= 281.25 && winddirection < 303.75)
    {
        return "WNW";
    }

    if (winddirection >= 303.75 && winddirection < 326.25)
    {
        return "NW";
    }

    if (winddirection >= 326.25 && winddirection < 348.75)
    {
        return "NNW";
    }

    return "?";
}

/**
 * @brief Display the time when the sun rises and sets.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displaySunAndMoon(int x, int y)
{
    time_t now = time(NULL);
    struct tm *now_utc = gmtime(&now);
    const int day_utc = now_utc->tm_mday;
    const int month_utc = now_utc->tm_mon + 1;
    const int year_utc = now_utc->tm_year + 1900;

    display.drawRect(x, y, 262, 62, GxEPD_BLACK);

    sunRiseSetIcon(x + 20, y + 20, SUN_UP);
    sunRiseSetIcon(x + 20, y + 45, SUN_DOWN);

    drawString(x + 40, y + 15, convertUnixTime(weather.sunrise).substring(0, 8), LEFT); // 08:00 AM
    drawString(x + 40, y + 40, convertUnixTime(weather.sunset).substring(0, 8), LEFT);  // 19:00 PM

    drawMoon(x + 117, y - 7, day_utc, month_utc, year_utc, Hemisphere, 38, true);
    moonPhase(x + 183, y + 20, day_utc, month_utc, year_utc, Hemisphere);
}

/**
 * @brief Display the weather forcast for the next 'forecast_counter' hours.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayWeatherForecast(int x, int y)
{
    int offset = 57;

    for (byte i = 0; i < forecast_counter; i++)
    {
        displaySingleForecast(x + offset * i, y, offset, i);
    }

    for (byte i = 0; i < forecast_counter; i++)
    {
        pressure_readings[i] = forecast[i].pressure;
        temperature_readings[i] = forecast[i].temperature;
        rain_readings[i] = forecast[i].rain;
    }
}

/**
 * @brief Display a single 3 hourly forcast
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param offset Offset to apply as this is called for large and small boxes on the display
 * @param index The index of the forecast to display
 */
void displaySingleForecast(int x, int y, int offset, int index)
{
    display.drawRect(x, y, offset - 1, 65, GxEPD_BLACK);              // big rectangle
    display.drawLine(x, y + 13, x + offset - 2, y + 13, GxEPD_BLACK); // line under time

    // If night time invert weather icon to show it's night time, feedback from focus group :-)
    if (forecast[index].icon.endsWith("n"))
    {
        display.fillRect(x, y + 14, offset - 1, 50, GxEPD_BLACK);
    }

    displayWeatherIcon(x + offset / 2 + 1, y + 35, forecast[index].icon, small_icon);

    drawString(x + offset / 2, y + 3, String(forecast[index].period.substring(11, 16)), CENTER);
    // Set text colour to white as the square is now black to show it's night time
    if (forecast[index].icon.endsWith("n"))
    {
        display.setTextColor(GxEPD_WHITE);
        drawString(x + offset / 2, y + 50, String(forecast[index].high, 0) + "/" + String(forecast[index].low, 0), CENTER);
        // Reset set text colour to black
        display.setTextColor(GxEPD_BLACK);
    }
    else
    {
        drawString(x + offset / 2, y + 50, String(forecast[index].high, 0) + "/" + String(forecast[index].low, 0), CENTER);
    }
}

/**
 * @brief Convert unix time to the current time
 * 
 * @param unix_time Unix time value to convert
 * @return String Current time hh:mm
 */
String convertUnixTime(int unix_time)
{
    // Returns '21:12 PM'
    time_t tm = unix_time;
    struct tm *now_tm = localtime(&tm);
    char output[10];

    strftime(output, sizeof(output), "%H:%M %p", now_tm);

    return output;
}

/**
 * @brief Draw the current moon on the display according to the date passed in.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param dd Day 
 * @param mm Month
 * @param yy Year
 * @param hemisphere Our hemisphere, North or South
 * @param diameter Diameter of the moon to display
 * @param surface Display a representation of the surface of the moon
 */
void drawMoon(int x, int y, int dd, int mm, int yy, String hemisphere, const int diameter, bool surface)
{
    //  const int diameter = 38;
    double phase = normalizedMoonPhase(dd, mm, yy);

    if (hemisphere == "south")
    {
        phase = 1 - phase;
    }

    // Draw dark part of moon
    display.fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, GxEPD_BLACK);
    const int number_of_lines = 90;
    for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++)
    {
        double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
        // Determine the edges of the lighted part of the moon
        double Rpos = 2 * Xpos;
        double Xpos1, Xpos2;
        if (phase < 0.5)
        {
            Xpos1 = -Xpos;
            Xpos2 = Rpos - 2 * phase * Rpos - Xpos;
        }
        else
        {
            Xpos1 = Xpos;
            Xpos2 = Xpos - 2 * phase * Rpos + Rpos;
        }
        // Draw light part of moon
        double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW1y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW2y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW3y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW4y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        display.drawLine(pW1x, pW1y, pW2x, pW2y, GxEPD_WHITE);
        display.drawLine(pW3x, pW3y, pW4x, pW4y, GxEPD_WHITE);
    }

    display.drawCircle(x + diameter - 1, y + diameter, diameter / 2, GxEPD_BLACK);

    if (surface)
    {
        // Need offset as the surface code was written by SeBassTian23.
        x = x + 57;
        y = y + 59;
        // Add moon surface on top
        display.drawPixel(x - diameter + 22, y - diameter + 1, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 3, GxEPD_BLACK);
        display.drawPixel(x - diameter + 24, y - diameter + 3, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 4, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 4, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 4, GxEPD_BLACK);
        display.drawPixel(x - diameter + 24, y - diameter + 4, GxEPD_BLACK);
        display.drawPixel(x - diameter + 26, y - diameter + 4, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 5, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 5, GxEPD_BLACK);
        display.drawPixel(x - diameter + 26, y - diameter + 5, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 6, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 6, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 6, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 23, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 25, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 28, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 30, y - diameter + 7, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 21, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 23, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 26, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 28, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 29, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 31, y - diameter + 8, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 22, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 23, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 25, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 26, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 28, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 29, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 30, y - diameter + 9, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 23, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 24, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 26, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 29, y - diameter + 10, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 21, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 24, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 28, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 30, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 31, y - diameter + 11, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 21, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 23, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 24, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 25, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 32, y - diameter + 12, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 22, y - diameter + 13, GxEPD_BLACK);
        display.drawPixel(x - diameter + 1, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 25, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 28, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 31, y - diameter + 14, GxEPD_BLACK);
        display.drawPixel(x - diameter + 1, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 29, y - diameter + 15, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 16, GxEPD_BLACK);
        display.drawPixel(x - diameter + 1, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 4, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 17, GxEPD_BLACK);
        display.drawPixel(x - diameter + 1, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 18, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 19, GxEPD_BLACK);
        display.drawPixel(x - diameter + 1, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 20, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 21, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 22, GxEPD_BLACK);
        display.drawPixel(x - diameter + 2, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 4, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 23, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 4, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 24, GxEPD_BLACK);
        display.drawPixel(x - diameter + 3, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 5, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 21, y - diameter + 25, GxEPD_BLACK);
        display.drawPixel(x - diameter + 4, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 12, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 13, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 26, GxEPD_BLACK);
        display.drawPixel(x - diameter + 6, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 20, y - diameter + 27, GxEPD_BLACK);
        display.drawPixel(x - diameter + 7, y - diameter + 28, GxEPD_BLACK);
        display.drawPixel(x - diameter + 8, y - diameter + 28, GxEPD_BLACK);
        display.drawPixel(x - diameter + 10, y - diameter + 28, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 28, GxEPD_BLACK);
        display.drawPixel(x - diameter + 18, y - diameter + 28, GxEPD_BLACK);
        display.drawPixel(x - diameter + 9, y - diameter + 29, GxEPD_BLACK);
        display.drawPixel(x - diameter + 11, y - diameter + 29, GxEPD_BLACK);
        display.drawPixel(x - diameter + 14, y - diameter + 29, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 29, GxEPD_BLACK);
        display.drawPixel(x - diameter + 16, y - diameter + 29, GxEPD_BLACK);
        display.drawPixel(x - diameter + 15, y - diameter + 30, GxEPD_BLACK);
        display.drawPixel(x - diameter + 19, y - diameter + 30, GxEPD_BLACK);
        display.drawPixel(x - diameter + 17, y - diameter + 31, GxEPD_BLACK);
    }
}

/**
 * @brief Calculate the approximate phase of the moon
 * 
 * @param d Day
 * @param m Month
 * @param y Year
 * @return double Normalised moon phase 
 */
double normalizedMoonPhase(int d, int m, int y)
{
    int j = julianDate(d, m, y);
    // Calculate the approximate phase of the moon
    double Phase = (j + 4.867) / 29.53059;

    return (Phase - (int)Phase);
}

/**
 * @brief Return the julian date of a date passed in.
 * 
 * @param d Day
 * @param m Month
 * @param y Year
 * @return int Julian date (0-366)
 */
int julianDate(int d, int m, int y)
{
    int mm, yy, k1, k2, k3, j;
    yy = y - (int)((12 - m) / 10);
    mm = m + 9;
    if (mm >= 12)
        mm = mm - 12;
    k1 = (int)(365.25 * (yy + 4712));
    k2 = (int)(30.6001 * mm + 0.5);
    k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
    // 'j' for dates in Julian calendar:
    j = k1 + k2 + d + 59 + 1;
    if (j > 2299160)
    {
        j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
    }

    return j;
}

/**
 * @brief Print the current moon phase 
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param day Day
 * @param month Month
 * @param year Year
 * @param hemisphere North or South
 */
void moonPhase(int x, int y, int day, int month, int year, String hemisphere)
{
    int c, e;
    double jd;
    int b;
    int yoffset = y + 13; // offset down for second word of moon type
    int xoffset = x + 4;  // offset across for shorter top word of moon type

    if (month < 3)
    {
        year--;
        month += 12;
    }

    ++month;
    c = 365.25 * year;
    e = 30.6 * month;
    jd = c + e + day - 694039.09; /* jd is total days elapsed */
    jd /= 29.53059;               /* divide by the moon cycle (29.53 days) */
    b = jd;                       /* int(jd) -> b, take integer part of jd */
    jd -= b;                      /* subtract integer part to leave fractional part of original jd */
    b = jd * 8 + 0.5;             /* scale fraction from 0-8 and round by adding 0.5 */
    b = b & 7;                    /* 0 and 8 are the same phase so modulo 8 for 0 */

    if (hemisphere == "south")
    {
        b = 7 - b;
    }

    if (b == 0)
    { // New; 0% illuminated
        drawString(xoffset, y, "New", LEFT);
        drawString(x, yoffset, "Moon", LEFT);
    }
    else if (b == 1)
    { // Waxing crescent; 25% illuminated
        drawString(x, y, " Waxing", LEFT);
        drawString(x, yoffset, "Crescent", LEFT);
    }
    else if (b == 2)
    { // First quarter; 50% illuminated
        drawString(x, y, " First", LEFT);
        drawString(x, yoffset, "Quarter", LEFT);
    }
    else if (b == 3)
    { // Waxing gibbous; 75% illuminated
        drawString(xoffset, y, "Waxing", LEFT);
        drawString(x, yoffset, "Gibbous", LEFT);
    }
    else if (b == 4)
    { // Full; 100% illuminated
        drawString(xoffset, y, "Full", LEFT);
        drawString(x, yoffset, "Moon", LEFT);
    }
    else if (b == 5)
    { // Waning gibbous; 75% illuminated
        drawString(xoffset, y, "Waning", LEFT);
        drawString(x, yoffset, "Gibbous", LEFT);
    }
    else if (b == 6)
    { // Last quarter; 50% illuminated
        drawString(x, y, " Third", LEFT);
        drawString(x, yoffset, "Quarter", LEFT);
    }
    else if (b == 7)
    { // Waning crescent; 25% illuminated
        drawString(x, y, " Waning", LEFT);
        drawString(x, yoffset, "Crescent", LEFT);
    }
}

/**
 * @brief Display the current weather as an icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param icon Icon to display as set by openweathermap.org
 * @param large_icon If this is a large icon or not
 */
void displayWeatherIcon(int x, int y, String icon, bool large_icon)
{
    if (large_icon)
    { // == large icon, TODO: need to change this logic, variable name!
        // Night time
        if (icon.endsWith("n"))
        {
            display.fillRect(x + 5, y, 124, 128, GxEPD_BLACK);
            display.setTextColor(GxEPD_WHITE); // invert for night time
            displayPressureAndTrend(x + 45, y + 100, weather.pressure, weather.trend, GxEPD_WHITE);
            displayRain(x + 67, y + 115);
            display.setTextColor(GxEPD_BLACK); // reset colour
        }
        else
        {
            display.drawRect(x + 5, y, 124, 128, GxEPD_BLACK);
            displayPressureAndTrend(x + 45, y + 100, weather.pressure, weather.trend, GxEPD_BLACK);
            displayRain(x + 67, y + 115);
        }
        x = x + 65;
        y = y + 65;
    }

    if (icon == "01d")
    { // sun
        sunnyIcon(x, y, large_icon, icon, GxEPD_RED);
    }
    else if (icon == "01n")
    {
        sunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
    }
    else if (icon == "02d")
    { // few clouds (clouds and sun)
        mostlySunnyIcon(x, y, large_icon, icon, GxEPD_RED);
    }
    else if (icon == "02n")
    {
        mostlySunnyIcon(x, y, large_icon, icon, GxEPD_BLACK);
    }
    else if (icon == "03d" || icon == "03n")
    { // scattered clouds, no sun
        cloudyIcon(x, y, large_icon, icon);
    }
    else if (icon == "04d" || icon == "04n")
    { // broken clouds, more clouds than scatterred!
        veryCloudyIcon(x, y, large_icon, icon);
    }
    else if (icon == "09d")
    {
        chanceOfRainIcon(x, y, large_icon, icon, GxEPD_RED);
    }
    else if (icon == "09n")
    {
        chanceOfRainIcon(x, y, large_icon, icon, GxEPD_BLACK);
    }
    else if (icon == "10d" || icon == "10n")
    {
        rainIcon(x, y, large_icon, icon);
    }
    else if (icon == "11d" || icon == "11n")
    {
        thunderStormIcon(x, y, large_icon, icon);
    }
    else if (icon == "13d" || icon == "13n")
    {
        snowIcon(x, y, large_icon, icon);
    }
    else if (icon == "50d")
    {
        hazeIcon(x, y, large_icon, icon, GxEPD_RED);
    }
    else if (icon == "50n")
    {
        fogIcon(x, y, large_icon, icon);
    }
    else
    {
        noData(x, y, large_icon);
    }
}

/**
 * @brief Display the current barometric pressure and the trend, i.e. is is going to rise, fall or stay as it is
 * in the next 3 hours.
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param pressure Current barometric ressure value
 * @param slope Is it going to rise/fall/stay the same
 * @param colour Colour of the trend arrow
 */
void displayPressureAndTrend(int x, int y, float pressure, pressure_trend slope, uint16_t colour)
{
    display.setFont(&DSEG7_Classic_Bold_21);
    drawString(x - 35, y - 95, String(pressure, 0), LEFT); // metric
    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(x + 36, y - 90, "hPa", LEFT);
    if (slope == LEVEL)
    {
        display.drawInvertedBitmap(x + 60, y - 96, FL_Arrow, 18, 18, colour); // Steady
    }

    if (slope == DOWN)
    {
        display.drawInvertedBitmap(x + 60, y - 96, DN_Arrow, 18, 18, colour); // Falling
    }

    if (slope == UP)
    {
        display.drawInvertedBitmap(x + 60, y - 96, UP_Arrow, 18, 18, colour); // Rising
    }
}

/**
 * @brief Display how much rainfall there is
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 */
void displayRain(int x, int y)
{
    if (forecast[1].rain > 0)
    {

        drawString(x, y, String(forecast[0].rain, (forecast[0].rain > 0.5 ? 2 : 3)) + "mm Rain", CENTER); // Only display rainfall if > 0
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
void mostlySunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 17;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    { // Night time, add stars
        addMoon(x, y + offset, scale);
    }
    else
    { // Day time, add sun
        addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
    }

    addCloud(x, y + offset, scale, linesize);
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
void sunnyIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color)
{
    int scale = SMALL;
    int offset = 0;

    if (large_size)
    {
        scale = LARGE;
        offset = 10;
    }

    if (icon_name.endsWith("n"))
    { // Night time, show stars
        addMoon(x, y + offset, scale);

        if (!large_size)
        {
            // Small stars
            addStar(x, y, SMALL_STAR);
            addStar(x - 18, y + 3, SMALL_STAR);
            addStar(x + 17, y - 10, SMALL_STAR);

            // Medium stars
            addStar(x - 1, y - 14, MEDIUM_STAR); // top left star
            addStar(x + 10, y, MEDIUM_STAR);     // bottom right star
        }
        else
        {
            // Fill area with random stars
            int horizontal = 0;
            int vertical = 0;
            int left = 150;
            int top = 45;
            int width = 115;
            int height = 90;
            display.setTextColor(GxEPD_WHITE);
            for (int i = 0; i <= 41; i++)
            {
                horizontal = (int)(((rand() / (RAND_MAX * 1.0f)) * width) + left);
                vertical = (int)(((rand() / (RAND_MAX * 1.0f)) * height) + top);
                drawString(horizontal, vertical, ".", LEFT);
            }
            display.setTextColor(GxEPD_BLACK);
        }
    }
    else
    { // Day time, show sun
        scale = scale * 1.5;
        addSun(x, y + offset, scale, large_size, icon_color);
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
void cloudyIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        if (icon_name.endsWith("n"))
            addMoon(x, y + offset, scale);

        linesize = 1;
        addCloud(x, y + offset, scale, linesize);
    }
    else
    {
        if (icon_name.endsWith("n"))
        {
            addMoon(x, y + offset, scale);
        }

        addCloud(x, y + offset, scale, linesize); // Main cloud
    }
}

/**
 * @brief Display the very cloudy icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
void veryCloudyIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        if (icon_name.endsWith("n"))
            addMoon(x, y + offset, scale);

        linesize = 1;
        addCloud(x - 7, y - 7 + offset, 2, linesize);  // Left v.small
        addCloud(x + 8, y - 10 + offset, 2, linesize); // Right v.small
        addCloud(x, y + offset, scale, linesize);      // Main cloud
    }
    else
    {
        if (icon_name.endsWith("n"))
        {
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
void chanceOfRainIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addRain(x, y + offset, scale, GxEPD_WHITE);
    }
    else
    {
        addSun(x - scale * 1.8, y - scale * 1.8 + offset, scale, large_size, icon_color);
        addRain(x, y + offset, scale, GxEPD_BLACK);
    }

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
void rainIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addRain(x, y + offset, scale, GxEPD_WHITE);
    }
    else
    {
        addRain(x, y + offset, scale, GxEPD_BLACK);
    }

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
void thunderStormIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addThunderStorm(x, y + offset, scale, GxEPD_WHITE);
    }
    else
    {
        addThunderStorm(x, y + offset, scale, GxEPD_BLACK);
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
void snowIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addSnow(x, y + offset, scale, GxEPD_WHITE);
    }
    else
    {
        addSnow(x, y + offset, scale, GxEPD_BLACK);
    }

    addCloud(x, y + offset, scale, linesize);
}

/**
 * @brief Display the fog icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 */
void fogIcon(int x, int y, bool large_size, String icon_name)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 12;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addFog(x, y + offset, scale, linesize, GxEPD_WHITE);
    }
    else
    {
        addFog(x, y + offset, scale, linesize, GxEPD_BLACK);
        addCloud(x, y + offset, scale, linesize);
    }
}

/**
 * @brief Display the haze icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Large or small icon
 * @param icon_name Icon name, looking to see if this is a day or night icon
 * @param icon_color Icon colour
 */
void hazeIcon(int x, int y, bool large_size, String icon_name, uint16_t icon_color)
{
    int scale = SMALL;
    int offset = 0;
    int linesize = 3;

    if (large_size)
    {
        scale = LARGE;
        offset = 7;
    }

    if (scale == SMALL)
    {
        linesize = 1;
    }

    if (icon_name.endsWith("n"))
    {
        addMoon(x, y + offset, scale);
        addFog(x, y + offset, scale * 1.4, linesize, GxEPD_WHITE);
    }
    else
    {
        addSun(x, y + offset, scale * 1.4, large_size, icon_color);
        addFog(x, y + offset, scale * 1.4, linesize, GxEPD_BLACK);
    }
}

/**
 * @brief Display the sun icon
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param direction Is the sun rising (SUN_UP) or setting (SUN_DOWN)
 */
void sunRiseSetIcon(uint16_t x, uint16_t y, sun_direction direction)
{
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
    if (direction == SUN_UP)
    {
        display.fillTriangle(x - r / 2 - 1, y + r - 2, x, y + r - 7, x + r / 2 + 1, y + r - 2, GxEPD_WHITE);
        display.drawLine(x - r / 2, y + r - 2, x, y + r - 6, GxEPD_BLACK);
        display.drawLine(x, y + r - 6, x + r / 2, y + r - 2, GxEPD_BLACK);
    }
    else
    {
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
void addMoon(int x, int y, int scale)
{
    if (scale == LARGE)
    {
        // time_t now = time(NULL);
        // struct tm * now_utc  = gmtime(&now);

        // drawMoon(x - 65, y - 60, now_utc->tm_mday, now_utc->tm_mon + 1, now_utc->tm_year + 1900, Hemisphere, 32, false);
        display.fillCircle(x - 37, y - 33, scale, GxEPD_WHITE);
        display.fillCircle(x - 25, y - 33, scale * 1.6, GxEPD_BLACK);
    }
    else
    {
        display.fillCircle(x - 20, y - 15, scale, GxEPD_WHITE);
        display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_BLACK);
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
void addSun(int x, int y, int scale, boolean icon_size, uint16_t icon_color)
{
    int linesize = 3;
    int dxo, dyo, dxi, dyi;

    if (icon_size == small_icon)
    {
        linesize = 1;
    }

    display.fillCircle(x, y, scale, icon_color);
    if (icon_color != GxEPD_RED)
    { // not day time or 2 colour display
        display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
    }

    for (float i = 0; i < 360; i = i + 45)
    {
        dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180);
        dxi = dxo * 0.6;
        dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180);
        dyi = dyo * 0.6;
        if (i == 0 || i == 180)
        {
            display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
            if (icon_size == large_icon)
            {
                display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y, GxEPD_BLACK);
                display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y, GxEPD_BLACK);
            }
        }
        if (i == 90 || i == 270)
        {
            display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1, GxEPD_BLACK);
            if (icon_size == large_icon)
            {
                display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0, GxEPD_BLACK);
                display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1, GxEPD_BLACK);
            }
        }
        if (i == 45 || i == 135 || i == 225 || i == 315)
        {
            display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
            if (icon_size == large_icon)
            {
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
void addCloud(int x, int y, int scale, int linesize)
{
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
void addRain(int x, int y, int scale, uint16_t colour)
{
    for (byte i = 0; i < 6; i++)
    {
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
void addSnow(int x, int y, int scale, uint16_t colour)
{
    int dxo, dyo, dxi, dyi;

    for (byte flakes = 0; flakes < 5; flakes++)
    {
        for (int i = 0; i < 360; i = i + 45)
        {
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
void addThunderStorm(int x, int y, int scale, uint16_t colour)
{
    y = y + scale / 2;

    for (byte i = 0; i < 5; i++)
    {
        display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, colour);
        if (scale != SMALL)
        {
            display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, colour);
            display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, colour);
        }
        display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, colour);
        if (scale != SMALL)
        {
            display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, colour);
            display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, colour);
        }
        display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, colour);
        if (scale != SMALL)
        {
            display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, colour);
            display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, colour);
        }
    }
}

/**
 * @brief Add some fog to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param scale Radius/size of the fog
 * @param linesize Used in the calculation of the size of the fog
 * @param colour Colour to draw the fog
 */
void addFog(int x, int y, int scale, int linesize, uint16_t colour)
{
    if (scale == SMALL)
    {
        y -= 10;
    }

    for (byte i = 0; i < 6; i++)
    {
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

/**
 * @brief Display the fact that we don't have any data to display!
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param large_size Used to set the font size to use 
 */
void noData(int x, int y, bool large_size)
{
    int scale = SMALL;
    int offset = 0;
    if (large_size)
    {
        scale = LARGE;
        offset = 7;
    }

    if (scale == LARGE)
    {
        display.setFont(&FreeMonoBold12pt7b);
    }
    else
    {
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
void addStar(int x, int y, star_size starsize)
{
    if (starsize == SMALL_STAR)
    {
        display.drawTriangle(x, y, x - 2, y + 3, x + 2, y + 3, GxEPD_WHITE);
        display.drawTriangle(x, y + 4, x - 2, y + 1, x + 2, y + 1, GxEPD_WHITE);
    }
    else if (starsize == MEDIUM_STAR)
    {
        display.drawTriangle(x, y, x - 4, y + 6, x + 4, y + 6, GxEPD_WHITE);
        display.drawTriangle(x, y + 8, x - 4, y + 2, x + 4, y + 2, GxEPD_WHITE);
    }
}

/**
 * @brief Draw a string to the screen - main function for all text written to the display
 * 
 * @param x Display x coordinates
 * @param y Display y coordinates
 * @param text Test to display
 * @param align Text alignment on the screen
 */
void drawString(int x, int y, String text, alignment align)
{
    int16_t x1, y1; // the bounds of x,y and w and h of the variable 'text' in pixels.
    uint16_t w, h;

    display.setTextWrap(false);
    display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
    if (align == RIGHT)
    {
        x = x - w;
    }

    if (align == CENTER)
    {
        x = x - w / 2;
    }

    display.setCursor(x, y + h);
    display.print(text);
}