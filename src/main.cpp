#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <AsyncDelay.h>
#include <user_interface.h>

// Include the correct display library
// For a connection via I2C using Wire include
#include <Wire.h>
#include <OneWire.h>
#include "SSD1306Wire.h"
#include <DallasTemperature.h>
//
#include "font-roboto-10.h"
#include "font-dialog-11.h"
#include "font-lato-14.h"
#include "font-dseg7.h"
#include "images.h"

#define DEG2RAD 0.0174532925
#define TIME_OFFSET 5

// Init GPS
SoftwareSerial serialGPS(13, 15);
TinyGPSPlus gps;

// Init temperature sensors
OneWire oneWire(2);
DallasTemperature tempSensors(&oneWire);

// Initialize the OLED display using Wire library
SSD1306Wire display1(0x3c, 4, 5);
SSD1306Wire display2(0x3d, 4, 5);

AsyncDelay delayGPS;
AsyncDelay delaySensors;

int gpsAltitude = 0;
double gpsCompass = 0;
double gpsSpeed = 0;
double gpsLat, gpsLong = 0;
double gpsSat = 0;
float tOutside = 0;
float tInside = 0;

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (serialGPS.available())
      gps.encode(serialGPS.read());
  } while (millis() - start < ms);
}

void drawRotatedBitmap(SSD1306Wire *display, int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *bitmap, uint16_t angle)
{

  //uint8_t w = pgm_read_byte(bitmap++);
  //uint8_t h = pgm_read_byte(bitmap++);

  int16_t newx, newy;
  uint8_t data = 0;

  float cosa = cos(angle * DEG2RAD), sina = sin(angle * DEG2RAD);

  x = x - ((w * cosa / 2) - (h * sina / 2));
  y = y - ((h * cosa / 2) + (w * sina / 2));

  for (int16_t j = 0; j < h; j++)
  {
    for (int16_t i = 0; i < w; i++)
    {
      if ((j * w + i) & 7)
        data <<= 1;
      else
        data = pgm_read_byte(bitmap++);

      newx = 0.5 + x + ((i * cosa) - (j * sina));
      newy = 0.5 + y + ((j * cosa) + (i * sina));

      if (data & 0x80)
      {
        display->setPixel(newx, newy);
        //display.drawPixel(newx, newy, 1);
      }
      //else            display.drawPixel(newx, newy, 0);
    }
  }
}

void drawAngularLine(int x, int y, int length, int rotation)
{
  float angleRadians = DEG2RAD * (rotation - 90);
  float x1 = x + cos(angleRadians) * length;
  float y1 = y + sin(angleRadians) * length;

  display1.drawLine(x, y, x1, y1);
}

void renderDisplay1()
{
  String strTemp = "";
  display1.clear();

  // draw compass info
  display1.drawXbm(80, 0, 48, 48, icon_nav_compass_48x);
  drawAngularLine(104, 24, 12, gpsCompass);
  display1.fillCircle(104, 24, 2);
  display1.setFont(DSEG7_Classic_Mini_Regular_28);
  display1.setTextAlignment(TEXT_ALIGN_LEFT);
  display1.drawString(2, 2, String(gpsCompass, 0));
  display1.drawCircle(display1.getStringWidth(String(gpsCompass, 0)) + 6, 8, 3);
  display1.drawCircle(display1.getStringWidth(String(gpsCompass, 0)) + 6, 8, 2);

  // speed
  display1.setFont(Lato_Thin_14);
  display1.setTextAlignment(TEXT_ALIGN_LEFT);
  display1.drawXbm(2, 38, 15, 14, icon_gauge_15x14);

  int speedNeedle = gpsSpeed <= 120 ? gpsSpeed : 120;
  speedNeedle = map(speedNeedle, 0, 120, -90, 90);
  drawAngularLine(9, 47, 4, speedNeedle);

  display1.drawString(20, 35, String(gpsSpeed, 0));
  display1.setFont(Dialog_plain_10);
  display1.drawString(48, 39, "KM/h");

  // bottom separator
  display1.drawLine(0, display1.height() - 10, display1.width(), display1.height() - 10);

  // location pin
  // display1.drawXbm(0, display1.height() - 8, 8, 8, icon_pin_8x);
  display1.setFont(Dialog_plain_10);
  display1.drawString(0, display1.height() - 10, "L: ");

  // location cordinates
  if (gps.location.isValid())
    strTemp = String(gpsLat, 3) + " : " + String(gpsLong, 3);
  else
    strTemp = "--.--- : --.---";

  display1.drawString(display1.getStringWidth("L: "), display1.height() - 10, strTemp);

  // sats
  if (gps.satellites.isValid())
    strTemp = "SAT:" + String(gpsSat, 0);
  else
    strTemp = "SAT: --";

  display1.drawString(display1.width() - display1.getStringWidth(strTemp), display1.height() - 10, strTemp);

  // show output
  display1.display();
}

bool secondState = true;
void renderDisplay2()
{
  String strTemp = "";
  display2.clear();

  // time hour & minutes
  display2.setFont(DSEG7_Classic_Mini_Regular_28);
  display2.setTextAlignment(TEXT_ALIGN_LEFT);
  // offset and 12h format
  byte h = hourFormat12();
  strTemp = h < 10 ? "0" + String(h) : String(h);
  display2.drawString(2, 2, strTemp);
  strTemp = minute() < 10 ? "0" + String(minute()) : String(minute());
  display2.drawString(6 + display2.getStringWidth("00"), 2, strTemp);

  // seconds
  if (secondState)
  {
    display2.fillCircle(50, 14, 2);
    display2.fillCircle(50, 26, 2);
  }
  secondState = !secondState;

  // day and month
  display2.setFont(Dialog_plain_11);
  display2.setTextAlignment(TEXT_ALIGN_CENTER);
  display2.drawString(110, 0, dayShortStr(weekday()));
  display2.drawString(111, 11, String(day()));
  display2.drawString(111, 22, monthShortStr(month()));

  // display template
  display2.drawXbm(0, 38, 128, 26, template_screen2_128x26);

  display2.setFont(Lato_Thin_14);
  display2.setTextAlignment(TEXT_ALIGN_LEFT);

  // altitude
  if (gpsAltitude >= 1000)
    strTemp = String(int(floor(gpsAltitude / 1000))) + "." + String(int(floor(gpsAltitude % 1000))).substring(0, 2) + "k";
  else
    strTemp = String(gpsAltitude);

  display2.drawString(12, display2.getHeight() - 20, strTemp);

  // temperature outside
  strTemp = (tOutside != -127) ? String(tOutside, 1) : "--";
  display2.drawString(60, display2.getHeight() - 20, strTemp);

  // temperature cabin
  strTemp = (tInside != -127) ? String(tInside, 1) : "--";
  display2.drawString(99, display2.getHeight() - 20, strTemp);

  display2.display();
}

// arduino fx
void setup()
{
  Serial.begin(115200);
  serialGPS.begin(9600);
  tempSensors.begin();

  Serial.println();
  Serial.println("Starting...");

  display1.init();
  display2.init();

  display1.flipScreenVertically();
  display2.flipScreenVertically();

  // display1.setFont(ArialMT_Plain_16);
  // display1.setTextAlignment(TEXT_ALIGN_LEFT);
  // display2.setFont(ArialMT_Plain_16);
  // display2.setTextAlignment(TEXT_ALIGN_LEFT);

  delayGPS.start(2000, AsyncDelay::MILLIS);
}

time_t lastTimeUpdate = -5;
// time_t lastDataUpdate = -1;
void loop()
{
  // update time
  if (gps.time.isValid() && gps.time.age() < 500 && now() - lastTimeUpdate > 5)
  {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    smartDelay(50);
    adjustTime(TIME_OFFSET * SECS_PER_HOUR);
    smartDelay(50);
    lastTimeUpdate = now() + SECS_PER_MIN;
  }

  if (delayGPS.isExpired())
  {
    unsigned long t = millis();

    // get compass data if valid
    gpsSpeed = gps.speed.isValid() ? gps.speed.kmph() : gpsSpeed;
    gpsLat = gps.location.isValid() ? gps.location.lat() : gpsLat;
    gpsLong = gps.location.isValid() ? gps.location.lng() : gpsLong;
    gpsCompass = gps.course.isValid() ? gps.course.deg() : gpsCompass;
    gpsSat = gps.satellites.isValid() ? gps.satellites.value() : gpsSat;
    gpsAltitude = gps.altitude.isValid() ? gps.altitude.feet() : gpsAltitude;

    // render screens
    smartDelay(10);
    renderDisplay1();
    smartDelay(10);
    renderDisplay2();
    smartDelay(10);

    Serial.print("Time ms: ");
    Serial.println(millis() - t);

    //delaySensors.start(100, AsyncDelay::MILLIS);
    delayGPS.repeat();
  }
  // refresh temperature data
  if (delaySensors.isExpired())
  {
    // Init temperature sensors
    // OneWire oneWire(2);
    // DallasTemperature tempSensors(&oneWire);
    // tempSensors.begin();
    tempSensors.requestTemperatures();
    tOutside = tempSensors.getTempCByIndex(0);
    tInside = tempSensors.getTempCByIndex(1);
    Serial.println("Read sensors");
  }

  //lastDataUpdate = now();

  // GPS data process
  smartDelay(100);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
