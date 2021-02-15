#include <Arduino.h>
#include <MD_Parola.h>
#include <TimeLib.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <OneButton.h>

//8x8 segments amount
#define MAX_DEVICES 4
//i2c pins
#define CLK_PIN   13
#define DATA_PIN  11
#define CS_PIN    10

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
//other pins
#define BTN  3
#define LIGHT_SENSOR     A0
#define THERMO     4

//real time clock module object
RTC_DS1307 Rtc;
//display object
MD_Parola ledMatrix = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
//DS18xx temperature sensor via OneWire interface
OneWire oneWire(THERMO);
DallasTemperature sensors(&oneWire);
//button object for easy debounce and short/long press detection
OneButton button = OneButton(BTN);

//my odd idea of using optoresistor as a proximity/touch sensor...
//recent history of brightness measurement
int prevBrightnessValues[10];

void refreshDisplay();
void loop1Hz();
void showTime(uint8_t h,uint8_t m);
void showTemperature(float t);
float getTemperature();
bool isLightSensorTriggered(int currentMeasurement);
int brightnessMeasure();
void offsetMinutes();
void updateTimeToRtc();
time_t getTimeUnix();

void setup() {

  //Input Pin setup
  pinMode(BTN, INPUT_PULLUP);
  
  //initializing methods
  Rtc.begin();
  sensors.begin();
  setSyncProvider(getTimeUnix);
  ledMatrix.begin();

  ledMatrix.setIntensity(0);

  //short click
  button.attachClick(offsetMinutes);
  //long, 1s press
  button.attachLongPressStop(updateTimeToRtc);
  button.setPressTicks(1000);
}

void loop() {
  button.tick();
  loop1Hz();
}

void loop1Hz()
{
  static unsigned long lastT;
  if(millis()-lastT>1000)
  {
    refreshDisplay();
    lastT = millis();
  }
}

void refreshDisplay()
{
  //if the light sensor indicates it's touched, show temperature
  //otherwise show time
  if(isLightSensorTriggered(brightnessMeasure()))
    showTemperature(getTemperature());
  else showTime(hour(),minute());
  ledMatrix.displayAnimate();
}

time_t getTimeUnix()
{
  return Rtc.now().unixtime();
}

void showTime(uint8_t h,uint8_t m)
{
  char hhmm_str[6];
  sprintf(hhmm_str,"%02d:%02d",h,m);

  ledMatrix.displayText(hhmm_str,PA_CENTER,0,0,PA_PRINT,PA_NO_EFFECT);
}

void showTemperature(float t)
{
  char temp[6];
  dtostrf(t,3,1,temp);
  strcat(temp,"C");

  ledMatrix.displayText(temp,PA_CENTER,0,0,PA_PRINT,PA_NO_EFFECT);
}

float getTemperature()
{
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

int brightnessMeasure()
{
  static byte i=0;
  int currentMeasurement = analogRead(LIGHT_SENSOR);
  prevBrightnessValues[i++]=currentMeasurement;
  if(i>=10)i=0;
  return currentMeasurement;
}

bool isLightSensorTriggered(int currentMeasurement)
{
  int sum=0;
  for(int i=0; i<10; i++)
  {
    sum+=prevBrightnessValues[i];
  }

  return (sum/10 - currentMeasurement) > 30 && currentMeasurement<50;
}

void offsetMinutes()
{
  //to implement manual time adjustment with only one button
  //I subordinated the direction of change to the brightness (touching or not)
  if(brightnessMeasure()<80)adjustTime(60);
  else adjustTime(-60);
  refreshDisplay();
  //we don't want to sync the time with rtc anymore,
  //because syncing would disrupt time incrementing 
  setSyncProvider(nullptr);
}

void updateTimeToRtc()
{
  Rtc.adjust(DateTime(now()));
  setSyncProvider(getTimeUnix);
}