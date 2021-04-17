#include <Arduino.h>
#include <TimeLib.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <simple_matrix.h>
#include <OneButton.h>

//8x8 segments amount
#define MAX_DEVICES 4
//i2c pins
#define CLK_PIN   13
#define DATA_PIN  11
#define CS_PIN    10

//#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
//other pins
#define BTN             3
#define LIGHT_SENSOR    A0
#define THERMO          4

// Time drifting compensation
#define YR_ADDR         0
#define MO_ADDR         1
#define DA_ADDR         2
#define WD_ADDR         3
#define HR_ADDR         4
#define MI_ADDR         5
#define SC_ADDR         6
#define SECS_A_DAYx100  1000

//real time clock module object
RTC_DS1307 Rtc;
//display object
simpleMatrix ledMatrix = simpleMatrix(CS_PIN,true);
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
void checkTimeDrift();
time_t getLastSetTime();
time_t getTimeUnix();
void buttonInterrupt();

void setup() {
  //Input Pin setup
  pinMode(BTN, INPUT_PULLUP);
  
  //initializing methods
  Rtc.begin();
  sensors.begin();
  setSyncProvider(getTimeUnix);
  ledMatrix.begin();

  ledMatrix.setIntensity(0);
  updateTimeToRtc();

  //short click
  button.attachClick(offsetMinutes);
  //long, 2s press
  button.attachLongPressStop(updateTimeToRtc);
  button.setPressTicks(2000);
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
    static unsigned int iterator;
    refreshDisplay();
    lastT = millis();

    // Every hour
    if(++iterator>3600)
    {
      checkTimeDrift();
      iterator=0;
    }
  }

}

void refreshDisplay()
{
  //if the light sensor indicates it's touched, show temperature
  //otherwise show time
  if(isLightSensorTriggered(brightnessMeasure()))
    showTemperature(getTemperature());
  else showTime(hour(),minute());
}

time_t getTimeUnix()
{
  return Rtc.now().unixtime();
}

void showTime(uint8_t h,uint8_t m)
{
  char hhmm_str[6];
  sprintf(hhmm_str,"%02d:%02d",h,m);

  ledMatrix.print(hhmm_str,2);
}

void showTemperature(float t)
{
  char temp[6];
  dtostrf(t,3,1,temp);
  strcat(temp,"C");

  ledMatrix.print(temp,2);
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

// Every RTC (especially with no temp. compensation) builds some time drift over time.
// To compensate it, we check how much time elapsed from the last calibration
// And we multiply this by an empirically obtained value (drift per day)
void checkTimeDrift()
{
  time_t elapsedTime = now()-getLastSetTime();
  int secsDrift = elapsedTime/86400 * SECS_A_DAYx100 / 100;

  // If the drift is big enough, compensate it
  if(secsDrift>=10)
  {
    adjustTime(-secsDrift);
    updateTimeToRtc();
  }
}

// Read the date of last time update/set/calibration of RTC from built-in memory
time_t getLastSetTime()
{
  tmElements_t lastSet;
  lastSet.Day = Rtc.readnvram(DA_ADDR);
  lastSet.Month = Rtc.readnvram(MO_ADDR);
  lastSet.Year = Rtc.readnvram(YR_ADDR);
  //lastSet.Wday = Rtc.readnvram(WD_ADDR);
  lastSet.Hour = Rtc.readnvram(HR_ADDR);
  lastSet.Minute = Rtc.readnvram(MI_ADDR);
  lastSet.Second = Rtc.readnvram(SC_ADDR);
  return makeTime(lastSet);
}

void updateTimeToRtc()
{
  tmElements_t nowElements;
  breakTime(now(),nowElements);
  Rtc.adjust(DateTime(now()));
  Rtc.writenvram(YR_ADDR,nowElements.Year);
  Rtc.writenvram(MO_ADDR,nowElements.Month);
  Rtc.writenvram(DA_ADDR,nowElements.Day);
  //Rtc.writenvram(WD_ADDR,nowElements.Wday);
  Rtc.writenvram(HR_ADDR,nowElements.Hour);
  Rtc.writenvram(MI_ADDR,nowElements.Minute);
  Rtc.writenvram(SC_ADDR,nowElements.Second);

  setSyncProvider(getTimeUnix);
}