/*
   Copyright 2021 Leon Kiefer

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

/*
    In Arduino IDE, set board type to Raspberry Pi Pico 2040.
    Set the USB Stack to Tiny USB or Adafruit Tiny USB.

    Additional info
    Xiao RP2040: https://wiki.seeedstudio.com/XIAO-RP2040/
    GitHub Source: https://github.com/Legion2/CorsairLightingProtocol
*/

/*
    iCue Lighting Node PRO Controller setup:
    Set both channels on the controller to 6 Strips / 60 LEDs. Change the settings for Channel 1 to values show below.

    Channel 1:      Strip #1 (00..09) --> CPU Temp (also reference for brightness set on Green channel)
                    Strip #2 (10..19) --> GPU Temp (not used for now)
                    Strip #3 (20..29) --> Fan 1..Fan 3 Color
                    Strip #4 (30..39) --> Fan 1..Fan 3 & Fan CPU sparkle/heartbeat color
                    Strip #5 (40..49) --> Fan CPU Color
                    Strip #6 (50..59) --> Not used --> Set to some fast chaning pattern to prevent watchdog activation
    Channel 2:      Normal operation 6 Strips / 60 LEDs
*/

#include <CorsairLightingProtocol.h>
#include <FastLED.h>

#define DATA_PIN_ICUE1        0      // Original iCue LED Strip 1 (60 LEDs) --> Not usable ADC for VSYS (GPIO29)
#define DATA_PIN_ICUE2        1      // Original iCue LED Strip 2 (60 LEDs)
#define LEDS_ICUE_STRIP      60

#define PWR_PIN_XIAO         11
#define DATA_PIN_XIAO        12
#define LEDS_XIAO             1
#define LED_RGBG             16
#define LED_RGBR             17
#define LED_RGBB             25
#define I2C_SDA               6
#define I2C_SCL               7
#define EXT_PIN_LIGHT        26      // ADC0 used for LDR light sensor

#define EXT_PIN_FAN1         27      // External Output Fan 1 I/O Pin
#define EXT_LEDS_FAN1        16      // External Output Fan 1 Number of LEDs
#define EXT_PIN_FAN2         29      // External Output Fan 2 I/O Pin
#define EXT_LEDS_FAN2        16      // External Output Fan 1 Number of LEDs
#define EXT_PIN_FAN3         28      // External Output Fan 3 I/O Pin --> GPIO 29 is not usable so asigned to iCue 1
#define EXT_LEDS_FAN3        16      // External Output Fan 1 Number of LEDs
#define EXT_PIN_EXTRA         3      // External Output Extra I/O Pin
#define EXT_LEDS_EXTRA       20      // External Output Extra Number of LEDs
#define EXT_PIN_CPU           4      // External Output CPU I/O Pin
#define EXT_LEDS_CPU         16      // External Output CPU Number of LEDs
#define EXT_PIN_CASE          2      // External Output Case I/O Pin
#define EXT_LEDS_CASE        16      // External Output Case Number of LEDs
#define EXT_LEDS_FAN123      EXT_LEDS_FAN1 + EXT_LEDS_FAN2 + EXT_LEDS_FAN3
#define LED_FPS              90      // Update rate for external LEDs (FastLED)

#define SPARKLES_NUM         15      // Number of sparkles
#define SPARKLES_CHANGE       3      // Randomness of sparkles
#define SPARKLES_FPS        200      // Number of FPS/Updates
#define SPARKLES_DELAY     1000 / SPARKLES_FPS   // Don't change, this is a calculated value
#define CRASH_COLOR           CRGB(64, 20, 0)
#define CRASH_SPARKLE         CRGB(128, 32, 0)

#define HEARTBEAT_MIN         0      // Minimum brights for heartbeat fading
#define HEARTBEAT_TEMPO     200
#define HEARTBEAT_PULSE     (int) ((float)HEARTBEAT_TEMPO * 0.35)

#define SCALING             30000    // Used for smooth motion on extra case fan, milliseconds for 1 round

CRGB ledsiCue1[LEDS_ICUE_STRIP];     // Create LED array for iCue LED strip 1 (color control data)
CRGB ledsiCue2[LEDS_ICUE_STRIP];     // Create LED array for iCue LED strip 2 (normal output)
//CRGB ledsXiao[LEDS_XIAO];            // Create LED array for Xiao onboard WS2812 --> Disabled due to limit of 7 pins for FastLED
CRGB ledsFan123[EXT_LEDS_FAN1 + EXT_LEDS_FAN2 + EXT_LEDS_FAN3];    // Create LED array for Fan 1, Fan 2 and Fan 3 as single array
CRGB ledsExtra[EXT_LEDS_EXTRA];      // Create LED array for Fan 4
CRGB ledsCPU[EXT_LEDS_CPU];          // Create LED array for CPU
CRGB ledsCase[EXT_LEDS_CASE];        // Create LED array for CASE

// Most ARM devices do not contain an EEPROM; we will use static storage for the Device ID
DeviceID deviceID = {0x9A, 0xDA, 0xA7, 0x8E};
CorsairLightingFirmwareStorageStatic firmwareStorage(deviceID);
CorsairLightingFirmware firmware(CORSAIR_LIGHTING_NODE_PRO, &firmwareStorage);
FastLEDController ledController(nullptr);
CorsairLightingProtocolController cLP(&ledController, &firmware);
CorsairLightingProtocolTinyUSBHID cHID(&cLP);

int rawbrightness = 0, calcbrightness = 100, avgbrightness = 1200;
int sparkles[SPARKLES_NUM][2];
int cpuheat = 1;
bool hasSerialData = false;

void setup() {
  pinMode(EXT_PIN_FAN1, OUTPUT);
  pinMode(EXT_PIN_FAN2, OUTPUT);
  pinMode(EXT_PIN_FAN3, OUTPUT);
  pinMode(EXT_PIN_EXTRA, OUTPUT);
  pinMode(EXT_PIN_CPU, OUTPUT);
  pinMode(EXT_PIN_CASE, OUTPUT);
  pinMode(LED_RGBR, OUTPUT);
  pinMode(LED_RGBG, OUTPUT);
  pinMode(LED_RGBB, OUTPUT);
  pinMode(PWR_PIN_XIAO, OUTPUT);
  delay(1);
  digitalWrite(LED_RGBR, HIGH);
  digitalWrite(LED_RGBG, HIGH);
  digitalWrite(LED_RGBB, HIGH);
  digitalWrite(PWR_PIN_XIAO, LOW);   // Power-off onboard WS2812 LED

  analogReadResolution(12);

	cHID.setup();

  // Add all external LED connections to FastLED
	FastLED.addLeds<WS2812B, DATA_PIN_ICUE1, GRB>(ledsiCue1, LEDS_ICUE_STRIP);
	FastLED.addLeds<WS2812B, DATA_PIN_ICUE2, GRB>(ledsiCue2, LEDS_ICUE_STRIP);
	//FastLED.addLeds<WS2812B, DATA_PIN_XIAO, GRB>(ledsXiao, LEDS_XIAO);      // Disables since FastLED can't handle more than 8 channels
	FastLED.addLeds<WS2812B, EXT_PIN_EXTRA, GRB>(ledsExtra, EXT_LEDS_EXTRA);
	FastLED.addLeds<WS2812B, EXT_PIN_CPU, GRB>(ledsCPU, EXT_LEDS_CPU);
	FastLED.addLeds<WS2812B, EXT_PIN_CASE, GRB>(ledsCase, EXT_LEDS_CASE);
  // Fan 1, Fan 2 and Fan 3 are chained in the same LED array but with individual external data pins
	FastLED.addLeds<WS2812B, EXT_PIN_FAN1, GRB>(ledsFan123, 0, EXT_LEDS_FAN1);
	FastLED.addLeds<WS2812B, EXT_PIN_FAN2, GRB>(ledsFan123, EXT_LEDS_FAN1, EXT_LEDS_FAN2);
	FastLED.addLeds<WS2812B, EXT_PIN_FAN3, GRB>(ledsFan123, EXT_LEDS_FAN1 + EXT_LEDS_FAN2, EXT_LEDS_FAN3);

  // https://fastled.io/docs/group___color_enums.html
	// FastLED.setCorrection(TypicalSMD5050);
  FastLED.setBrightness(64);
  FastLED.setCorrection(TypicalSMD5050);

  // Add FastLED LED strip data to iCue controller
	ledController.addLEDs(0, ledsiCue1, LEDS_ICUE_STRIP);
	ledController.addLEDs(1, ledsiCue2, LEDS_ICUE_STRIP);

  for(int i = 0; i< SPARKLES_NUM; i++) {
    sparkles[i][0] = -1;      // Sparkle brightness, -1 means available for new position
    sparkles[i][1] = 0;       // Sparkle position
  }

  // Start Watchdog Timer
  rp2040.wdt_begin(1000);     // Set watchdog timer to 1000 milliseconds of no response
}

void loop() {
  static unsigned long dlyUpdateStatus;
  static unsigned long dlyUpdateLEDs;
  static unsigned long dlySerialData;

	cHID.update();

	// Check iCue data for LED updates
  if(ledController.updateLEDs()) {
    dlyUpdateStatus = millis();
    dlySerialData = millis();
    hasSerialData = true;
    digitalWrite(LED_RGBB, HIGH);
	}
  // Blink Green RGB LED if data is received and there's an iCue update
  if(millis() - dlyUpdateStatus >= 5)
    digitalWrite(LED_RGBG, HIGH);
  else
    digitalWrite(LED_RGBG, LOW);
  // If no serial data is received after 3 seconds, set to "safe mode" to stop iCue Circus Lights
  if(millis() - dlySerialData >= 3000) {
    hasSerialData = false;
    digitalWrite(LED_RGBG, HIGH);
    digitalWrite(LED_RGBB, LOW);
  }


  // Update external LEDs
  if(millis() - dlyUpdateLEDs >= 1000 / LED_FPS) {
    dlyUpdateLEDs = millis();
    //ledsXiao[0] = ledsiCue1[0];
    //ledsXiao[0].fadeToBlackBy(192); // Dim the onboard WS2812 (limited current)

    get_cpuheat();
    run_sparkles();
    run_heartbeat();
    run_heatgauge();
    run_casefans();

    // Change overall brightness based on light sensor
    rawbrightness = analogRead(EXT_PIN_LIGHT);
    avgbrightness = (avgbrightness * 4 + rawbrightness) / 5;
    calcbrightness = map(constrain(rawbrightness, 100, 1900), 100, 1900, 224, 60);
    if(!hasSerialData)
      calcbrightness = 32;
    FastLED.setBrightness(calcbrightness);

		FastLED.show();

    rp2040.wdt_reset();   // Reset the watchdog timer after updating the LEDs
  }
}


void get_cpuheat() {
  // Get the average cpu temp over the last 5 seconds, returns value between 1 and 100
  static unsigned long delayHeat;
  static int heataveraged = 0;

  if(millis() - delayHeat >= 500) {
    delayHeat = millis();

    int heat = (int) map((long) ledsiCue1[2].r, 0, (long) ledsiCue1[2].g, 10, 100);

    heataveraged = (heataveraged * 9 + heat) / 10;
    cpuheat = heataveraged;
  }
}


void run_heatgauge() {
  static unsigned long delayCircle;
  static bool firstrun = true;
  static CHSV temprange[32];
  double msecPos, msecPos2, y, a;
  int msec, whatled;

  if(firstrun) {
    firstrun = false;
    float step = (48.0 - 0.0) / 32.0;         // HSV 0 = Red, HSV 48 = Yellow/Orange
    for(int i = 0; i < 32; i++) {
      temprange[i] = CHSV(0 + (int) (step * (float) i), 255, 255);
    }
  }

  for(int i = 0; i < EXT_LEDS_EXTRA; i++) {
    ledsExtra[i] = CRGB::Black;
  }

  msec = millis() % SCALING;
  msecPos = msec / (float)SCALING * EXT_LEDS_EXTRA; 
  msecPos2 = ((msec + SCALING / 2) % SCALING) / (float)SCALING * EXT_LEDS_EXTRA;
  for(int i = 0; i < EXT_LEDS_EXTRA; i++){
    if(msecPos > EXT_LEDS_EXTRA / 4 && msecPos < 3 * (EXT_LEDS_EXTRA / 4)) {
      a = i - msecPos;     // 'msecPos' determines where the peak is along the strip, change for different application
      a = -0.3 * a * a;             // adjust the 0.x to change the width - smaller number = wider pulse, 1.2 max for smooth movement
      y = 240 * pow(2, a); // adjust the 255 to change brightness
      whatled = i;
    } else {
      a = i - msecPos2;     // 'msecPos' determines where the peak is along the strip, change for different application
      a = -0.3 * a * a;             // adjust the 0.x to change the width - smaller number = wider pulse, 1.2 max for smooth movement
      y = 240 * pow(2, a); // adjust the 255 to change brightness
      whatled = (i + EXT_LEDS_EXTRA / 2) % EXT_LEDS_EXTRA;
    }
    int templed = map(cpuheat, 0, 100, 31, 0);
    ledsExtra[whatled] = temprange[templed];
    ledsExtra[whatled] = ledsExtra[whatled].fadeToBlackBy(255 - y);
  }
}


void run_sparkles() {
  CRGB fancolor, sparklecolor;

  fancolor = ledsiCue1[22]; //COLOR_FAN;    // Use the color of the 3rd block in iCue
  sparklecolor = ledsiCue1[32]; //COLOR_SPARKLE;    // Use the color of the 4th block in iCue
  if(!hasSerialData) {
    fancolor = CRASH_COLOR;
    sparklecolor = CRASH_SPARKLE;
  }
  for(int i = 0; i < EXT_LEDS_FAN123; i++) {
    ledsFan123[i] = fancolor; //ledsChannel1[0];
  }
  int sparkles_speed = cpuheat / 6;
  if(sparkles_speed < SPARKLES_CHANGE) 
    sparkles_speed = SPARKLES_CHANGE;
  for(int i = 0; i < SPARKLES_NUM; i++) {
    if(sparkles[i][0] < 0 && random(2000) < sparkles_speed) {
      sparkles[i][0] = random(128, 255);    // Choose random brightness
      sparkles[i][1] = random(EXT_LEDS_FAN123);    // Choose position
    }
    if(sparkles[i][0] > 0) {
      ledsFan123[sparkles[i][1]] = nblend(ledsFan123[sparkles[i][1]], sparklecolor, sparkles[i][0]); // update leds[0] with the HSV result      
      sparkles[i][0] -= 4;
      // Once fade out, set color back to background color
      if(sparkles[i][0] <= 10) {
        ledsFan123[sparkles[i][1]] = fancolor;
        sparkles[i][0] = -1;
      }
    }
  }
}


void run_heartbeat() {
  static int delayBeat;
  static int cyclecountertop = 0, blendratetop = 0, blendratebottom = 0;

  CRGB beatcolor = ledsiCue1[32]; //COLOR_SPARKLE;    // Use the color of the 4th block in iCue
  CRGB basecolor = ledsiCue1[42];
  if(!hasSerialData) {
    basecolor = CRASH_COLOR;
    beatcolor = CRASH_SPARKLE;
  }

  int heat = (int) map((long) cpuheat, 0,  100, 0, EXT_LEDS_CPU - 1);
  for(int i = 0; i < EXT_LEDS_CPU; i++) {
    ledsCPU[i] = basecolor;
  }

  if(millis() - delayBeat >= 1 + cpuheat / 10) {
    cyclecountertop++;
    if(cyclecountertop >= 160 - cpuheat)
      cyclecountertop = 0;
    if(cyclecountertop == 0)
      blendratetop = 80;
    if(cyclecountertop == 35)
      blendratebottom = 60;
  }
  for(int i = 0; i < EXT_LEDS_CPU / 2; i++) {
    ledsCPU[i * 2] = nblend(ledsCPU[i * 2], beatcolor, blendratetop);
    ledsCPU[i * 2 + 1] = nblend(ledsCPU[i * 2 + 1], beatcolor, blendratebottom);
  }
  blendratetop--;
  if(blendratetop < 0)
    blendratetop = 0;
  blendratebottom--;
  if(blendratebottom < 0)
    blendratebottom = 0;
}


void run_casefans() {
  CRGB fancolor = ledsiCue1[22]; //COLOR_FAN;    // Use the color of the 3rd block in iCue
  CRGB sparklecolor = ledsiCue1[32]; //COLOR_SPARKLE;    // Use the color of the 4th block in iCue
  if(!hasSerialData) {
    fancolor = CRASH_COLOR;
    sparklecolor = CRASH_SPARKLE;
  }
  for(int i = 0; i < EXT_LEDS_CASE; i++) {
    ledsCase[i] = fancolor; //ledsChannel1[0];
  }
}