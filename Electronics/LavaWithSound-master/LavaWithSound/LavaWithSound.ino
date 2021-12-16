/*************************************************** 
  Lava - With Sound
  Mark Iuzzolino
  23-Apr-2019

  LEDs: Digital Pin 9
  Photo sensor: Analog Pin 0
 ****************************************************/
//include FastLed library for Neopixels
#include <FastLED.h>

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// -- MP3 SHIELD -- //
// define the pins used
#define CLK 13       // SPI Clock, shared with SD card
#define MISO 12      // Input data, from VS1053/SD card
#define MOSI 11      // Output data, to VS1053/SD card
// Connect CLK, MISO and MOSI to hardware SPI pins. 
// See http://arduino.cc/en/Reference/SPI "Connections"

//// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

uint8_t currTrack = 0;

#define DAY_TRACK 0
#define NIGHT_TRACK 1
const char * trackNames[2] = { "/L_DAY.MP3\0", "/L_NIGHT.MP3\0" }; // day track, night track.
 
// -- PHOTO SENSOR -- //
#define DARKNESS_THRESH 650 // value below which is considered "dark". This will need to be tuned.
#define CONTINUE_TIME 3000 // How many milliseconds we should continue before we transition day/night 
#define DELAY_TIME 1  

#define photocellPin 0     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
unsigned long lightChangeStart = 0;
unsigned long lightChangeEnd = 0;
bool lightOn;
bool transitioning = false;

// -- NEONUM_LEDS -- //
#define DATA_PIN 9 // Data pin that data will be written out over

#define NUM_LEDS 30

#define LAVA_COLOURS 8
#define LERP_STEPS 32

#define LIGHTS_ON_BRIGHTNESS 10
#define LIGHTS_OFF_BRIGHTNESS 130

int lavaIdx = 0;
uint8_t lavaColours[LAVA_COLOURS][3] = {
 {255, 65, 0},
 {255, 60, 0},
 {255, 55, 0},
 {255, 50, 0},
 {255, 30, 5},
 {255, 20, 5},
 {255, 10, 0},
 {255, 0, 0}
};

byte currColours[NUM_LEDS][3];
byte newColours[NUM_LEDS][3];
CRGB lava[NUM_LEDS];

void setup() {
  Serial.begin(9600);
  Serial.println("Lava With Sound");

  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
  
   if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }

  // Set volume for left, right channels. lower NUM_LEDSbers == louder volume! 
  // 0 is full volume, 255 is mute
  musicPlayer.setVolume(0, 0);
  // Note: The mp3 shield supports independent volume for each channel. 
  //However, we set it the same on both channels.

  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
 
  
  lightOn = isBright();
  startPlayingTrack();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(lava, NUM_LEDS);

  for (uint32_t i = 0; i < NUM_LEDS; ++i)
  {
    for (uint32_t x = 0; x < 3; ++x) 
    {
      currColours[i][x] = 0;
      newColours[i][x] = 0;
    }
  } // for i

}

void loop() {
  checkBrightnessChange();
  checkTransition();
  doLava();
  Serial.flush();
  
} // void loop()

bool isBright() 
{
  int lightLevel = analogRead(photocellPin);
  Serial.print("Light level = ");
  Serial.println(lightLevel);
  return (lightLevel > DARKNESS_THRESH); 
} // bool isBright()

void checkBrightnessChange()
{
  bool currLight = isBright();
  
  if (isBright() != lightOn  && !transitioning ) {
    Serial.println("Transition started");
    transitioning = true;
    lightChangeStart = millis();
    lightChangeEnd = lightChangeStart + CONTINUE_TIME;
    if (lightChangeEnd < lightChangeStart) 
    {
      // overflow occurred. ToDo: fix 
    }
    // store new light value
    lightOn = currLight;
  } // if
} // void checkBrightnessChange()

void checkTransition()
{
  if (!transitioning) return;
  
  if (millis() >= lightChangeEnd)
  {
    transitioning = false;
    Serial.print("Transition ended. Light is now ");
    Serial.println(lightOn ? "On" : "Off");
    
    startPlayingTrack();
  } // if
  
} // void checkTransition()

void startPlayingTrack()
{
  //selectTrackNUM_LEDSber
  currTrack = (lightOn ? DAY_TRACK : NIGHT_TRACK);
  
  if (!musicPlayer.stopped()) 
  {
    musicPlayer.stopPlaying();
  }
  
  musicPlayer.startPlayingFile(trackNames[currTrack]);
  Serial.print("Playing track: ");
  Serial.println(trackNames[currTrack]);
} // void startPlayingTrack()

void doLava()
{
  float lerpDiffs[NUM_LEDS][3];

  // move all colours over
  for (uint32_t i = NUM_LEDS-1; i > 0; --i)
  {
    for (uint32_t c = 0; c < 3; ++c)
    {
      newColours[i][c] = newColours[i-1][c];
    } // for c
  } // for i

  // Generate a new random colour from our presets
  int randIdx = (++lavaIdx) % LAVA_COLOURS; //random(LAVA_COLOURS);
  randIdx = random(LAVA_COLOURS);
  for (uint32_t c = 0; c < 3; ++c)
  {
    newColours[0][c] = lavaColours[randIdx][c];
  } // for c

  // Setup the lerpDiffs
  for (uint32_t i = 0; i < NUM_LEDS; ++i)
  {
    for (uint32_t c = 0; c < 3; ++c)
    {
      lerpDiffs[i][c] = ((float)currColours[i][c] - (float)newColours[i][c]) / (float)LERP_STEPS;
    } // for c
  } // for i
  
  for (uint32_t lerp = 0; lerp < LERP_STEPS; ++lerp)
  {
    for (uint32_t i = 0; i < NUM_LEDS; ++i)
    {
      for (uint32_t c = 0; c < 3; ++c)
      {
        currColours[i][c] = newColours[i][c]+(int)(lerpDiffs[i][c]*lerp);
      } // for c
    } // for i
    drawColours();
  } // for lerp

} // void doLava()


void drawColours()
{
  for (uint8_t i = 0; i < NUM_LEDS; ++i)
  {
    uint8_t adjustedBrightness; // = brightness[i] + twinkling[i];
    if (lightOn & !transitioning)
    {
      adjustedBrightness = LIGHTS_ON_BRIGHTNESS;//map(adjustedBrightness, 0, 255, 0, LIGHTS_ON_BRIGHTNESS);
    } 
    else 
    {
      adjustedBrightness = LIGHTS_OFF_BRIGHTNESS;
    }
    
    uint8_t red = currColours[i][0];
    red = map(red, 0, 255, 0, adjustedBrightness);
    
    uint8_t green = currColours[i][1];
    green = map(green, 0, 255, 0, adjustedBrightness);
    
    uint8_t blue = currColours[i][2];
    blue = map(blue, 0, 255, 0, adjustedBrightness);
    
    lava[i] = CRGB(red, green, blue);
    
  } // for i
  FastLED.show();
 // delay(DELAY_TIME);
  
} // void drawColors()
