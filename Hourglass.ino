#include "Ultrasonic.h"
#include "FastLED.h"
#include <SimpleKalmanFilter.h>

#define DATA_PIN 5

// How many leds in your strip?
#define NUM_LEDS 50

// Define the array of leds
CRGB leds[NUM_LEDS];

// Background defines the color off the lights that are not in the "meteor"
CRGB background(0x00, 0x00, 0x00);

// These value's are the ones that define the variables of the "meteor"
byte Size = 10;
byte Decay = 128;
bool Randoms = true;
int speed = 10;

//these variables controll the mapping value's of the speed sensor
byte minSpeed = 10;
byte maxSpeed = 150;

// initate the sensors
Ultrasonic saturationSonar(3);  //Sonar to controll colours, Signal = pin 2
Ultrasonic SpeedSonar(7, 8);    //Sonar to controll speed, Trig = pin 7, Echo = pin 8

// initiate the smoothening formula using the kalman filter
SimpleKalmanFilter simpleKalmanFilterSpeed(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilterSaturation(2, 2, 0.01);

void setup() {
  // init serial port
  Serial.begin(9600);

  // init ledstrip
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}


void loop() {
  // call for a "meteor"
  meteorRain(background, Size, Decay, Randoms, speed);
}


void meteorRain(CRGB ColorBackground, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {
  int satPreMap;
  int satMapped;

  int speedPreMap;
  int speedMapped;

  // set background color
  fill_solid(leds, NUM_LEDS, ColorBackground);

  for (int i = 0; i < NUM_LEDS + NUM_LEDS; i++) {

    // fade color to background color for all LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
      if ((!meteorRandomDecay) || (random(10) > 5)) {
        leds[j] = fadeTowardColor(leds[j], ColorBackground, meteorTrailDecay);
      }
    }

    satPreMap = simpleKalmanFilterSaturation.updateEstimate(saturationSonar.read());
    satMapped = map(satPreMap, 10, 250, 250, 0);

    if (satMapped < 5) satMapped = 0;
    if (satMapped > 240) satMapped = 255;


    // Serial.print("saturation ");
    // Serial.println(satMapped);

    // draw meteor with creepy maths
    for (int j = 0; j < meteorSize; j++) {
      if ((i - j < NUM_LEDS) && (i - j >= 0)) {
        leds[i - j] = CHSV(satMapped, satMapped, 255);
        // TODO: test if this reverses
        // leds[NUM_LEDS - i - j] = CHSV(satMapped, satMapped, 255);
      }
    }
    speedPreMap = simpleKalmanFilterSpeed.updateEstimate(SpeedSonar.read());

    speedMapped = map(speedPreMap, 0, 360, minSpeed, maxSpeed);

    // Serial.print("speed ");
    Serial.print(satMapped);
    // Serial.print(satPreMap);
    Serial.print(",");
    // Serial.println(speedPreMap);
    Serial.println(speedMapped);


    FastLED.show();
    delay(speedMapped);
  }
}

// Functions from Kriegsman example
CRGB fadeTowardColor(CRGB& cur, const CRGB& target, uint8_t amount) {
  nblendU8TowardU8(cur.red, target.red, amount);
  nblendU8TowardU8(cur.green, target.green, amount);
  nblendU8TowardU8(cur.blue, target.blue, amount);
  return cur;
}

// function used by "fadeTowardColor"
void nblendU8TowardU8(uint8_t& cur, const uint8_t target, uint8_t amount) {
  if (cur == target) return;

  if (cur < target) {
    uint8_t delta = target - cur;
    delta = scale8_video(delta, amount);
    cur += delta;
  } else {
    uint8_t delta = cur - target;
    delta = scale8_video(delta, amount);
    cur -= delta;
  }
}