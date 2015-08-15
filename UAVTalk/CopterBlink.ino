#include "UAVTalk.h"
#include <Adafruit_NeoPixel.h>

#define PIN 4
#define PIXEL 16;

Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

const uint32_t RED = ring.Color(255, 0, 0);
const uint32_t GREEN = ring.Color(0, 255, 0);
const uint32_t BLUE = ring.Color(0, 0, 255);
const uint32_t MAGENTA = ring.Color(255, 0, 255);
const uint32_t YELLOW = ring.Color(255, 255, 0);
const uint32_t PINK = ring.Color(219,112,147);
const uint32_t SKY_BLUE = ring.Color(135,206,255);
const uint32_t MAIZE = ring.Color(128,158,10);
const uint32_t LAVENDER = ring.Color(88,2,163);
const uint32_t SEA_FOAM = ring.Color(32,178,170);
const uint32_t SPleds = ring.Color(102,205,0);
const uint32_t DARK_ORANGE = ring.Color(237,180,6);
const uint32_t ORANGE = ring.Color(237,120,6);
const uint32_t WHITE = ring.Color(255,255,255);
const uint32_t BLACK = ring.Color(0,0,0);

void setup() {
//  Serial.begin(57600);
  ring.begin();
  ring.show(); // Initialize all pixels to 'off'
  ring.setBrightness(100);
  ring.setPixelColor(15,0x202020); // checkpixel
}

void loop() {
  // int returncode = uavtalk_read();

   // armed
   if (osd_armed > 0) { // does only work sometimes!
     ring.setPixelColor(0, RED); // armed = red
   } else {
     ring.setPixelColor(0, BLACK);
   }

   // yaw
   uint32_t value = (osd_yaw + 180) * 255 / 360; // 0 - 255
   if (value > 255) {
     value = 255;
   }
   uint32_t color =  value;
   ring.setPixelColor(1, color);
   
   // throttle (0-100)
   value = osd_throttle;
   value = value * 2.55;
   color = value << 16; // red
   ring.setPixelColor(2, color);

   // flight mode
   if (osd_mode == 1) {
     ring.setPixelColor(3, GREEN); // green
   } else if (osd_mode == 2) {
     ring.setPixelColor(3, YELLOW); // yellow
   } else if (osd_mode == 3) {
     ring.setPixelColor(3, RED); // red
   } else {
     ring.setPixelColor(3, LAVENDER); // 
   }

   ring.show();
}

