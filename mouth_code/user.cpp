#if 1  // Change to 0 to disable this code (must enable ONE user*.cpp only!)
// This file provides a crude way to "drop in" user code to the eyes,
// allowing concurrent operations without having to maintain a bunch of
// special derivatives of the eye code (which is still undergoing a lot
// of development). Just replace the source code contents of THIS TAB ONLY,
// compile and upload to board. Shouldn't need to modify other eye code.

// User globals can go here, recommend declaring as static, e.g.:
// static int foo = 42;
// FOR CONTROLLING THE LED STRIPE
#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"

#define PIN_WS2812B 17  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 12   // The number of LEDs (pixels) on WS2812B LED strip

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
int brightness = 80;
const uint16_t LED_OFF_MASK = 0b00000000000;      // all zeros
const uint16_t LED_LOW_MASK = 0b0000100000;       // led 5
const uint16_t LED_MEDLOW_MASK = 0b00001110000;   // leds 4 to 6
const uint16_t LED_MED_MASK = 0b00011111000;      // leds 3 to 7
const uint16_t LED_MEDHIGH_MASK = 0b00111111100;  // leds 2 to 8
const uint16_t LED_HIGH_MASK = 0b11111111111;     // leds 0 to 10 (all ones)

const uint32_t COLOR_WHITE = ws2812b.Color(255, 255, 255);
const uint32_t COLOR_BLUE = ws2812b.Color(0, 0, 255);
const uint32_t COLOR_GREEN = ws2812b.Color(0, 255, 0);
const uint32_t COLOR_RED = ws2812b.Color(255, 0, 0);

uint32_t my_color = COLOR_BLUE;

extern String incomingData;
extern bool serial_data_available;

void lightLEDs(uint16_t mask, uint32_t color = COLOR_BLUE) {
  for (int i = 0; i < NUM_PIXELS; ++i) {
    if (mask & (1 << i)) {
      ws2812b.setPixelColor(i, color);  // it only takes effect if pixels.show() is called
    }
  }
  ws2812b.show();  // update to the WS2812B Led Strip
}
/** /
void lightLEDs(uint16_t mask) {
  for (int i = 0; i < NUM_PIXELS; ++i) {
    if (mask & (1 << i)) {
      ws2812b.setPixelColor(i, ws2812b.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
    }
  }
  ws2812b.show();                                          // update to the WS2812B Led Strip
}
/**/
// *********************************************

// Called once near the end of the setup() function.
void user_setup(void) {
  ws2812b.begin();
  ws2812b.setBrightness(40);  // <- Solo una vez aquí
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  delay(1000);
}


// Called periodically during eye animation. This is invoked in the
// interval before starting drawing on the last eye so it won't exacerbate
// visible tearing in eye rendering.
// This function BLOCKS, it does NOT multitask with the eye animation code,
// and performance here will have a direct impact on overall refresh rates,
// so keep it simple. Avoid loops (e.g. if animating something like a servo
// or NeoPixels in response to some trigger) and instead rely on state
// machines or similar. Additionally, calls to this function are NOT time-
// constant -- eye rendering time can vary frame to frame, so animation or
// other over-time operations won't look very good using simple +/-
// increments, it's better to use millis() or micros() and work
// algebraically with elapsed times instead.
void user_loop(void) {
  ws2812b.clear();  // set all pixel colors to 'off'. It only takes effect if pixels.show() is called

  /** /
  for (int i = 0; i < NUM_PIXELS; i++) { // Bucle de 0 a 10
    ws2812b.setPixelColor(i, ws2812b.Color(0, 0, 255)); // LED 'i', Color Azul
    ws2812b.show();
    Serial.print("Encendiendo LED índice: "); Serial.println(i);
    delay(100); // Pequeña pausa para ver el efecto
  }

  delay(1000); // Espera 1 segundo con todos encendidos

  // Apagar todos los LEDs
  for (int i = 0; i < NUM_PIXELS; i++) { // Bucle de 0 a 10
    ws2812b.setPixelColor(i, ws2812b.Color(0, 0, 0)); // Apagado
  }
  ws2812b.show();
  Serial.println("Todos los LEDs apagados.");
  delay(2000); // Espera 2 segundos
  /**/

  /** /
  int randomNumber = random(4);
  if (randomNumber == 0) {
    ws2812b.clear();  // set all pixel colors to 'off'. It only takes effect if pixels.show() is called
    ws2812b.show(); 
  }
  if (randomNumber == 1) {
    lightLEDs(LED_LOW_MASK);
  }
  else if (randomNumber == 2) {
    lightLEDs(LED_MEDIUM_MASK);
  }
  else if (randomNumber == 3) {
    lightLEDs(LED_HIGH_MASK);
  }
/**/
  /**/
  if (Serial.available() > 0) {
    // String incomingData = Serial.readStringUntil('\n');
    incomingData = Serial.readStringUntil('\n');
    // Process the received string as needed
    Serial.println("[LED] Received: " + incomingData);

    if (incomingData.equals("color_white")) {
      my_color = COLOR_WHITE;
    } else if (incomingData.equals("color_blue")) {
      my_color = COLOR_BLUE;
    } else if (incomingData.equals("color_red")) {
      my_color = COLOR_RED;
    } else if (incomingData.equals("color_green")) {
      my_color = COLOR_GREEN;
    } else if (incomingData.equals("off")) {
      Serial.println("[LED] Received: off");
      lightLEDs(LED_OFF_MASK, my_color);
    } else if (incomingData.equals("low")) {
      Serial.println("[LED] Received: low");
      lightLEDs(LED_LOW_MASK, my_color);
    } else if (incomingData.equals("medium-low")) {
      Serial.println("[LED] Received: medium-low");
      lightLEDs(LED_MEDLOW_MASK, my_color);
    } else if (incomingData.equals("medium")) {
      Serial.println("[LED] Received: medium");
      lightLEDs(LED_MED_MASK, my_color);
    } else if (incomingData.equals("medium-high")) {
      Serial.println("[LED] Received: medium-high");
      lightLEDs(LED_MEDHIGH_MASK, my_color);
    } else if (incomingData.equals("high")) {
      Serial.println("[LED] Received: high");
      lightLEDs(LED_HIGH_MASK, my_color);
    } else {
      Serial.println("[LED] Received: Unknown string");
    }

    serial_data_available = true;
  }
  /**/

  /*
  Suppose we have a global bool "animating" (meaning something is in
  motion) and global uint32_t's "startTime" (the initial time at which
  something triggered movement) and "transitionTime" (the total time
  over which movement should occur, expressed in microseconds).
  Maybe it's servos, maybe NeoPixels, or something different altogether.
  This function might resemble something like (pseudocode):

  if(!animating) {
    Not in motion, check sensor for trigger...
    if(read some sensor) {
      Motion is triggered! Record startTime, set transition
      to 1.5 seconds and set animating flag:
      startTime      = micros();
      transitionTime = 1500000;
      animating      = true;
      No motion actually takes place yet, that will begin on
      the next pass through this function.
    }
  } else {
    Currently in motion, ignore trigger and move things instead...
    uint32_t elapsed = millis() - startTime;
    if(elapsed < transitionTime) {
      Part way through motion...how far along?
      float ratio = (float)elapsed / (float)transitionTime;
      Do something here based on ratio, 0.0 = start, 1.0 = end
    } else {
      End of motion reached.
      Take whatever steps here to move into final position (1.0),
      and then clear the "animating" flag:
      animating = false;
    }
  }
*/
}

#endif  // 0
