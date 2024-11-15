#include <Arduino.h>

#include "ros.h"
#include "ros/time.h"
#include <std_msgs/Int8.h>
#include <FastLED.h>

#define NUM_LEDS 174
#define DATA_PIN 2

// Define the array of leds
CRGB leds[NUM_LEDS];

uint8_t gHue0 = 0;
uint8_t gHue1 = 0;
uint8_t gHue2 = 0;

uint8_t step = 0; 

void LED_send_Callback(const std_msgs::Int8 &data);
void Effect_off();
ros::NodeHandle nodeHandle;
std_msgs::Int8 led_query;
std_msgs::Int8 led_reply;

ros::Publisher LED_pub("led_reply", &led_reply);
ros::Subscriber<std_msgs::Int8> LED_sub("led_query", LED_send_Callback);

unsigned long saveTime_pub = 0;

void LED_send_Callback(const std_msgs::Int8 &data)
{
    led_query = data;
} 

void setup() {
  delay(10);
  // - ROS
  nodeHandle.initNode();
  nodeHandle.getHardware()->setBaud(57600);
  delay(10);
  FastLED.addLeds<WS2812,DATA_PIN,GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255); // - 84
  delay(10);

  nodeHandle.advertise(LED_pub);
  nodeHandle.subscribe(LED_sub);
  delay(10);
  // -
  // - ROS
  while (!nodeHandle.connected())
  {   
    Effect_off();
    delay(10);
    nodeHandle.spinOnce();
  }
}

void fadeall(int scale) { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(scale); } } // - 250

void Effect_off(){
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

/* -- Move -- */
void Effect_1() {
	for(int i = 0; i < 58; i++) {
		leds[i] = CHSV(gHue0++, 255, 255);
		leds[i+58] = CHSV(gHue1++, 255, 255);
		leds[i+116] = CHSV(gHue2++, 255, 255);
		FastLED.show(); 
		fadeall(220);
    FastLED.delay(6);
  }
}

/* -- Parking -- */
void Effect_2() {
	for(int i = 0; i < NUM_LEDS/2; i++) {   
    fadeToBlackBy(leds, NUM_LEDS, 20);
		// leds[i] = CHSV(gHue0++,255,255);

    leds[NUM_LEDS/2 - i] = CRGB::Aqua;
    leds[NUM_LEDS/2 + i] = leds[NUM_LEDS/2 - i];
		FastLED.delay(8);
	}
}

/* -- Wait -- */
void Effect_3()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i ++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
  FastLED.show();
}

/* -- Stop -- */
void Effect_4()
{
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = CRGB::Purple;
  }
  delay(250);
  FastLED.show();
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = CRGB::Black;
  }
  delay(250);
  FastLED.show();
}

/* -- Error -- */
void Effect_5(){
  for(int i = 0; i < 43; i++) {  
    leds[i] = CRGB::Red;
    leds[i + 43] = CRGB::Black;
    leds[i + 86] = CRGB::Red;
    leds[i + 129] = CRGB::Black;
  }
  FastLED.show();
  delay(300);
  for(int i = 0; i < 43; i++) {  
    leds[i] = CRGB::Black;
    leds[i + 43] = CRGB::Red;
    leds[i + 86] = CRGB::Black;
    leds[i + 129] = CRGB::Red;
  }
  FastLED.show();
  delay(300);
}

/* -- Lifting UP/DOWN -- */
void Effect_6() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(gHue0++, 255, 255);
    FastLED.show(); 
    fadeall(253);
    delay(4);
  }

	// for(int i = (NUM_LEDS)-1; i >= 0; i--) {
	// 	leds[i] = CHSV(hue++, 255, 255);
	// 	FastLED.show();
	// 	fadeall();
	// 	delay(5);
	// }
}

void loop() {
  switch (led_query.data)
  // switch (6)
  {
    case 1:
      Effect_1();
      break;
    
    case 2:
      Effect_2();
      break;

    case 3:
      Effect_3();
      break;

    case 4:
      Effect_4();
      break;

    case 5:
      Effect_5();
      break;

    case 6:
      Effect_6();
      break;

    default:
      Effect_off();
      break;
  }

    // Effect_6();
    // Effect_1();
    // Effect_2();
    // Effect_3();
	// Effect_4();
    // Effect_5();

    // - ROS
    if ((millis() - saveTime_pub) > (500))
    {   
        saveTime_pub = millis();
        LED_pub.publish(&led_reply); 
    }
	/* -- -- -- -- */
	nodeHandle.spinOnce();
}