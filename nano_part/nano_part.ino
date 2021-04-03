#include <IRremote.h>
#include <CppList.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define Roll_left 4
#define Roll_right 5
#define Floor_left 6
#define Floor_right 7

#define NUMPIXELS 30
#define CALIBRATIONTIME 20000

int WIPE_COUNT = 0;

int RECEIVERS = 3;

const int LeftIr = 3;                             //Variable
const int CenterIr = 9;                           //for
const int RightIr = 10;                            //GPIO

long range_time, range_time_bumper;

unsigned long bit_L[3];
unsigned long bit_C[3];
unsigned long bit_R[3];

unsigned long bit_T[3];


bool buttonState = LOW;


int A_variable;
int B_variable;
int C_variable;
int led_val;
String command; //global variable
int led_pin_user[3] = {4, 5, 7};




IRrecv **irrecvs;
decode_results results;


Adafruit_NeoPixel RL_pixels = Adafruit_NeoPixel(NUMPIXELS, Roll_left, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel RR_pixels = Adafruit_NeoPixel(NUMPIXELS, Roll_right, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel FL_pixels = Adafruit_NeoPixel(NUMPIXELS, Floor_left, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel FR_pixels = Adafruit_NeoPixel(NUMPIXELS, Floor_right, NEO_GRB + NEO_KHZ800);



unsigned long pixelsInterval = 50; // the time we need to wait
unsigned long colorWipePreviousMillis = 0;
unsigned long theaterChasePreviousMillis = 0;
unsigned long theaterChaseRainbowPreviousMillis = 0;
unsigned long rainbowPreviousMillis = 0;
unsigned long rainbowCyclesPreviousMillis = 0;

int theaterChaseQ = 0;
int theaterChaseRainbowQ = 0;
int theaterChaseRainbowCycles = 0;
int rainbowCycles = 0;
int rainbowCycleCycles = 0;

uint16_t currentPixel = 0;// what pixel are we operating on




void setup()
{
  currentPixel = 0;
  Serial.begin(115200);
  irrecvs = (IRrecv **)malloc(RECEIVERS * sizeof(int));
  irrecvs[0] = new IRrecv(LeftIr); // Receiver #0: pin 4
  irrecvs[1] = new IRrecv(CenterIr); // Receiver #1: pin 5
  irrecvs[2] = new IRrecv(RightIr); // Receiver #2: pin 6

  for (int i = 0; i < RECEIVERS; i++)
  {
    irrecvs[i]->enableIRIn();
  }

#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif

  RL_pixels.begin();
  RR_pixels.begin();
  FL_pixels.begin();
  FR_pixels.begin();

  /* my eyes!!!!! ehhh..... */
//  RL_pixels.setBrightness(100);
//  RR_pixels.setBrightness(100);
//  FL_pixels.setBrightness(100);
//  FR_pixels.setBrightness(100);

}

void loop() {
  if (Serial.available() > 0) {
    Change_Value_in_Serial();
  }

  for (int i = 0 ; i < RECEIVERS ; i++) {
    if (irrecvs[i]->decode(&results)) {

      switch (results.value) {
        case 0xFFE21D :
          bit_L[i] = B001;
          break;
        case 0xFFA25D :
          bit_C[i] = B010;
          break;
        case 0xFF629D :
          bit_R[i] = B100;
          break;
        default:
          bit_L[i] = B000;
          bit_C[i] = B000;
          bit_R[i] = B000;
      }
      irrecvs[i]->resume();
    }
  }


  if (millis() >= range_time) {
    int data = 0;
    for (int i = 0; i < 3; i++) {
      bit_T[i] = bit_L[i] + bit_C[i] + bit_R[i];
    }

    data =  bit_T[0] << 6 | bit_T[1] << 3 | bit_T[2] ;
    Serial.println(data);

    range_time = millis() + 25;
  }
}


void Change_Value_in_Serial() {
  char c = Serial.read();

  if (c == '\n') {

    String part1;
    part1 = command;
    led_val = part1.toInt();
    intepret(led_val);

    command = "";
  }

  else {
    command += c;
  }
}



void intepret(int _led_val) {
  char led_arr[4];
  sprintf(led_arr, "%d", _led_val);
  execute(led_arr[0], &RL_pixels);
  execute(led_arr[1], &RR_pixels);
  execute(led_arr[2], &FL_pixels);
  execute(led_arr[3], &FR_pixels);
}


void execute(char _led_num , Adafruit_NeoPixel *pixels) {
  int led_num_data = _led_num - '0';
  switch (int(led_num_data)) {
    case 1: red(pixels);
      break;

    case 2: yellow(pixels);
      break;

    case 3: green(pixels);
      break;

    case 4: blue(pixels);
      break;

    case 5: turn_off(pixels);
      break;

    case 6: rainbow(5, pixels);
      break;

    case 7:
      for (int i = 0; i < 5; i++) {
        color_relay(pixels);
      }
      break;

      //    case 7: greenWipe(pixels);
      //      break;

  }

}


void red(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip->setPixelColor(i, RL_pixels.Color(150, 0, 0)); // red
    strip->show();
  }
}

void yellow(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip->setPixelColor(i, RL_pixels.Color(250, 250, 0)); // yellow
    strip->show();
  }
}

void green(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip->setPixelColor(i, RL_pixels.Color(0, 115, 0)); // green
    strip->show();
  }
}

void blue(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip->setPixelColor(i, RL_pixels.Color(0, 0, 250)); // blue
    strip->show();
  }
}


void turn_off(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip->setPixelColor(i, RL_pixels.Color(0, 0, 0)); // off
    strip->show();
  }
}




void greenWipe(Adafruit_NeoPixel * strip) {

  unsigned long WIPE_FREQUENCY = 1;
  unsigned long WIPE_CYCLE_T = 1000 / WIPE_FREQUENCY;

  if (millis() >= WIPE_CYCLE_T * WIPE_COUNT) {

    strip->setPixelColor(currentPixel, RL_pixels.Color(0, 115, 0));
    strip->show();
    currentPixel++;
    if (currentPixel == NUMPIXELS) {
      currentPixel = 0;
    }

    WIPE_COUNT++;
  }
}

void color_relay(Adafruit_NeoPixel * strip) {
  for (int i = 0; i < NUMPIXELS; i++) {

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip->setPixelColor(i, RL_pixels.Color(0, 115, 0));
    strip->show();

    delay(10); // Delay for a period of time (in milliseconds).
  }
}



void rainbow(uint8_t wait, Adafruit_NeoPixel * strip) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip->numPixels(); i++) {
      strip->setPixelColor(i, Wheel((i + j) & 255, strip));
    }
    strip->show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos , Adafruit_NeoPixel * strip) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
