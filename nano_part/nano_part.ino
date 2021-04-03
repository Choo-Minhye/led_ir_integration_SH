// 저희가 나노를 Serial 통신으로 이용하는 이유는, opencr 핀의 부족과,
// irremote library를 opencr board 는 지원하지 않기 때문입니다!!
// nano 에는 led(4,5,6,7), docking 의 수신 ir(3, 9, 10)이 올라가 있습니다. 

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

int led_val;
String command; //global variable


// ir 수신부 라이브러리 불러오기
IRrecv **irrecvs;
decode_results results;

// led 라이브러리 객체 선언
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
  // Serial 통신은 baudrate 를 맞춰줘야 하기 때문에, opencr과 동일한 115200 으로 설정해줍니다!
  Serial.begin(115200);

  // ir 수신부 동적할당 및 객체 선언
  irrecvs = (IRrecv **)malloc(RECEIVERS * sizeof(int));
  irrecvs[0] = new IRrecv(LeftIr); // Receiver #0: pin 3
  irrecvs[1] = new IRrecv(CenterIr); // Receiver #1: pin 9
  irrecvs[2] = new IRrecv(RightIr); // Receiver #2: pin 10

  for (int i = 0; i < RECEIVERS; i++)
  {
    // ir receiever 동작을 활성화
    irrecvs[i]->enableIRIn();
  }

#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif

  // led 활성화
  RL_pixels.begin();
  RR_pixels.begin();
  FL_pixels.begin();
  FR_pixels.begin();

  // 밝기 조절, 제임스는 하우징이 있어서 최대 밝기로 하는 것이 좋을 듯 그러나 디버깅 중에 우리 눈은 소중하니까~
  /* my eyes!!!!! ehhh..... */
//  RL_pixels.setBrightness(100);
//  RR_pixels.setBrightness(100);
//  FL_pixels.setBrightness(100);
//  FR_pixels.setBrightness(100);

}

void loop() {
  // Serial.read() 함수 을 해주는 이유는, serial read 와 관련이 있습니다!
  // Serial.available 를 해주게 되면, Serial 포트에서 읽을 수 있는 바이트 수를 반환합니다. 
  // 반환된 바이트 수만큼 Serial.read() 함수 등을 사용해 데이터를 읽어들일 수 있습니다.
  // 따라서 Serial.read() 함수를 사용할 때는 Serial.available 를 꼭 해주시길!!
    if (Serial.available() > 0) {
    // LED 관련 함수 
    Change_Value_in_Serial();
  }


  // docking ir 수신부 관련 코드
  // 가장 처음에는 나노에 도킹만 올라가져 있어서, 따로 함수화 하지 않았습니다. ㅠㅠ
  // 시간 되실때, loop 문의 간소화를 위해 함수화 부탁드립니다!! ㅎㅎ
  for (int i = 0 ; i < RECEIVERS ; i++) {
    // 아래의 if 문은 발신부에서 전송한 ir data 를 읽고
    // 해당 배열에 이진 표기로 저장하게 됩니다. 만약 data 가 들어오지 않는 다면, default 로 000 을 저장합니다. (switch case 문)
    // irremote 라이브러리에는 ir data 를 irSender.sendNEC 를 통해 보내기도 하지만,
    // irrecvs->decode 를 통해 받아 올 수 있습니다. 본 함수를 통해, IR data를 수신하여 변조된 데이터를 results 변수에 저장합니다.
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
      // resume 함수는 IR 데이터를 수신 한 후 다음 신호를 받을 수 있도록 준비 하는 함수 입니다.
      irrecvs[i]->resume();
    }
  }

  // ir 수신 data 를 opencr 에 전송할 때의 스케줄링해주며, rate 을 조절해주는 if 문입니다. 현재는 40 hz 로 전송하고 있습니다.
  if (millis() >= range_time) {
    int data = 0;
    for (int i = 0; i < 3; i++) {

      // 129 줄 부터 있는 if 문에서 L, C, R 에 해당하는 비트 값을 저장 해주는 데, 이를 합쳐서 0-7 까지의 3bit 로 합쳐주는 구간입니다.
      // 이렇게 합쳐진 3bit 의 데이터는 하나의 ir 수신부(총 3개의 수신부)가 발신부부터 받아오는 Data 를 한번에 표기하는 변수가 됩니다.
      // 000 : 아무것도 보이지 않음 
      // 100 : Left ir 을 수신하는 중
      // 010 : center ir 을 수신하는 중
      // 001 : Right ir 을 수신하는 중
      bit_T[i] = bit_L[i] + bit_C[i] + bit_R[i];
    }

    // bit_T[i] 가 하나의 수신부가 확인 하는 ir data 를 3bit 에 저장 했다고 하면, data 는 비트 연산을 이용하여 이를 int 로 변환한 값입니다.
    // data=16 이라면, 16 = B000010000 = 000 010 000 두번째 ir 수신부가 가운데 발신부 ir 데이터 값을 받고 있다.
    data =  bit_T[0] << 6 | bit_T[1] << 3 | bit_T[2] ;

    // 해서 다음과 같이 serial.print 로 opencr 에게 넘겨줍니다.
    Serial.println(data);

    range_time = millis() + 25;
  }
}


// opencr 에서 전송한 serial data로 led 를 제어하는 함수 입니다.
// opencr 에서는 topic 을 subscribe 하고, 이를 serial 를 통해 전송합니다. 
void Change_Value_in_Serial() {
  // opencr 에서 전송한 serial data 를 read 합니다.
  char c = Serial.read();

  if (c == '\n') {
    // opencr 이 전송 할때, serial.println 으로 전송하게 하였습니다. 
    // serial 에서는 줄바꿈을 \n 으로 인식하기 합니다. serial 전송이 string 이나 int 이더라도,
    // 수신되는 값은 char 형태이기 때문에,
    // \n 이 들어오기 전까지의 char data(현 코드에서는 180 줄에 선언되어 있는 char c) 를 string command 에 저장해주고,
    // 시용이 완료되면, 초기화 해줍니다. (command = "")

    String part1;
    part1 = command;
    // string data 를 int 로 형변환 해줍니다. :)
    led_val = part1.toInt();
    
    // openCr 은  4자리 수의 int 를 subscribe 합니다.
    // 4자리 수인 이유는 직관적이게도, 4개의 led 에게 case 를 부여하기 위함입니다.
    // 따라서, 하나의 led 는 0-9, 10가지의 동작 case 를 가질 수 있습니다.
    // intepret 함수에서는 네자리 수의 숫자를 하나씩 쪼개어서 배열에 넣어주는 역할을 합니다.
    intepret(led_val);
    
    // 초기화 :)
    command = "";
  }

  else {
    // \n 이 들어오지 않았다면, command sring 에 data 를 계속 추가
    command += c;
  }
}



void intepret(int _led_val) {
  char led_arr[4];
  // sprintf 는 int 갑을 char 의 형태로 led_arr 에 각각 저장해주는 것을 도와주는 함수 입니다.
  // 예를 들면, 1234 라는 data 가 _led_val 에 들어 왔으면,
  // led_arr[0] = 1,led_arr[1] = 2,led_arr[2] = 3,led_arr[3] = 4 처럼 각각의 배열에 자릿수대로 할당됩니다.
  sprintf(led_arr, "%d", _led_val);

  // 위에서 하나의 led 가 가지는 동작 case 가 10가지라고 하였습니다. 
  // execute 는 이 case 들을 동작시키는 함수입니다.
  // 4개의 led 객체를 변수처럼 활용하여 따로따로 제어할 수 있게 해두었습니다.
  // 따라서, 1234 라는 데이터가 들어오면, 4개의 led 는 서로 다른 동작을 구현하게 됩니다.
  execute(led_arr[0], &RL_pixels);
  execute(led_arr[1], &RR_pixels);
  execute(led_arr[2], &FL_pixels);
  execute(led_arr[3], &FR_pixels);
}

void execute(char _led_num , Adafruit_NeoPixel *pixels) {
  // char 를 int 로 형변환 할때는 아래와 같이 기술 하면 됩니다.
  int led_num_data = _led_num - '0';

  //동작을 실행하는 case 문입니다. 아직 구현한 case 가 몇가지 없습니다.
  // inteprte 함수에서 led 동작 case 를 알려주는 숫자를 알려줬기 때문에, 해당하는 번호에 있는 함수가 동작됩니다.
  // 예를 들면, 1234라는 data 가 있었을 때, 첫번째 led 는 1 번 case 로 이동하여 빨간색의 불빞을 키게 됩니다.
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
