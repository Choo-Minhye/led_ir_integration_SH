#include <IRremote.h>

IRsend irSender;    //Left Transmitter: pin 3
IRsend1 irSender1;  //Center Transmitter: pin 9  //수정 해야함.
IRsend2 irSender2;  //Right Transmitter: pin 10  //수정 해야함.

// library manager 에서 라이브러리를 다운 받으시지 마시고, 폴더 안에 있는 libraries_for_opencr.zip 파일에서 이용하시기 바랍니다.
// 공식 library 에는 우노에서 지원하는 핀은 3 pin 뿐이었으나, 저희가 3개의 발신 ir 을 사용하기 위하여,
// pwm 3, pwm 4 에 해당하는 디지털 핀(9, 10)을 추가적으로 열어주어서 header 가 다소 다릅니다!


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  irSender.sendNEC(0xFF629D, 32); 
  delay(25);
  irSender1.sendNEC(0xFFE21D, 32);
  delay(25);
  irSender2.sendNEC(0xFFA25D, 32);
  delay(25);
} 

// delay 가 들어가 있는 이유는, ir data 를 보낼 때, delay 가 없으면, 
// 송신 과정 중 data 가 섞이거나, 순차적으로 송신되지 않는 이슈가 있어서 들어가 있습니다.
// 수작업으로 맞춘 delay 라서 더 좋은 value 가 있다면, 수정하셔도 좋지만, 당시 테스트 할 때에는, 25 가 가장 적당하였습니다.




//*** ERROR **//
// IRremote 에 들어가면, sneder1 과 sender2 의 digital pin 이 반대로 설정되어 있어서,
// 현재는 9번과 10번 핀이 반대로 send 됩니다. ㅠㅠ 이점은 header 를 수정하면 되는데,
// nano_part.ino 에서도 반대로 받게 되어 있어서 이부분 관련해서는 수정하지 않으셔도 됩니다!
// 조금 여유 있을때, 두 코드 다 제가 수정해두겠습니다!!