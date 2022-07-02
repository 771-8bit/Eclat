#include <TimerTCC0.h>
#include <IcsHardSerialClass.h>

#define DAC 0 //未使用
#define STICK_X 1
#define STICK_Y 2
#define EN_PIN  10
#define LED_YELLOW  LED_BUILTIN
#define LED_BLUE    PIN_LED_RXL

const float range_deg = 30.0; // -range_deg ~ +range_deg
const float phase_rad = 3.14 / 4;
//      y
//      |  pi/4
//      | /
//  ----|----x 0
//      |
//      |

const long BAUDRATE = 115200;  //一番遅いのがこれ
const int TIMEOUT = 50;    //通信できてないか確認用にわざと遅めに設定

IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT); //インスタンス＋ENピン(2番ピン)およびUARTの指定

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);
  //pinMode(DAC, INPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_YELLOW    , HIGH);
  digitalWrite(LED_BLUE    , HIGH);

  Serial.begin(115200);

  krs.begin();  //サーボモータの通信初期設定

  //TimerTcc0.initialize((1000000 / PWM_Hz) / 1000); //PWM1パルスを1000回計測
  //TimerTcc0.attachInterrupt(timerIsr);
}

void loop() {
  /*
    Serial.print(analogRead(STICK_X));
    Serial.print(",");
    Serial.print(analogRead(STICK_Y));
    Serial.print(",");*/
  
    Serial.print(stick_normalized());
    Serial.print(",");
    Serial.print(control_curve_degree(stick_normalized()));
    Serial.print(",");
    servo_write_degree(control_curve_degree(stick_normalized()));
  
  /*
  delay(1);
  for (int i = 5500; i < 9500; i+=5) {
    krs.setPos(0, i);     //位置指令　ID:0サーボを9500へ 右
    delay(1);
  }
  for (int i = 9500; i > 5500; i-=5) {
    krs.setPos(0, i);     //位置指令　ID:0サーボを9500へ 右
    delay(1);
  }*/

  

  /*
          //位置指令　ID:0サーボを7500へ 中央
    delay(500);              //0.5秒待つ
    krs.setPos(0,9500);      //位置指令　ID:0サーボを9500へ 右
    delay(500);              //0.5秒待つ
    krs.setPos(0,7500);      //位置指令　ID:0サーボを7500へ 中央
    delay(500);              //0.5秒待つ
    krs.setPos(0,5500);      //位置指令　ID:0サーボを5500へ 左
    delay(500);*/

}

void servo_write_degree(float degree) { //get degree -15.0 ~ +15.0, control servo
  int servo_val = 7500 + (2000 * degree / 135);
  if (abs(servo_val - 7500) < 2) {
    servo_val = 7500;
  }
  krs.setPos(0, servo_val );
  Serial.println(servo_val);
}

float control_curve_degree(float val_normalized) { //get -1 ~ +1 , return degree -15 ~ +15
  float degree = val_normalized * abs(val_normalized) * range_deg;
  if (degree >= range_deg) {
    degree = range_deg;
  }
  if (degree <= -range_deg) {
    degree = -range_deg;
  }
  return degree;
}

float stick_normalized() { //return -1 ~ +1
  uint32_t stick_x_raw10 = 0;
  uint32_t stick_y_raw10 = 0;
  for (int i = 0; i < 10; i++) {
    stick_x_raw10 += analogRead(STICK_X);
    stick_y_raw10 += analogRead(STICK_Y);
  }
  float stick_x = ((float)stick_x_raw10 / 10 - 2078.0) / 2048.0;
  float stick_y = ((float)stick_y_raw10 / 10 - 2057.0) / 2048.0;
  float len = sqrt(stick_x * stick_x + stick_y * stick_y);
  return (len * cos(atan2(stick_y, stick_x) - phase_rad));
}