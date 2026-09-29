#include <Servo.h>
#include <string.h>
#include <stdlib.h>

const uint8_t buttonPins[3]={10,11,12};
struct ButtonState
{
  uint8_t pin;
  bool lastReading; //上一次读数
  bool stableState; //去抖后的稳定状态
  int  lastChangeTime;
};

ButtonState buttons[3];

#define OPENCLAW  130
#define CLOSECLAW 180      // 原值 10 小于 MINCpos(25)，夹爪会一直堵转

const int MAXMpos = 180, MINMpos = 0;     // Middle
const int MAXRpos = 180, MINRpos = 30;    // Right
const int MAXLpos = 120, MINLpos = 20;    // Left
const int MAXCpos = 180, MINCpos = 90;    // Claw
const int DEBOUNCE_MS=30;

Servo Middle;
Servo Left;
Servo Right;
Servo Claw;

int Mpos = 100;
int Lpos = 90;
int Rpos = 50;
int Cpos = 150;
int speed = 30;               // 每 1 度的延时(ms)，越大越慢

char rxBuf[32];               // 接收缓存区
uint8_t rxLen = 0;            // 当前长度

void Action_A();
void Action_B();
void Action_C();
void turn(char name, int frompos, int topos);
void parseAndControl(char *cmd);


void setup() {
  Claw.attach(6);
  Right.attach(7);
  Left.attach(8);
  Middle.attach(9);

  Serial.begin(9600);

  delay(300);                 // 等 USB 串口稳定
  Serial.println("Serial ready");
}
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {          // 行结束符
      if (rxLen > 0) {                     // 收到一整行 -> 解析
        rxBuf[rxLen] = '\0';
        parseAndControl(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) { // 普通字符 -> 入缓存
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                            // 溢出，整条丢弃
    }
  }
}

void Action_A()
{
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
  turn('M',Mpos,30);
  Mpos=30;
  turn('R',Rpos,110);
  Rpos=110;
  turn('L',Lpos,25);
  Lpos=25;
  delay(400);
  turn('C',Cpos,CLOSECLAW);
  Cpos=CLOSECLAW;
  turn('M',Mpos,50);
  Mpos=50;
  turn('R',Rpos,100);
  Rpos=100;
  turn('L',Lpos,30);
  Lpos=30;
  delay(400);
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  delay(200);
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
}
void Action_B()
{
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,110);
  Rpos=110;
  turn('L',Lpos,25);
  Lpos=25;
  delay(400);
  turn('C',Cpos,CLOSECLAW);
  Cpos=CLOSECLAW;
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,120);
  Rpos=120;
  turn('L',Lpos,50);
  Lpos=50;
  delay(400);
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  delay(200);
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
}
void Action_C()
{
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
  turn('M',Mpos,145);
  Mpos=145;
  turn('R',Rpos,100);
  Rpos=100;
  turn('L',Lpos,20);
  Lpos=20;
  delay(400);
  turn('C',Cpos,CLOSECLAW);
  Cpos=CLOSECLAW;
  turn('M',Mpos,120);
  Mpos=120;
  turn('R',Rpos,100);
  Rpos=100;
  turn('L',Lpos,25);
  Lpos=25;
  delay(400);
  turn('C',Cpos,OPENCLAW);
  Cpos=OPENCLAW;
  delay(200);
  turn('M',Mpos,100);
  Mpos=100;
  turn('R',Rpos,50);
  Rpos=50;
  turn('L',Lpos,90);
  Lpos=90;
}
/* 从 frompos 平滑转到 topos，只在两端写一次，中间每度停 speed 毫秒 */
void turn(char name, int frompos, int topos) {
  if (frompos == topos) return;

  int step = (frompos < topos) ? 1 : -1;
  for (int i = frompos; ; i += step) {
    switch (name) {
      case 'M': Middle.write(i); break;
      case 'R': Right.write(i);  break;
      case 'L': Left.write(i);   break;
      case 'C': Claw.write(i);   break;
      default: return;
    }
    if (i == topos) break;
    delay(speed);
  }
}

void parseAndControl(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;   // 跳过行首空白
  if (*cmd == '\0') return;

  switch (cmd[0]) {
    case 'O':                                  // 张开夹爪
      turn('C', Cpos, OPENCLAW);
      Cpos = OPENCLAW;
      return;

    case 'S':                                  // 闭合夹爪
      turn('C', Cpos, CLOSECLAW);
      Cpos = CLOSECLAW;
      return;

    case 'H':                                  // 变快
      speed = constrain(speed - 2, 2, 100);
      Serial.print("Speed: "); Serial.println(speed);
      return;

    case 'L':                                  // 变慢
      speed = constrain(speed + 2, 2, 100);
      Serial.print("Speed: "); Serial.println(speed);
      return;
    
    case 'A':
      Action_A();
      return;
    
    case 'B':
      Action_B();
      return;
    case 'C':
      Action_C();
      return;
    
  }

  /* x -> Middle   y -> Right   z -> Left */
  char *px = strchr(cmd, 'x');
  char *py = strchr(cmd, 'y');
  char *pz = strchr(cmd, 'z');

  if (px && py && pz) {
    //读取数据
    int x = constrain(atoi(px + 1), MINMpos, MAXMpos);
    int y = constrain(atoi(py + 1), MINRpos, MAXRpos);
    int z = constrain(atoi(pz + 1), MINLpos, MAXLpos);
    //记录旧位置
    int oldM = Mpos, oldR = Rpos, oldL = Lpos;
    //更新当前位
    Mpos = x; Rpos = y; Lpos = z;        

    turn('M', oldM, x);
    turn('R', oldR, y);
    turn('L', oldL, z);
  }
}