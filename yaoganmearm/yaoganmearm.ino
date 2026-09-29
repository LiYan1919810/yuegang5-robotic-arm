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

#define OPENCLAW  120
#define CLOSECLAW 180      // 原值 10 小于 MINCpos(25)，夹爪会一直堵转

const int MAXMpos = 180, MINMpos = 0;     // Middle
const int MAXRpos = 180, MINRpos = 45;    // Right
const int MAXLpos = 120, MINLpos = 35;    // Left
const int MAXCpos = 180, MINCpos = 120;    // Claw
const int DEBOUNCE_MS=30;

Servo Middle;
Servo Left;
Servo Right;
Servo Claw;

int Mpos = 90;
int Lpos = 45;
int Rpos = 45;
int Cpos = 90;
int speed = 30;               // 每 1 度的延时(ms)，越大越慢
/*------------摇杆相关定义------------------*/
//摇杆一：X(A0)->Middle     Y(A1)->Left
//摇杆二：X(A2)->Right      Y(A3)->Claw
#define J1_X_PIN A0
#define J1_Y_PIN A1
#define J2_X_PIN A2
#define J2_Y_PIN A3
#define J_SW_PIN A4

const int JOY_CENTER=512; //ADC中点
const int JOY_DEADZONE=80;//死区（用于抑制抖动）
const int JOY_STEP=2;     //每个采样周期最大变化角度
const int JOY_INTERVAL=15;//采样周期(ms)

unsigned long joyLastRead=0;
unsigned long joySwChangeTime=0;
bool joySwLast=HIGH;
bool joySwStable=HIGH;
//用unsigned long使变量非负
/*-------------摇杆函数声明--------*/
void joystickControl();
void joystickButtonScan();
int move(int pin);
/*-------舵机动作函数声明----------------*/
void turn(char name, int frompos, int topos);
void setup() {
  // put your setup code here, to run once:
  Claw.attach(6);
  Right.attach(7);
  Left.attach(8);
  Middle.attach(9);
  Serial.begin(9600);
  for(int i=0;i<3;i++)
  {
    pinMode(buttonPins[i],INPUT_PULLUP);

    buttons[i].pin=buttonPins[i];
    buttons[i].lastReading=HIGH;
    buttons[i].stableState=HIGH;
    buttons[i].lastChangeTime=0;
  }
  pinMode(J_SW_PIN,INPUT_PULLUP);
  delay(300);                 // 等 USB 串口稳定
  Serial.println("Serial ready");
}
//非阻塞状态检测
bool updateButton(ButtonState &btn)
  {
    bool reading =digitalRead(btn.pin);
    bool pressedEvent=false;

    if(reading!=btn.lastReading)
    {
      btn.lastChangeTime=millis();
      btn.lastReading=reading;
    }

    if(millis()-btn.lastChangeTime>=DEBOUNCE_MS)
    {
      if(reading!=btn.stableState)
      {
        btn.stableState=reading;

        if(btn.stableState==LOW)
        {
          pressedEvent=true;
        }
      }
    }
    return pressedEvent;
  }
void loop() 
{
  joystickControl();
  joystickButtonScan();
  // put your main code here, to run repeatedly:
}
//舵机控制函数
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
/*-----动作组函数--------*/
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



//摇杆动作函数
int move(int pin)
{
  int v=analogRead(pin);
  int off=v-JOY_CENTER;    //偏离中心的值（相对位移）

  //死区内让摇杆判定居中，不动作
  if(off>-JOY_DEADZONE&&off<JOY_DEADZONE) return 0;

  //偏离值越大，走得越快，死区处1度/周期，推到顶2度/周期
  int mag=map(abs(off),JOY_DEADZONE,JOY_CENTER,1,JOY_STEP);
  mag=constrain(mag,1,JOY_STEP);
  //输出相对值
  return (off>0)?mag:-mag;
}

//摇杆控制舵机的函数
void joystickControl()
{
  static int lastM=-1,lastR=-1,lastL=-1,lastC=-1;
  //限速
  if(millis()-joyLastRead<JOY_INTERVAL) return;
  joyLastRead=millis();

  //constrain保护舵机在合适范围内活动
  Mpos=constrain(Mpos-move(J1_X_PIN),MINMpos,MAXMpos);
  Rpos=constrain(Rpos+move(J2_X_PIN),MINRpos,MAXRpos);
  Lpos=constrain(Lpos-move(J1_Y_PIN),MINLpos,MAXLpos);
  Cpos=constrain(Cpos-move(J2_Y_PIN),MINCpos,MAXCpos);

  if(Mpos!=lastM){Middle.write(Mpos);lastM=Mpos;}
  if(Rpos!=lastR){Right.write(Rpos);lastR=Rpos;}
  if(Lpos!=lastL){Left.write(Lpos);lastL=Lpos;}
  if(Cpos!=lastC){Claw.write(Cpos);lastC=Cpos;}
}

void joystickButtonScan()
{
  int count=0;
  bool reading =digitalRead(J_SW_PIN);

  if(reading!=joySwLast)
  {
    joySwChangeTime=millis();
    joySwLast=reading;
  }
  if(millis()-joySwChangeTime>=DEBOUNCE_MS)
  {
    if(reading!=joySwStable)
    {
      joySwStable=reading;
      if(joySwStable==LOW) 
      {
        Action_A();
      }
    }
  }
}