#include <Servo.h>
#include <string.h>
#include <stdlib.h>

const uint8_t buttonPins[4]={10,11,12,13};
struct ButtonState
{
  uint8_t pin;
  bool lastReading; //上一次读数
  bool stableState; //去抖后的稳定状态
  int  lastChangeTime;
};

ButtonState buttons[4];

#define OPENCLAW  120
#define CLOSECLAW 180      // 原值 10 小于 MINCpos(25)，夹爪会一直堵转

const int MAXMpos = 180, MINMpos = 0;     // Middle
const int MAXRpos = 180, MINRpos = 45;    // Right
const int MAXLpos = 120, MINLpos = 20;    // Left
const int MAXCpos = 180, MINCpos = 120;    // Claw
const int DEBOUNCE_MS=30;

Servo Middle;
Servo Left;
Servo Right;
Servo Claw;
/*---------舵机控制相关变量1-------------*/
int Mpos = 90;
int Lpos = 45;
int Rpos = 45;
int Cpos = 135;
int speed = 30;               // 每 1 度的延时(ms)，越大越慢（注：当前未使用，实际节拍由 TURN_INTERVAL 控制）
int Action_Mode=2;
const int TURN_INTERVAL=30;   // 舵机每走 1 度之间的最小间隔(ms)

/*---------舵机控制相关变量2-------------*/
#define ACTION_MAX_QUEUE 16
struct ServoTarget{int M,R,L,C;};
ServoTarget actionQueue[ACTION_MAX_QUEUE];
uint8_t queueHead=0;     //下标
uint8_t queueCount=0;    //队列中待执行动作数
uint8_t actionPhase=1;   //当前动作进行到的待机序号
/*
1->M 2->R 3->L 4->C
*/
unsigned long lastStepTime=0;
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
/*------------按键变量--------*/
#define KEY1_MASK 0x01
#define KEY2_MASK 0x02
#define KEY3_MASK 0x04
#define KEY4_MASK 0x08

bool repeatMode =false;     //循环状态判断

uint8_t updateButton();

/*-------舵机动作函数声明----------------*/
bool turn(char name,int topos);
void Action(int toMpos,int toRpos,int toLpos,int toCpos);
void actionTick();
void clearActionQueue();
void Action_A();
void Action_B();
void Action_C();
void setup() {
  // put your setup code here, to run once:
  Claw.attach(6);
  Right.attach(7);
  Left.attach(8);
  Middle.attach(9);
  Serial.begin(9600);
  Action(100,50,90,OPENCLAW);
  for(int i=0;i<4;i++)
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
uint8_t updateButton()
{
  uint8_t pressMask=0;

  for(int i=0;i<4;i++)
  {
    bool reading=digitalRead(buttons[i].pin);
    //状态检测
    if(reading!=buttons[i].lastReading)
    {
      buttons[i].lastChangeTime=millis();
      buttons[i].lastReading=reading;
    }
    //消抖
    if(millis()-buttons[i].lastChangeTime>=DEBOUNCE_MS)
    {
      if(reading!=buttons[i].stableState)
      {
        buttons[i].stableState=reading;
        if(buttons[i].stableState==LOW)
        {
          pressMask|=(1<<i);
          //这里询问了ai才得出来的
          //记录第i个按键被按下
        }
      }
    }
  }
return pressMask;
}
void loop() 
{
  joystickControl();
  joystickButtonScan();
  uint8_t keyEvent=updateButton();
  //按下按键4，停止循环，并回归到初始状态
  if(keyEvent&KEY4_MASK)
  {
    repeatMode=false;
    clearActionQueue();             //清空未完成的动作
    Action(100,50,90,CLOSECLAW);
  }
  if(keyEvent&KEY1_MASK)
  {
    Action_Mode=(Action_Mode+1)%3;
    repeatMode=true;
    clearActionQueue();            //清空动作，立即执行下一个动作组
  }
  //循环模式
  if(repeatMode&&queueCount==0)
  {
    switch(Action_Mode)
    {
      case 0:Action_A();break;
      case 1:Action_B();break;
      case 2:Action_C();break;
      default:break;
    }
  }
  actionTick();   //每个主循环都推进一次动作，否则动作会走 1 度就卡住
  // put your main code here, to run repeatedly:
}
//舵机控制函数
bool turn(char name,int topos)
{
  int *pos=NULL;
  //获取位置信息
  switch(name)
  {
    case 'M':pos=&Mpos;break;
    case 'R':pos=&Rpos;break;
    case 'L':pos=&Lpos;break;
    case 'C':pos=&Cpos;break;
    default:return true;    //未知舵机：直接视为完成，防止卡死
  }

  if(*pos==topos) return true;//已到达终点
  if(millis()-lastStepTime<TURN_INTERVAL) return false;
    //未到时间先返回
    //终于想到了我哭死,终于不阻塞了QAQ
    lastStepTime=millis();
    *pos+=(*pos<topos)?1:-1;    //确定移动方向

    switch(name)
    {
      case 'M':Middle.write(*pos);break;
      case 'R':Right.write(*pos);break;
      case 'L':Left.write(*pos);break;
      case 'C':Claw.write(*pos);break; 
    }
    return (*pos==topos);
  
}
//动作函数
void Action(int toMpos,int toRpos,int toLpos,int toCpos)
{
  if(queueCount>=ACTION_MAX_QUEUE) return; //防止动作溢出;
  if(queueCount==0) actionPhase=1;         //新序列开始
  uint8_t idx=(queueHead+queueCount)%ACTION_MAX_QUEUE;
  actionQueue[idx].M=toMpos;
  actionQueue[idx].R=toRpos;
  actionQueue[idx].L=toLpos;
  actionQueue[idx].C=toCpos;
  queueCount++;
}
//清空队列函数
void clearActionQueue()
{
  queueHead=0;
  queueCount=0;
  actionPhase=1;
}
//用于一个节拍推进一步
void actionTick()
{
  if(queueCount==0) return; //没有可执行动作

  ServoTarget &t =actionQueue[queueHead];
  bool done=false;
  switch(actionPhase)
  {
    case 1:done=turn('M',t.M);break;
    case 2:done=turn('R',t.R);break;
    case 3:done=turn('L',t.L);break;
    case 4:done=turn('C',t.C);break;
    default: actionPhase=1;return;
  }
  //到位后切换到下一个舵机
  if(done)
  {
    actionPhase++;
    if(actionPhase>4)
    {
      actionPhase=1;
      queueHead=(queueHead+1)%ACTION_MAX_QUEUE;
      queueCount--;
    }       
  }
}
/*-----动作组函数--------*/
void Action_A()
{
  Action(100,50,90,OPENCLAW);
  Action(30,110,25,CLOSECLAW);
  Action(50,100,30,OPENCLAW);
  Action(100,50,90,CLOSECLAW);
}
void Action_B()
{
  Action(100,50,90,OPENCLAW);
  Action(100,110,25,CLOSECLAW);
  Action(100,120,50,OPENCLAW);
  Action(100,50,90,CLOSECLAW);
}
void Action_C()
{
  Action(100,50,90,OPENCLAW);
  Action(135,100,20,CLOSECLAW);
  Action(120,100,25,OPENCLAW);
  Action(100,50,90,CLOSECLAW);
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
  if(queueCount>0) return;   //动作执行期间暂停摇杆，避免与动作争抢同一舵机
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