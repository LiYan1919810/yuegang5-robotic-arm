#include <Servo.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
const uint8_t buttonPins[4]={10,11,12,13};
struct ButtonState
{
  uint8_t pin;
  bool lastReading; //上一次读数
  bool stableState; //去抖后的稳定状态
  bool pressed;     //本次扫描检测到的按下事件（在下一次扫描前有效）
  unsigned long lastChangeTime;
};
ButtonState buttons[4];

#define OPENCLAW  60
#define CLOSECLAW 130      

const int MAXMpos = 180, MINMpos = 0;     // Middle
const int MAXRpos = 180, MINRpos = 20;    // Right
const int MAXLpos = 130, MINLpos = 10;    // Left
const int MAXCpos = 130, MINCpos = 60;    // Claw
const int DEBOUNCE_MS=30;

Servo Middle;
Servo Left;
Servo Right;
Servo Claw;
/*---------舵机控制相关变量1-------------*/
int Mpos = 90;
int Lpos = 45;
int Rpos = 45;
int Cpos = 60;
int speed = 30;               // 每 1 度的延时(ms)，越大越慢（注：当前未使用，实际节拍由 TURN_INTERVAL 控制）
int Action_Mode=2;
const int TURN_INTERVAL=15;   // 舵机每走 1 度之间的最小间隔(ms)：越小越快（15=比原来快一倍）

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
/*-------逆运动学相关变量---------*/
const float ARM_L1=80.0;
const float ARM_L2=80.0;
const float HEIGHT=50.0;
const float CLAW_LEN=50.0;   
//关节角零点标定
const int MIDDLE_ZERO=100;
const int LEFT_ZERO=110;
const int RIGHT_ZERO=145;
//方向系数
const int MIDDLE_DIR=1;
const int RIGHT_DIR=-1;
const int LEFT_DIR=1;
//直线移动插值相关变量
static bool  lineActive = false;
static float lineStart[3];   // 起点
static float lineTarget[3];  // 终点
static float lineServoStart[3];   //M/R/L
static float lineServoStep[3];    // 每步各舵机增量（浮点，用于累计）
static float lineServoCount[3];   // 各舵机误差累积器：够 ±1 度才转
static int   lineServoSteps = 0;  // 总步数
static int   lineServoIndex = 0;  // 已推进步数
static unsigned long lineLastTime = 0;
static unsigned int  lineIntervalMs = 30;  // 当前直线每步间隔(ms)，moveLineTo 会重置为 LINE_INTERVAL_MS
#define LINE_INTERVAL_MS   30     // 普通直线每步间隔(ms)（动作组 / V 命令等）
#define TEACH_LINE_INTERVAL_MS 60 // 示教播放时每步间隔(ms)：越大越慢，便于观察
/*--------摩擦力/精度误差补偿（自适应 z 修正）--------*/
#define Z_COMP_MM   1.0f
#define Z_MIN_MM    1.0f    // 补偿后 z 的下限保护(mm)
#define Z_MAX_MM   250.0f   // 补偿后 z 的上限保护(mm)
//
/*--------录制功能相关--------*/
const int MAX_FRAMES=50;          //最大录制帧数（50帧×10mm≈50cm 行程）
const int REC_SAMPLE_MS=30;       //录制采样周期（ms）
const float REC_DIST_MM=10.0f;    //爪尖位移阈值（mm）
const int REC_CLAW_DEG=40;        //爪子角度阈值（度）
const int FRAME_INTERVAL=200;     //回放每帧间隔（ms）
uint8_t recordedAngles[MAX_FRAMES][4];  //记录舵机角度的二维数组
int frameCount=0;                   //当前记录帧数
int reproduceIndex=0;               //当前回放到第几帧
bool recordMode=false;
bool reproduceMode=false;           //在判断时需注意，只有当记录模式为fasle且再现模式为true的时候才会复现
bool paused=false;                  //播放任务中是否被暂停
unsigned long frameLastRead=0;
unsigned long reproduceLastRead=0;
//上一次的爪子位置以及爪子角度
float recLastX=0, recLastY=0, recLastZ=0;
int   recLastC=0;
bool  recHasLast=false;
/*--------示教功能相关--------*/
//串口 7 = 普通录制模式（现状），串口 8 = 示教模式；
//示教模式下：按键2 开始/结束示教，按键3 录制中打点、非录制中播放示教轨迹（逐点 moveLineTo）
#define TEACH_MAX_POINTS 5        //最多示教点数（5 点示教）
bool  teachMode=false;            //示教模式开关（串口 8 进入，串口 7 退出）
int   teachPointCount=0;          //已记录的示教点数
int16_t teachPoints[TEACH_MAX_POINTS][3];  //各示教点的爪尖坐标 x/y/z（按毫米取整）
int   teachPlayIndex=-1;          //示教播放进度：-1=未播放，-2=回初始位+等待(播放前置)，0..n-1=正在走向第 n 个点
#define TEACH_SETTLE_MS 1000      //示教播放：每到达一个点后，非阻塞等待舵机完全到位的时间(ms)
#define TEACH_HOME_WAIT_MS 2000   //示教播放：回到初始状态后、走向第1个点前的非阻塞等待(ms)
unsigned long teachSettleStart=0; //稳定等待计时起点
bool  teachSegArmed=false;        //已启动一段直线，走完后需要等待舵机稳定
/*-------动作组变量-------*/
int Line1=0;
unsigned long nowtime=0;
/*--------串口通讯------*/
char rxBuf[32];               // 接收缓存区
uint8_t rxLen = 0;            // 当前长度
/*-------串口通讯检测函数声明------*/
void parseAndControl(char *cmd);
/*-------------摇杆函数声明--------*/
void joystickControl();
void joystickButtonScan();
int move(int pin);
/*------------按键变量--------*/
bool repeatMode=false;     //循环状态判断
void updateButton();
/*--------录制播放函数声明--------*/
void Frame();
void Reproduce();
/*--------示教函数声明--------*/
void teachRecordPoint();
void teachPlayStart();
void teachTick();
/*-------舵机动作函数声明----------------*/
bool turn(char name,int topos);
void Action(int toMpos,int toRpos,int toLpos,int toCpos);
void actionTick();
void clearActionQueue();
void Action_A();
void Action_B();
void Action_C();
void Action_Line();
void Action_Line2();
void Action_V();
void Action_Linenext();
/*-----逆解运动学函数声明--------*/
bool solveIK(float x, float y, float z, float &baseDeg, float &shoulderDeg, float &elbowDeg);
bool computeJointAngles(float x, float y, float z, int &m, int &r, int &l, char &limitServo);
void parseSerialCommand();
void parseDirect(const String &s);
void driveDirect(float x, float y, float z);
/*-----正解运动学函数声明------*/
void solveFK(float &x,float &y,float &z);
void reportCurrentPosition();
bool driveIK(float x, float y, float z);
void moveLineTo(float x, float y, float z);
void moveLineTick();

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
    buttons[i].pressed=false;
    buttons[i].lastChangeTime=0;
  }
  pinMode(J_SW_PIN,INPUT_PULLUP);
  delay(300);                 // 等 USB 串口稳定
  Serial.println("Serial ready");
}
//非阻塞状态检测
void updateButton()
{
  for(int i=0;i<4;i++)
  {
    bool reading=digitalRead(buttons[i].pin);
    buttons[i].pressed=false;   //默认清空本次按下事件

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
          buttons[i].pressed=true;   //记录第i个按键被按下
        }
      }
    }
  }
}
void loop() 
{
  joystickControl();
  joystickButtonScan();
  updateButton();
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
  //按下按键4，停止循环，并回归到初始状态
  if(buttons[3].pressed)
  {
    repeatMode=false;
    clearActionQueue();             //清空未完成的动作
    lineActive=false;               //停止在途的直线插值
    teachPlayIndex=-1;              //停止示教播放
    teachSettleStart=0;             //清除示教稳定等待
    teachSegArmed=false;
    recordMode=false;
    reproduceMode=false;
    paused=false;
    Action(100,50,90,CLOSECLAW);
    Serial.println("[按键4] 初始化完成，已回归初始状态");
  }
  if(buttons[0].pressed)
  {
    Action_Mode=(Action_Mode+1)%3;
    repeatMode=true;
    clearActionQueue();            //清空动作，立即执行下一个动作组
  }
  //按键2：普通模式=录制/暂停；示教模式=开始/结束示教、播放中=暂停/继续
  if(buttons[1].pressed)
  {
    if(teachMode)
    {
      if(teachPlayIndex!=-1)   //播放中（含回初始位+等待阶段 -2）
      {
        if(!paused)
        {
          paused=true;
          Serial.println(F("[示教] 播放已暂停（按键2 继续）"));
        }
        else
        {
          paused=false;
          lineLastTime=millis();   //恢复时重置计时，避免立即推进一大步
          Serial.println(F("[示教] 播放已继续"));
        }
      }
      else if(recordMode)
      {
        recordMode=false;         //结束示教
        Serial.print(F("[示教] 示教结束，共 "));
        Serial.print(teachPointCount);
        Serial.println(F(" 个点"));
      }
      else
      {
        teachPointCount=0;        //开始示教前清空旧点
        reproduceMode=false;      //示教与帧播放互斥
        paused=false;
        recordMode=true;
        Serial.println(F("[示教] 示教开始：用摇杆摆好姿态后，按按键3 记录当前点"));
      }
    }
    else if(reproduceMode)   //播放中 -> 暂停/继续切换
    {
      if(!paused)
      {
        paused=true;
        Serial.println("[按键2] 播放已暂停");
      }
      else
      {
        paused=false;
        reproduceLastRead=millis();   //恢复时重置计时，避免立即推进一帧
        Serial.println("[按键2] 播放已继续");
      }
    }
    else                //非播放 -> 录制
    {
      if(recordMode)
      {
        recordMode=false;         //停止录制
        Serial.print("[按键2] 录制结束，共录制 ");
        Serial.print(frameCount);
        Serial.println(" 帧");
      }
      else
      {
        frameCount=0;             //开始录制前清空旧数据
        recHasLast=false;         //重置判据基准点（首帧必录）
        frameLastRead=millis();   //重置采样计时
        reproduceMode=false;      //录制与播放互斥
        recordMode=true;
        Serial.println("[按键2] 录制开始");
      }
    }
  }
  //按键3：普通模式=播放；示教模式=录制中打点 / 非录制中播放示教轨迹
  if(buttons[2].pressed)
  {
    if(teachMode)
    {
      if(recordMode)
      {
        teachRecordPoint();       //示教：记录当前点
      }
      else if(teachPlayIndex!=-1)   //播放中（含回初始位+等待阶段）-> 手动停止
      {
        teachPlayIndex=-1;        //手动停止示教播放
        lineActive=false;
        paused=false;             //清除暂停状态
        teachSettleStart=0;       //清除稳定等待状态
        teachSegArmed=false;
        clearActionQueue();
        Serial.println(F("[示教] 播放已停止（手动）"));
      }
      else if(teachPointCount>0)
      {
        teachPlayStart();
      }
      else
      {
        Serial.println(F("[示教] 暂无示教点，忽略播放"));
      }
    }
    else if(reproduceMode)
    {
      reproduceMode=false;      //手动停止播放
      Serial.println("[按键3] 播放结束（手动停止）");
    }
    else if(frameCount>0)      //没有录制数据则忽略
    {
      recordMode=false;         //播放与录制互斥
      reproduceIndex=0;         //从头开始播放
      paused=false;             //启动播放时清除暂停状态
      reproduceMode=true;
      Serial.print("[按键3] 播放开始，共 ");
      Serial.print(frameCount);
      Serial.println(" 帧");
    }
    else
    {
      Serial.println("[按键3] 无录制数据，忽略播放");
    }
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
  if(recordMode && !teachMode)   //示教模式只负责打点，不跑帧采样
  {
    Frame();
  }
  if(reproduceMode)
  {
    Reproduce();
  }
  actionTick();   //每个主循环都推进一次动作，否则动作会走 1 度就卡住
  moveLineTick();
  teachTick();    //示教播放：逐点用 moveLineTo 走直线
  Action_Linenext();
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
  Action(50,50,45,OPENCLAW);
  Action(50,100,25,CLOSECLAW);
  Action(50,50,45,CLOSECLAW);
  Action(80,100,30,OPENCLAW);
  Action(80,50,90,CLOSECLAW);
  Action(100,50,90,CLOSECLAW);
}
void Action_B()
{
  Action(100,50,90,OPENCLAW);
  Action(100,50,20,OPENCLAW);
  Action(100,100,20,CLOSECLAW);
  Action(100,100,50,CLOSECLAW);
  Action(100,50,90,CLOSECLAW);
  Action(100,50,50,CLOSECLAW);
  Action(100,110,50,OPENCLAW);
  Action(100,50,90,CLOSECLAW);
}
void Action_C()
{
  Action(100,50,90,OPENCLAW);
  Action(100,50,25,OPENCLAW);
  Action(135,110,20,CLOSECLAW);
  Action(135,50,90,CLOSECLAW);
  Action(120,100,25,OPENCLAW);
  Action(120,50,25,CLOSECLAW);
  Action(100,50,90,CLOSECLAW);
}
void Action_Line()
{
  moveLineTo(100,0,50);
  nowtime=millis();
  Line1=1;
}
void Action_Line2()
{
  moveLineTo(120,50,50);
}
void Action_V()
{
  moveLineTo(150,40,50);
  nowtime=millis();
  Line1=2;
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
  //动作队列执行期间、或直线插值期间暂停摇杆，避免与它们争抢同一批舵机
  if(queueCount>0 || lineActive) return;
  static int lastM=-1,lastR=-1,lastL=-1,lastC=-1;
  //限速
  if(millis()-joyLastRead<JOY_INTERVAL) return;
  joyLastRead=millis();

  //constrain保护舵机在合适范围内活动
  Mpos=constrain(Mpos-move(J1_X_PIN),MINMpos,MAXMpos);
  Rpos=constrain(Rpos+move(J2_X_PIN),MINRpos,MAXRpos);
  Lpos=constrain(Lpos-move(J1_Y_PIN),MINLpos,MAXLpos);
  //示教模式（录制/播放中）爪子保持闭合夹持笔，忽略 J2_Y 摇杆输入
  if(teachMode && (recordMode || teachPlayIndex!=-1))
    Cpos=CLOSECLAW;
  else
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
//每 REC_SAMPLE_MS（30ms） 采样一次，正解当前爪尖与"上一条记录点"比距离，
//位移>REC_DIST_MM (10mm)或 爪子角度变化>REC_CLAW_DEG (40)才记一帧（爪尖没动不浪费帧，爪子开合不丢）
void Frame()
{
  if(millis()-frameLastRead<=REC_SAMPLE_MS) return;
  frameLastRead=millis();

  if(frameCount>=MAX_FRAMES) return;   //防止数组越界

  int cm=Middle.read();
  int cr=Right.read();
  int cl=Left.read();
  int cc=Claw.read();

  //判断是否需要记录一帧
  bool needRec=false;
  if(!recHasLast)
  {
    needRec=true;                      //首帧无条件记录
  }
  else
  {
    //正解当前爪尖坐标（solveFK 内部使用 Mpos/Rpos/Lpos）
    float x,y,z;
    solveFK(x,y,z);
    float dx=x-recLastX, dy=y-recLastY, dz=z-recLastZ;
    float d=sqrt(dx*dx+dy*dy+dz*dz);
    if(d>REC_DIST_MM) needRec=true;            //爪尖移动超过阈值
    if(abs(cc-recLastC)>REC_CLAW_DEG) needRec=true;  //爪子开合超过阈值
  }
  //不需要记录就退出
  if(!needRec) return;

  //记入一帧
  recordedAngles[frameCount][0]=cm;
  recordedAngles[frameCount][1]=cr;
  recordedAngles[frameCount][2]=cl;
  recordedAngles[frameCount][3]=cc;
  frameCount++;

  //刷新"上一条记录点"
  solveFK(recLastX,recLastY,recLastZ);
  recLastC=cc;
  recHasLast=true;
}
void Reproduce()
{
  if(paused) return;   //暂停时不推进回放
  if(millis()-reproduceLastRead<=FRAME_INTERVAL) return;
  reproduceLastRead=millis();

  //回放结束：全部帧已送出且队列执行完毕
  if(reproduceIndex>=frameCount)
  {
    if(queueCount==0)
    {
      reproduceMode=false;   //播放一遍后自动停止
      Serial.println("[按键3] 播放结束（已播放一遍）");
    }
    return;
  }

  //逐帧入队，一次只送入一帧，避免超出队列容量导致丢帧
  if(queueCount<ACTION_MAX_QUEUE)
  {
    Action(recordedAngles[reproduceIndex][0],
           recordedAngles[reproduceIndex][1],
           recordedAngles[reproduceIndex][2],
           recordedAngles[reproduceIndex][3]);
    reproduceIndex++;
  }
}

/*-------示教功能----------*/
//示教：记录当前爪尖坐标（正解）与爪子角度
void teachRecordPoint()
{
  if(teachPointCount>=TEACH_MAX_POINTS)
  {
    Serial.print(F("[示教] 已达 "));
    Serial.print(TEACH_MAX_POINTS);
    Serial.println(F(" 个点上限，忽略本次记录"));
    return;
  }

  float x,y,z;
  solveFK(x,y,z);                 //正解当前爪尖坐标（基于 Mpos/Rpos/Lpos）

  //z 锁定为第一个点的 z：所有示教点保持同一水平面，便于用笔在纸面划线
  int16_t zUse=(teachPointCount==0) ? (int16_t)lround(z)
                                    : teachPoints[0][2];

  teachPoints[teachPointCount][0]=lround(x);   //按毫米取整后存入（int16_t）
  teachPoints[teachPointCount][1]=lround(y);
  teachPoints[teachPointCount][2]=zUse;

  Serial.print(F("[示教] 记录第 "));
  Serial.print(teachPointCount+1);
  Serial.print(F(" 点：("));
  Serial.print((int)teachPoints[teachPointCount][0]); Serial.print(F(","));
  Serial.print((int)teachPoints[teachPointCount][1]); Serial.print(F(","));
  Serial.print((int)zUse);
  Serial.println(F(")  爪子=闭合(CLOSE)"));

  teachPointCount++;
}

//示教：开始播放
void teachPlayStart()
{
  Line1=0;                          //取消在途的 Line 串联，避免与示教播放抢直线
  paused=false;                     //启动播放时清除暂停状态
  teachSettleStart=0;               //清除稳定等待状态
  teachSegArmed=false;
  //播放前置：先回到初始状态（夹爪闭合），走动作队列（非阻塞），
  //由 teachTick 等到队列走完后再非阻塞等待 TEACH_HOME_WAIT_MS，然后才走向第1个点
  clearActionQueue();
  Action(100,50,90,CLOSECLAW);
  teachPlayIndex=-2;                //-2=回初始位+等待阶段
  Serial.print(F("[示教] 播放开始，共 "));
  Serial.print(teachPointCount);
  Serial.println(F(" 个点（先回初始位，等待 2s 后逐点走直线）"));
}

//示教：播放推进——上一段直线走完、且舵机稳定 TEACH_SETTLE_MS 后，才启动下一段
void teachTick()
{
  if(teachPlayIndex<0 && teachPlayIndex!=-2) return;   //未在播放（-1=未播放）
  if(paused) return;                                    //暂停时不启动下一段

  //播放前置阶段（-2）：先等回初始位的动作队列走完，再非阻塞等待 TEACH_HOME_WAIT_MS，然后开始走向第1点
  if(teachPlayIndex==-2)
  {
    if(lineActive) return;                 //回初始位期间若混入直线，先等它结束
    if(queueCount>0){ teachSettleStart=0; return; }   //初始位动作组还在走
    if(teachSettleStart==0) teachSettleStart=millis();//初始位到位那一刻开始计时
    if(millis()-teachSettleStart<TEACH_HOME_WAIT_MS) return;
    teachSettleStart=0;
    teachPlayIndex=0;                      //等待结束，正式走向第1个点
    return;
  }

  if(lineActive) return;            //等上一段 moveLineTo 走完

  if(teachPlayIndex>=teachPointCount)
  {
    teachPlayIndex=-1;
    teachSettleStart=0;
    teachSegArmed=false;
    Serial.println(F("[示教] 播放结束（已走完全部示教点）"));
    return;
  }

  //非阻塞等待 
  if(teachSegArmed)
  {
    if(teachSettleStart==0) teachSettleStart=millis();          //本段走完的那一刻开始计时
    if(millis()-teachSettleStart<TEACH_SETTLE_MS) return;       
    teachSettleStart=0;
    teachSegArmed=false;
  }

  int i=teachPlayIndex;

  Line1=0;                          //确保没有 Line 串联在抢直线

  //爪子保持闭合（夹持笔），直接写不走动作队列，避免与 moveLineTo 争抢
  Cpos=CLOSECLAW;
  Claw.write(Cpos);

  Serial.print(F("[示教] 走向第 "));
  Serial.print(i+1);
  Serial.println(F(" 点"));
  //所有点与第1个点同高：z 统一取 teachPoints[0][2]
  moveLineTo(teachPoints[i][0],teachPoints[i][1],teachPoints[0][2]);
  lineIntervalMs=TEACH_LINE_INTERVAL_MS;   //示教播放放慢：每步间隔更大（moveLineTo 刚重置过，这里再覆盖）
  teachPlayIndex++;
  teachSegArmed=true;               //标记本段已启动：走完后需先等待舵机稳定，再走下一段
}
/*
逆解方程（定义：φ 从 +y 轴量起；θ2 为小臂与水平负方向的夹角，90°<=θ2<180°；
串口坐标 z 为离地高度，进入方程前换算为相对肩膀的高度 zArm=z-HEIGHT）
r=sqrt(x*x + y*y)                      
φ=atan2(x, y)                          
R=r-CLAW_LEN                        
D=sqrt(R*R+zArm*zArm)                
β=atan2(zArm,R)                       
α=acos[(L1*L1+D*D-L2*L2)/(2*L1*D)] 
θ1=α+β                               
θ2=θ1+acos[(L1*L1+L2*L2-D*D)/(2*L1*L2)]  
*/
//通过计算得出关节角
bool solveIK(float x, float y, float z, float &baseDeg, float &shoulderDeg, float &elbowDeg)
{
  float zArm=z-HEIGHT;   // 方程中的 z 是相对肩膀的高度，串口传入的是离地高度
  //底座轴到爪尖的水平距离
  float r=sqrt(x*x+y*y);
  //底座回转角：从 +y 轴量起，所以是 atan2(x,y)
  baseDeg=atan2(x,y)*180.0f/PI;
  //腕关节水平距离：机械爪恒水平，沿径向直接减去爪长
  float R=r-CLAW_LEN;
  if(R<0.0f) return false;   // 目标太近，腕关节无法退到该半径
  //肩关节到腕关节的直线距离
  float D=sqrt(R*R+zArm*zArm);
  //可达性检查
  if(D>ARM_L1+ARM_L2+0.01f) return false;
  if(D<1.0f) return false;
  //D与水平的夹角
  float beta=atan2(zArm,R);
  //L1与D 的夹角（余弦定理）
  float cosAlpha=(ARM_L1*ARM_L1+D*D-ARM_L2*ARM_L2)/(2.0f*ARM_L1*D);
  if(cosAlpha>1.0f) cosAlpha=1.0f;
  if(cosAlpha<-1.0f) cosAlpha=-1.0f;
  float alpha=acos(cosAlpha);
  //大臂角（肘在上分支）
  float theta1=alpha+beta;
  //小臂角θ2=θ1+肘部内角（余弦定理）
  float cosGamma=(ARM_L1*ARM_L1+ARM_L2*ARM_L2-D*D)/(2.0f*ARM_L1*ARM_L2);
  if(cosGamma>1.0f) cosGamma=1.0f;
  if(cosGamma<-1.0f) cosGamma=-1.0f;
  float theta2=theta1+acos(cosGamma);
  //大臂小臂角度
  shoulderDeg=theta1*180.0f/PI;
  elbowDeg=theta2*180.0f/PI;

  return true;
}



// 统一计算目标点对应的各舵机PWM值,并校验是否在行程内
// limitServo 标记是哪个舵机越界（'M'/'R'/'L'），为0表示纯几何不可达
bool computeJointAngles(float x, float y, float z, int &m, int &r, int &l, char &limitServo)
{
  limitServo=0;
  float baseDeg,shoulderDeg,elbowDeg;
  if(!solveIK(x,y,z,baseDeg,shoulderDeg,elbowDeg)) return false;
//各舵机角
  float mRaw=MIDDLE_ZERO+MIDDLE_DIR*(90.0f-baseDeg);
  float rRaw=RIGHT_ZERO+RIGHT_DIR*shoulderDeg;
  float lRaw=LEFT_ZERO+LEFT_DIR*(elbowDeg-180.0f);

  if(mRaw<MINMpos||mRaw>MAXMpos){limitServo='M';return false;}
  if(rRaw<MINRpos||rRaw>MAXRpos){limitServo='R';return false;}
  if(lRaw<MINLpos||lRaw>MAXLpos){limitServo='L';return false;}

  m=(int)lround(mRaw);
  r=(int)lround(rRaw);
  l=(int)lround(lRaw);
  return true;
}

bool driveIK(float x,float y,float z)
{
  int m,r,l;
  char limitServo;
  if(!computeJointAngles(x,y,z,m,r,l,limitServo)) return false;

  Action(m,r,l,Cpos);
  return true;
}
void parseAndControl(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;   // 跳过行首空白
  if (*cmd == '\0') return;

  switch (cmd[0]) {
    case 'S':
    Action(Mpos,Rpos,Lpos,CLOSECLAW);
    return;
    case 'O':
    Action(Mpos,Rpos,Lpos,OPENCLAW);  
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
    case '3':
      Action_Line();
      return;
    case '2':
      Action_Line2();
      return;
    case 'V':
      Action_V();
      return;
    case '7':
      //切回普通录制模式（原有的帧录制/帧播放）
      teachMode=false;
      teachPlayIndex=-1;
      Serial.println(F("[模式] 普通录制模式（按键2 录制 / 按键3 播放）"));
      return;
    case '8':
      //进入示教模式
      teachMode=true;
      reproduceMode=false;      //示教模式不与帧播放混用
      paused=false;
      teachPlayIndex=-1;
      Serial.println(F("[模式] 示教模式（按键2 开始/结束示教，按键3 打点或播放）"));
      return;
    case '0':
      // 查询当前机械臂坐标（正解，基于当前舵机值 Mpos/Rpos/Lpos）
      reportCurrentPosition();
      return;
    case 'G': {
      // 直接逆解到坐标（不走直线插值），用于验证逆解角度是否正确
      const char *p = cmd + 1;              // 跳过 'G'
      while (*p == ' ' || *p == '\t') p++;
      parseDirect(String(p));
      return;
    }
  }

  // 串口直接设置三舵机角度：形如 x90,y50,z45（x->Middle  y->Right  z->Left，值为舵机原始角度）
  if (cmd[0]=='x' || cmd[0]=='y' || cmd[0]=='z') {
    char *px = strchr(cmd, 'x');
    char *py = strchr(cmd, 'y');
    char *pz = strchr(cmd, 'z');
    // 三个字母都要有，且紧跟数字或负号（防止 "xyz" 被 atoi 解析成 0，把舵机顶到极限位）
    if (px && py && pz &&
        ((px[1]>='0'&&px[1]<='9') || px[1]=='-') &&
        ((py[1]>='0'&&py[1]<='9') || py[1]=='-') &&
        ((pz[1]>='0'&&pz[1]<='9') || pz[1]=='-')) {
      int tx = constrain(atoi(px + 1), MINMpos, MAXMpos);
      int ty = constrain(atoi(py + 1), MINRpos, MAXRpos);
      int tz = constrain(atoi(pz + 1), MINLpos, MAXLpos);
      Action(tx, ty, tz, Cpos);
      Serial.print(F("[舵机] M=")); Serial.print(tx);
      Serial.print(F(" R="));       Serial.print(ty);
      Serial.print(F(" L="));       Serial.println(tz);
    } else {
      Serial.println(F("[舵机] 格式应为 x90,y50,z45"));
    }
    return;
  }

  Serial.println(F("未知命令"));
}

// 解析 "x,y,z" 并直接逆解驱动（配合 'G' 命令，还是测试用的，可以无需理会）
void parseDirect(const String &s)
{
  int c1 = s.indexOf(',');
  if(c1 <= 0) return;
  int c2 = s.indexOf(',', c1 + 1);
  if(c2 <= 0) return;

  float x = s.substring(0, c1).toFloat();
  float y = s.substring(c1 + 1, c2).toFloat();
  float z = s.substring(c2 + 1).toFloat();

  driveDirect(x,y,z);
}

// 直接逆解到目标坐标（不做直线插值），串口输出关节角与舵机值，并直接驱动舵机
// 用于验证逆解结果是否能让机械臂到达指定位置
void driveDirect(float x, float y, float z)
{
  float baseDeg,shoulderDeg,elbowDeg;
  if(!solveIK(x,y,z,baseDeg,shoulderDeg,elbowDeg))
  {
    Serial.print("[直接] 目标(");
    Serial.print(x,0); Serial.print(",");
    Serial.print(y,0); Serial.print(",");
    Serial.print(z,0);
    Serial.println(") 逆解无解（超出工作空间）");
    return;
  }

  //新角度定义 -> 舵机值（换算关系同 computeJointAngles）
  float mRaw=MIDDLE_ZERO+MIDDLE_DIR*(90.0f-baseDeg);
  float rRaw=RIGHT_ZERO+RIGHT_DIR*shoulderDeg;
  float lRaw=LEFT_ZERO+LEFT_DIR*(elbowDeg-180.0f);

  Serial.print("[直接] 目标(");
  Serial.print(x,0); Serial.print(",");
  Serial.print(y,0); Serial.print(",");
  Serial.print(z,0);
  Serial.print(")  base="); Serial.print(baseDeg,1);
  Serial.print(" shoulder="); Serial.print(shoulderDeg,1);
  Serial.print(" elbow="); Serial.print(elbowDeg,1);
  Serial.print("  =>  M="); Serial.print((int)lround(mRaw));
  Serial.print(" R="); Serial.print((int)lround(rRaw));
  Serial.print(" L="); Serial.print((int)lround(lRaw));
  Serial.print(" C="); Serial.println(Cpos);

  if(mRaw<MINMpos||mRaw>MAXMpos||
     rRaw<MINRpos||rRaw>MAXRpos||
     lRaw<MINLpos||lRaw>MAXLpos)
  {
    Serial.println("[直接] 警告：舵机角度超出行程，未驱动");
    return;
  }

  Action((int)lround(mRaw),(int)lround(rRaw),(int)lround(lRaw),Cpos);
}

//正解得出x,y,z的方程
/*
正解方程（定义：φ 从 +y 轴量起；θ2 为小臂与水平负方向的夹角；
zArm 为相对肩膀的高度，输出时换算回离地高度 z=zArm+HEIGHT）
z=L1*sinθ1-L2*sinθ2
r=L1*cosθ1-L2*cosθ2 + CLAW_LEN
x=r*sinφ
y=r*cosφ
*/
void solveFK(float &x,float &y,float &z)
{
  //舵机值 -> 关节角（新定义，与 computeJointAngles 中的舵机映射互逆）
  float phi=(90.0f-(Mpos-MIDDLE_ZERO)*MIDDLE_DIR)*PI/180.0f;    
  float th1=(Rpos-RIGHT_ZERO)*RIGHT_DIR*PI/180.0f;              
  float th2=(180.0f+(Lpos-LEFT_ZERO)*LEFT_DIR)*PI/180.0f;      

  float zArm=ARM_L1*sin(th1)-ARM_L2*sin(th2);
  // 腕关节径向距离 + 恒水平爪长 = 底座轴到爪尖的水平距离
  float r=ARM_L1*cos(th1)-ARM_L2*cos(th2)+CLAW_LEN;

  x=r*sin(phi);
  y=r*cos(phi);
  z=zArm+HEIGHT;
}

//串口查询：输出当前爪尖坐标（正解）与舵机值，z 为离地高度(mm)
void reportCurrentPosition()
{
  float x,y,z;
  solveFK(x,y,z);
  Serial.print("[坐标] x="); Serial.print(x,1);
  Serial.print(" y=");      Serial.print(y,1);
  Serial.print(" z=");      Serial.print(z,1);
  Serial.print("  (M=");    Serial.print(Mpos);
  Serial.print(" R=");      Serial.print(Rpos);
  Serial.print(" L=");      Serial.print(Lpos);
  Serial.print(" C=");      Serial.print(Cpos);
  Serial.println(")");
}
//启动一次直线移动（XY 平面直线：起点、终点各逆解一次，中间对 M/R/L 舵机角线性插值）
void moveLineTo(float x,float y,float z)
{
  lineIntervalMs=LINE_INTERVAL_MS;   //默认速度（示教播放会在调用后覆盖为更慢的值）
  //停止其他任务
  repeatMode=false;
  recordMode=false;
  reproduceMode=false;
  paused=false;
  clearActionQueue();

  //起点坐标先正解一次：既用于打印，也用于本次的摩擦力/误差补偿比较
  solveFK(lineStart[0],lineStart[1],lineStart[2]);

  //=== 摩擦力/精度误差补偿：按离中心距离变化自适应修正目标 z ===
  // 由近到远：z -= Z_COMP_MM（现场表现为"越远越高"，故压低）；由远到近：z += Z_COMP_MM
  float zComp=0.0f;
  float rStart=sqrt(lineStart[0]*lineStart[0]+lineStart[1]*lineStart[1]); //当前点径向距离
  float rTarg =sqrt(x*x+y*y);                                            //目标点径向距离
  if(rTarg>rStart+0.5f)       zComp=-Z_COMP_MM;   //近 -> 远：高度减 1
  else if(rTarg<rStart-0.5f)  zComp= Z_COMP_MM;   //远 -> 近：高度加 1
  z+=zComp;
  if(z<Z_MIN_MM) z=Z_MIN_MM;
  if(z>Z_MAX_MM) z=Z_MAX_MM;

  //终点逆解（含关节角行程检查），不可达则直接报错取消
  int tm,tr,tl; char tLimit;
  if(!computeJointAngles(x,y,z,tm,tr,tl,tLimit))
  {
    Serial.print("[直线] 目标点(");
    Serial.print(x,0); Serial.print(",");
    Serial.print(y,0); Serial.print(",");
    Serial.print(z,0); Serial.print(") 不可达：");
    if(tLimit){ Serial.print("舵机"); Serial.print(tLimit); Serial.println(" 超出行程"); }
    else      { Serial.println("超出机械臂工作空间"); }
    return;
  }

  //起点舵机值：直接取当前 Mpos/Rpos/Lpos（不再用 solveFK 反推坐标）
  lineServoStart[0]=Mpos;
  lineServoStart[1]=Rpos;
  lineServoStart[2]=Lpos;

  lineTarget[0]=x;
  lineTarget[1]=y;
  lineTarget[2]=z;   //注意：这里的 z 已是补偿后的实际目标高度

  float dx=lineTarget[0]-lineStart[0];
  float dy=lineTarget[1]-lineStart[1];
  float dz=lineTarget[2]-lineStart[2];
  float dist=sqrt(dx*dx+dy*dy+dz*dz);

  //各舵机从起点到终点的总增量
  float dM=tm-lineServoStart[0];
  float dR=tr-lineServoStart[1];
  float dL=tl-lineServoStart[2];

  //步数 = 变化最大那个舵机的增量（取整，至少 1 步）
  float maxD=fabs(dM);
  if(fabs(dR)>maxD) maxD=fabs(dR);
  if(fabs(dL)>maxD) maxD=fabs(dL);
  lineServoSteps=(int)lround(maxD);
  if(lineServoSteps<1) lineServoSteps=1;

  //每步各舵机平均增量
  lineServoStep[0]=dM/lineServoSteps;
  lineServoStep[1]=dR/lineServoSteps;
  lineServoStep[2]=dL/lineServoSteps;

  //误差累积器清零
  lineServoCount[0]=0;
  lineServoCount[1]=0;
  lineServoCount[2]=0;
  lineServoIndex=0;
  lineLastTime=millis();
  lineActive=true;//更新状态

//写串口通讯方便观察和调试（可以忽略）
  Serial.print("[直线] 从(");
  Serial.print(lineStart[0], 0); Serial.print(",");
  Serial.print(lineStart[1], 0); Serial.print(",");
  Serial.print(lineStart[2], 0); Serial.print(") 到(");
  Serial.print(x, 0); Serial.print(",");
  Serial.print(y, 0); Serial.print(",");
  Serial.print(z, 0); Serial.print(")  距离=");
  Serial.print(dist, 0);
  Serial.print("mm  步数=");
  Serial.print(lineServoSteps);
  if(zComp!=0.0f)
  {
    Serial.print("  [补偿 z");
    Serial.print(zComp>0?'+':'-');
    Serial.print(fabs(zComp),0);
    Serial.print(": r ");
    Serial.print(rStart,0);
    Serial.print("->");
    Serial.print(rTarg,0);
    Serial.print("]");
  }
  Serial.println();
}
//每 15ms 推进一次：误差累积法——每个舵机累加每步增量，够 ±1 度才转一度，使转动平滑
void moveLineTick()
{
  if(!lineActive) return;
  if(paused && teachPlayIndex>=0) return;   //示教播放暂停时冻结当前段（不影响其他直线）

  //限速
  if(millis()-lineLastTime<lineIntervalMs) return;
  lineLastTime=millis();

  //累加三个舵机的每步增量
  lineServoCount[0]+=lineServoStep[0];
  lineServoCount[1]+=lineServoStep[1];
  lineServoCount[2]+=lineServoStep[2];
  lineServoIndex++;

  //误差累积：够 +1 度就正转一度，够 -1 度就反转一度
  while(lineServoCount[0]>=1.0f){ Mpos++; Middle.write(Mpos); lineServoCount[0]-=1.0f; }
  while(lineServoCount[0]<=-1.0f){ Mpos--; Middle.write(Mpos); lineServoCount[0]+=1.0f; }

  while(lineServoCount[1]>=1.0f){ Rpos++; Right.write(Rpos); lineServoCount[1]-=1.0f; }
  while(lineServoCount[1]<=-1.0f){ Rpos--; Right.write(Rpos); lineServoCount[1]+=1.0f; }

  while(lineServoCount[2]>=1.0f){ Lpos++; Left.write(Lpos); lineServoCount[2]-=1.0f; }
  while(lineServoCount[2]<=-1.0f){ Lpos--; Left.write(Lpos); lineServoCount[2]+=1.0f; }

  //走满 steps 步即到终点
  if(lineServoIndex>=lineServoSteps)
  {
    lineActive=false;
    Serial.println("[直线] 已到达终点");
  }
}

void Action_Linenext()
{
  if(Line1==0) return;
  if(Line1==1)
  {
    if(millis()-nowtime<3000 || lineActive) return;
    nowtime=millis();
    moveLineTo(150,0,50);
    Line1=0;
    return;
  }
  if(Line1==2)
  {
    if(millis()-nowtime<3000 || lineActive) return;
    nowtime=millis();
    moveLineTo(150,0,50);
    Line1=3;
    return;
  }
  if(Line1==3)
  {
    if(millis()-nowtime<2000 || lineActive) return;
    nowtime=millis();
    moveLineTo(90,0,51);
    Line1=4;
    return;
  }
  if(Line1==4)
  {
    if(millis()-nowtime<2000 || lineActive) return;
    nowtime=millis();
    moveLineTo(150,40,49);
    Line1=0;
    return;
  }
}
