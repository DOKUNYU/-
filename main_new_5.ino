 /*********************************PPM*********************************/
//PPM引脚用D10-13
#include "ppm.h"
#define THROTTLE        3
#define ROLL            1
#define PITCH           2
#define YAW             4
#define SB              5
#define SA              6
#define SD              7     // trim-pot for left/right motor mix  (face trim)
#define V1              8     // trim-pot on the (front left edge trim)
#define MiddlePoint     1508
/*******************************servo********************/
#include <Servo.h>
Servo bottom;
Servo upper;
Servo left;
Servo right;

// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

int counter=0;
int anglex=90;
int angley=90;
int addressx[4]={0,0,0,0};
int addressy[4]={0,0,0,0};

//Varible Initialization
short throttle,roll,pitch,yaw,sb,sa,sd,v1;
/*********************************TB6612*********************************/
#define STBY1 22
#define STBY2 24
#define STBY3 23
#define AIN1  26
#define AIN2  28
#define BIN1  30
#define BIN2  32
#define CIN1  34
#define CIN2  36
#define DIN1  38
#define DIN2  40
#define EIN1  25
#define EIN2  27
#define PWMA  8
#define PWMB  9
#define PWMC  10
#define PWMD  11
#define PWME  12
/////////编码器引脚////////
#define ENCODER_A 2
#define ENCODER_B 4
/*********************************PID*********************************/
//速度
int lastError=0;
int allError=0;
int delta=0;
//角度
float Velocity,Position=0,Motor;            //速度和位置测量值
float Target_Position=0,Target_Velocity=0;  //目标速度和目标位置
float Position_KP=120,Position_KI=15,Position_KD=800,Velocity_KP=20,Velocity_KI=30;
int last_time=0,current_time,last_position;
const int time_gap=20;
/*********************************人工设置*********************************/
const int DeadZone=40;
const int Domain=470;
const int maxLinearSpeed=220;
/*********************************串口通讯*********************************/

int data=0;
int data1[5]={0};
int data2[4]={0};
int flag=0;
int coordinate=0;
int coordinate1[3]={0};
int coordinate2[2]={0};
int flag1=0;

/**********************************************我是大分割线**********************************************/
void setup() {
  //PPM
  Serial.begin(38400);
  ppm.begin(13, false);

  //servo
  upper.attach(7);
  bottom.attach(6);
  upper.write(90);
  bottom.write(90);
  left.attach(44);
  left.write(20);
  right.attach(45);
  right.write(20);
  
  
  
  //TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY1, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(STBY2, OUTPUT);
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY1, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(CIN1, 1);
  digitalWrite(CIN2, 0);
  digitalWrite(DIN1, 1);
  digitalWrite(DIN2, 0);
  digitalWrite(STBY2, 1);
  analogWrite(PWMC, 0);
  analogWrite(PWMD, 0);
  
//拨轮
  pinMode(EIN1, OUTPUT);
  pinMode(EIN2, OUTPUT);
  pinMode(PWME, OUTPUT);
  pinMode(STBY3, OUTPUT);
  digitalWrite(STBY3, 1);
  digitalWrite(EIN1,0);
  digitalWrite(EIN2,0);
  //编码器
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);


  
  //开启外部中断 
  attachInterrupt(0, READ_ENCODER_A, CHANGE);           
  attachInterrupt(4, READ_ENCODER_B, CHANGE);
  
  //电脑控制
  Serial1.begin(38400);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only}
  }
  //视觉
  Serial2.begin(38400);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only}
  }
 }
/**********************************************我是大分割线**********************************************/
void loop() {
  // put your main code here, to run repeatedly:
  ReadPPM();
  //PrintPPM();
  if(sb == -1)//底盘控制模式
  { if (sa==-1)//摩擦轮停止
    left.write(20);
    right.write(20);
    {mecanum(roll,pitch,yaw);}
    if(sa==1)
    {shoot();
    fullAuto();}
  }
  if(sb==0)//云台和发射控制模式
  { //摩擦轮启旋
    left.write(160);//20-160
    right.write(160);
    servocontrol(pitch,roll);
    if(sa==-1)//半自动模式
    {semiAuto(); }
    if(sa==1)//全自动模式
    { fullAuto();}  
  }
  if(sb==1)//符文模式
  {
    if(sa==-1)//标定模式
    { servocontrol(pitch,roll);
      address();}
    if(sa==1)//手控模式
    { left.write(160);//20-160
      right.write(160);
      ComputerControl();}
   }
  
  
}
/**********************************************我是大分割线**********************************************/
void mecanum(float xSpeed, float ySpeed, float aSpeed)
{
    float speed1 = ySpeed + xSpeed + aSpeed; 
    float speed2 = ySpeed - xSpeed - aSpeed;
    float speed3 = ySpeed + xSpeed - aSpeed;
    float speed4 = ySpeed - xSpeed + aSpeed;
    
    float max = abs(speed1);
    if (max < abs(speed2))  max = abs(speed2);
    if (max < abs(speed3))  max = abs(speed3);
    if (max < abs(speed4))  max = abs(speed4);
    
    if (max > maxLinearSpeed)
    {
        speed1 = speed1 / max * maxLinearSpeed;
        speed2 = speed2 / max * maxLinearSpeed;
        speed3 = speed3 / max * maxLinearSpeed;
        speed4 = speed4 / max * maxLinearSpeed;
    }
    
    SetMotor(speed1, speed2, speed3, speed4);

    //display
    /*Serial.print(speed1); Serial.print("  ");
    Serial.print(speed2); Serial.print("  ");
    Serial.print(speed3); Serial.print("  ");
    Serial.print(speed4); Serial.print("  ");
    Serial.println();*/
}
/*********************************我是分割线*********************************/
void SetMotor(int speed1,int speed2,int speed3,int speed4)
{
  SetPWM(1,-speed1);
  SetPWM(2,speed2);
  SetPWM(3,speed3);
  SetPWM(4,-speed4);
}
/*********************************我是分割线*********************************/
void SetPWM(int motor, int pwm)
{
  if(pwm>255)
    pwm=255;
  else if(pwm<-255)
    pwm=-255;
    
  if(motor==1&&pwm>=0)
  {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  }
  else if(motor==1&&pwm<0)
  {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  }
  else if(motor==2&&pwm>=0)
  {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, pwm);
  }
  else if(motor==2&&pwm<0)
  {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, -pwm);
  }
  else if(motor==3&&pwm>=0)
  {
    digitalWrite(CIN1, 1);
    digitalWrite(CIN2, 0);
    analogWrite(PWMC, pwm);
  }
  else if(motor==3&&pwm<0)
  {
    digitalWrite(CIN1, 0);
    digitalWrite(CIN2, 1);
    analogWrite(PWMC, -pwm);
  }
  else if(motor==4&&pwm>=0)
  {
    digitalWrite(DIN1, 0);
    digitalWrite(DIN2, 1);
    analogWrite(PWMD, pwm);
  }
  else if(motor==4&&pwm<0)
  {
    digitalWrite(DIN1, 1);
    digitalWrite(DIN2, 0);
    analogWrite(PWMD, -pwm);
  }
  else if(motor==5&&pwm>=0)
  {
    digitalWrite(EIN1, 0);
    digitalWrite(EIN2, 1);
    analogWrite(PWME, pwm);
  }
  else if(motor==5&&pwm<0)
  {
    digitalWrite(EIN1, 1);
    digitalWrite(EIN2, 0);
    analogWrite(PWME, -pwm);
  }
}

/**********************************************我是大分割线**********************************************/
void servocontrol(int y,int x)
{   int a,b;//比例
    a=y*-0.01;
    b=x*-0.01;
    angley=angley+a;
    if(angley>90) angley=90;
    else if(angley<30) angley=30;
    anglex=anglex+b;
    if(anglex>180) anglex=180;
    else if(anglex<0) anglex=0;
    upper.write(angley);
    bottom.write(anglex);
    /*Serial.print("anglex:");
    Serial.print(anglex);
    Serial.print("\n");*/
    delay(10);
  }
/*********************************我是分割线*********************************/
void semiAuto()
{
  if(sd==1)
  {
    Position=0;
    while(sd==1)
    {
      ReadPPM();
    }
    Target_Position+=46;//=390*30/21.3/2/6
  }
  Motor=Position_PID(Position,Target_Position);
  SetPWM(5,-Motor);
}
/*********************************我是分割线*********************************/
void fullAuto()
{
  if(sd==1)
    {
      SetPWM(5,255);
      while(sd==1)
      {
        ReadPPM();
      }
      SetPWM(5,0);
    }
  }
  /*********************************我是分割线*********************************/
void address()
{ ReadPPM();
 if(sd==1)
 {while(sd==1)
 {
  ReadPPM();
 }
 Serial.print("anglex:");Serial.println(anglex);
 Serial.print("angley:");Serial.println(angley);
 Serial.print("counter:");Serial.println(counter);
 if(counter >=6)counter=0;
  switch(counter){
  case 1:
  addressx[1]=anglex;addressy[1]=angley;counter += 1;
  Serial.print("addressx[1]:");Serial.println(addressx[1]);
  Serial.print("addressy[1]:");Serial.println(addressy[1]);
  break;
  case 2:
  addressx[2]=anglex;counter += 1;
  Serial.print("addressx[2]:");Serial.println(addressx[2]);break;
  case 3:
  addressx[3]=anglex;counter += 1;
  Serial.print("addressx[3]:");Serial.println(addressx[3]);break;
  case 4:
  addressy[2]=angley;counter += 1;
  Serial.print("addressy[2]:");Serial.println(addressy[2]);break;
  case 5:
  addressy[3]=angley;counter += 1;
  Serial.print("addressy[3]:");Serial.println(addressy[3]);break;
  default:counter += 1;break;}
 
  
  }
}
/*********************************我是分割线*********************************/
void ComputerControl()
{
  while(Serial1.available()>0) //接收数据
  {
    data=Serial1.parseInt();
    if(data!=0)
    {
     Serial.println(data);}
  }
  for(int i=4;i>=0;i--)
  { flag =1;
    data1[i]=data%10;
    data=data/10;
  }
  for(int i=0;i<=2;i++)
  {
    data2[i]=data1[i];
  }
   data2[3]=data1[3]*10+data1[4];

   if(data2[3]!=data2[0]+data2[1]+data2[2])//校验数据
   {
    Serial1.write("99999");
    flag=0;
   }
  
   if(flag==1)
   { for(int i=0;i<=2;i++)//击打符文
    {
    switch(data2[i]){
    case 1: bottom.write(addressx[1]);upper.write(addressy[3]);delay(100);break;
    case 2: bottom.write(addressx[2]);upper.write(addressy[3]);delay(100);break;
    case 3: bottom.write(addressx[3]);upper.write(addressy[3]);delay(100);break;
    case 4: bottom.write(addressx[1]);upper.write(addressy[2]);delay(100);break;
    case 5: bottom.write(addressx[2]);upper.write(addressy[2]);delay(100);break;
    case 6: bottom.write(addressx[3]);upper.write(addressy[2]);delay(100);break;
    case 7: bottom.write(addressx[1]);upper.write(addressy[1]);delay(100);break;
    case 8: bottom.write(addressx[2]);upper.write(addressy[1]);delay(100);break;
    case 9: bottom.write(addressx[3]);upper.write(addressy[1]);delay(100);break;
    defalut:break;
    }
    SetPWM(5,255);
    delay(1000);
    SetPWM(5,0);
    }
   }
  }
/*********************************我是分割线*********************************/
void shoot()
{
  while(Serial2.available()>0) //接收数据
  {
    coordinate=Serial2.parseInt();
    if(coordinate!=0)
    {
     Serial.println(coordinate);}
  }
  for(int i=2;i>=0;i--)
  { flag1 =1;
    coordinate1[i]=coordinate%1000;
    coordinate=coordinate/1000;
  }
  for(int i=0;i<=1;i++)
  {
    coordinate2[i]=coordinate1[i];
  }
   

   if(coordinate2[2]!=coordinate2[0]+coordinate2[1])//校验数据
   {
    Serial2.write("99999");
    flag=0;
   }
  
   if(flag==1)
  {
    servocontrol(coordinate2[1],coordinate2[0]);
    }
  
  } 

/*********************************我是分割线*********************************/
void ReadPPM()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Acquiring all the channels values
    throttle      =   (ppm.read_channel(THROTTLE)-MiddlePoint)*0.545;
    roll          =   (ppm.read_channel(ROLL)-MiddlePoint)*0.545;
    pitch         =   (ppm.read_channel(PITCH)-MiddlePoint)*0.545;
    yaw           =   (ppm.read_channel(YAW)-MiddlePoint)*0.545;
    sb            =   ppm.read_channel(SB)-MiddlePoint;
    sa            =   ppm.read_channel(SA)-MiddlePoint;
    sd            =   ppm.read_channel(SD)-MiddlePoint;
    v1            =   ppm.read_channel(V1)-MiddlePoint;
    
    //Dead Zone
    if(abs(throttle)<DeadZone) throttle=0;
    if(abs(roll)<DeadZone)     roll=0;
    if(abs(pitch)<DeadZone)    pitch=0;
    if(abs(yaw)<DeadZone)      yaw=0;

    //Switch Transform
    if(sb<-300)                 sb=-1;
    else if(sb>-100 && sb<100)  sb=0;
    else if(sb>300)             sb=1;

    if(sa<-300)                 sa=-1;
    else if(sa>-100 && sa<100)  sa=0;
    else if(sa>300)             sa=1;

    if(sd<-300)                 sd=-1;
    else if(sd>-100 && sd<100)  sd=0;
    else if(sd>300)             sd=1;
  }
}
/*********************************我是分割线*********************************/
/*void PrintPPM()
{
  Serial.print("Throttle:");        Serial.print(throttle);       Serial.print(" ");
  Serial.print("Roll:");            Serial.print(roll);           Serial.print(" ");
  Serial.print("Pitch:");           Serial.print(pitch);          Serial.print(" ");
  Serial.print("Yaw:");             Serial.print(yaw);            Serial.print(" ");
  Serial.print("SB:");              Serial.print(sb);             Serial.print(" ");
  Serial.print("SA:");              Serial.print(sa);             Serial.print(" ");
  Serial.print("SD:");              Serial.print(sd);             Serial.print(" ");
  Serial.print("V1:");              Serial.print(v1);             Serial.print(" ");
  Serial.println(); 
}*/
/*********************************我是分割线*********************************/
void READ_ENCODER_A() {
    if (digitalRead(ENCODER_A) == HIGH) {     
    if (digitalRead(ENCODER_B) == LOW)      Position++;  //根据另外一相电平判定方向
    else      Position--;
  }
    else {    
    if (digitalRead(ENCODER_B) == LOW)      Position--; //根据另外一相电平判定方向
    else     Position++;
  }
}
void READ_ENCODER_B() {
    if (digitalRead(ENCODER_B) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(ENCODER_A) == LOW)      Position++;//根据另外一相电平判定方向
    else      Position--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(ENCODER_A) == LOW)      Position--; //根据另外一相电平判定方向
    else     Position++;
  }
}

/*********************************我是分割线*********************************/
int Position_PID (int Encoder,int Target)
{   
   static float Pwm,Integral_bias,Last_Bias;
   float Bias;
   Bias=Encoder-Target;
   Integral_bias+=Bias;
   Pwm=Position_KP*Bias/28+Position_KI*Integral_bias/2800+Position_KD*(Bias-Last_Bias)/28;
   Last_Bias=Bias;

   if(Pwm>255) Pwm=255;
   if(Pwm<-255) Pwm=-255;
   return Pwm;
}
