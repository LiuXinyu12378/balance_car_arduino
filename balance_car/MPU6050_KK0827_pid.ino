
//24l01接m2560引脚...
//* MISO -> D50  
// * MOSI ->D51  
// * SCK ->D52

 //* CE ->D53
 //* CSN ->D38
 


#include "Wire.h"
//-----------------
#include "SPI.h"  
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"
//-----------------

//#include <LiquidCrystal.h>;
//LiquidCrystal lcd( 12, 11, 10, 9, 8,7);

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;
//---------------
int16_t ax, ay, az;
int16_t gx, gy, gz;
//========================

#define Gry_offset -20     // 陀螺仪偏移量
#define Gyr_Gain 0.00763358    //对应的1G
#define pi 3.14159



/*********** PID控制器参数 *********/
float kp, ki, kd,kpp; 
float angleA,omega;
//float P[2][2] = {{ 1, 0 },{ 0, 1 }};
//float Pdot[4] ={ 0,0,0,0};
//static const double C_0 = 1;
//float abcd=0.0;
float LOutput,ROutput;   
char buffer [4];
//--------------------------------------
float LSpeed_Need=0.0,RSpeed_Need=0.0;
//char data;
int data,adata;

  unsigned long now;
unsigned long preTime = 0;
float SampleTime = 0.08;  //-------------------互补滤波+PID 采样时间0.08 s
unsigned long lastTime;
float Input, Output, Setpoint;
float errSum,dErr,error,lastErr,f_angle;
int timeChange; 
//-------------------------------l298接m2560引脚---------------------------
int TN1=6;
int TN2=7;
int ENA=5;

int TN3=9;
int TN4=10;
int ENB=3;
int LED1=2;
int LED2=4;
int LED3=8;

//-------------------------------------
void setup() {
 Wire.begin();
//lcd.begin(16, 2);
//lcd.print("hello, world!");
accelgyro.initialize();
delay(100); 

pinMode(TN1,OUTPUT);         
pinMode(TN2,OUTPUT);
pinMode(TN3,OUTPUT);
pinMode(TN4,OUTPUT);
pinMode(ENA,OUTPUT);
pinMode(ENB,OUTPUT);
      pinMode(LED1,OUTPUT);
      pinMode(LED2,OUTPUT); 
      pinMode(LED3,OUTPUT);
   
delay(100);
   Mirf.spi = &MirfHardwareSpi;   //加载24L01  SPI
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");//接收到"接收地址"
  Mirf.payload = sizeof(int);   //接收类型（整数）
 // mirf.channel = ();
 //Mirf.configRegister(EN_AA,0x00);  //Disable auto ack
delay(100);
      Mirf.config();         
    delay(500);
Serial.begin(9600);
    delay(200); 
    Serial.print("蓝牙已准备好");
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(1000);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(500);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(300);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(200);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(200);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(200);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(200);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    delay(500);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);  
}

void loop() {
  llD();//---------------------------接收----------------------
 // if (abs(angleA>38)){LSpeed_Need*=0.6;RSpeed_Need*=0.6;} 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //------------------------------------------------------------------------
  angleA= atan2(ay , az) * 180 / pi+0.5;   // 根据加速度分量得到的角度(degree)加0.5偏移量
  //180度至0至-180（360度）取0度为坚直时中立点 因为坚直时有偏差，所以加0.5....
  omega=  Gyr_Gain * (gx +  Gry_offset); // 当前角速度(degree/s)

  if (abs(angleA)<45) {    // 角度小于45度 运行程序

PIDD();//---------------------互补滤波+PID--------------------------

PWMB(); //----------------------PWM调速输出--------------------------

//LCDD();
 delay(3);
   }
    else {      // 角度大于45度 停止PWM输出
analogWrite(ENA, 0); 
analogWrite(ENB, 0);        
  }
 }
        //---------------------------接收----------------------
 void llD(){
if(Serial.available()>0){   //接收到信号，开始 
       //接收数据  
     Serial.readBytes(buffer,4);
     adata=*buffer;
     Serial.println(adata);
     if (adata==53){LSpeed_Need=0;RSpeed_Need=0;Setpoint=0;digitalWrite(LED1, LOW);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);}//停止
else if (adata==49){LSpeed_Need=2;  digitalWrite(LED1, HIGH);digitalWrite(LED2, HIGH);digitalWrite(LED3, LOW);}  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>前进-----------------------------------
else if (adata==50){LSpeed_Need=-3; digitalWrite(LED1, LOW);digitalWrite(LED2, LOW);digitalWrite(LED3, HIGH);}//后退
else if (adata==51){RSpeed_Need=15;digitalWrite(LED1, LOW);digitalWrite(LED2, HIGH);digitalWrite(LED3, LOW);} //左
else if (adata==52){RSpeed_Need=-15;digitalWrite(LED1, HIGH);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);} // 右
else if (adata==56){RSpeed_Need=30;digitalWrite(LED1, LOW);digitalWrite(LED2, HIGH);digitalWrite(LED3, LOW);} //左
else if (adata==57 ){RSpeed_Need=-30;digitalWrite(LED1, HIGH);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);} // 右
else if (adata==54){digitalWrite(LED1, HIGH);digitalWrite(LED2, HIGH);digitalWrite(LED3, HIGH);} 
else if (adata==55){digitalWrite(LED1, LOW);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);} 
else {LSpeed_Need=0;RSpeed_Need=0;Setpoint=0;}//停止
    }
     delay(2);
  }
//---------------------互补滤波+PID-----------------
 void  PIDD(){ 
      kp = analogRead(0)*0.03; //取0~1024*  (这些值是小车调试后得出，请按自己小车调试后修改)
 ki = analogRead(1)*0.0002;//取0~1024* .........
 kd = analogRead(2)*1;  //取0~1024* .........
 //kpp = analogRead(11)*0.005; 
    //------------------互补滤波 ------------------------
  unsigned long now = millis();                           // 当前时间(ms)
    float dt = (now - preTime) / 1000.0;                    // 微分时间(ms)
    preTime = now;  
    float K = 0.8;                    
    float A = K / (K + dt);                    
   f_angle = A * (f_angle + omega * dt) + (1-A) * angleA;  // 互补滤波算法 
//----------------------------PID控制器 ------------------------------ 
  //now = millis();
   timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
     Setpoint=LSpeed_Need;// =0，(+ -值使电机前进或后退)
        Input =f_angle;
      error = Setpoint- Input;
      errSum += error* timeChange;
      dErr = (error - lastErr)/ timeChange;
 //PID Output
      Output = kp * error + ki * errSum + kd * dErr;
 LOutput=Output+RSpeed_Need;//左电机
 ROutput=Output-RSpeed_Need;//右电机
      lastErr = error;
      lastTime = now;
   }
   //  Serial.print(angleA); Serial.print("\t");
    //   Serial.print(omega);Serial.print("\t");
   // Serial.print(f_angle); Serial.print("\t");
    ///  Serial.println(Output);
 }
 //----------------------PWM调速输出--------------------------
void PWMB(){
  if(LOutput>0.3)//左电机-------或者取0
{

    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
}
  else if(LOutput<-0.3)//-------或者取0
{
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
}
  else  //刹车-------取0后可以不用
{
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, HIGH);
}
if(ROutput>0.3)//右电机--------或者取0
{
     digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW); 

}
  else if(ROutput<-0.3)//-------或者取0
{
     digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);  
}
  else//刹车-------取0后可以不用
{
     digitalWrite(TN3, HIGH);
    digitalWrite(TN4, HIGH);  
}

    analogWrite(ENA,min(255,abs(LOutput)+40)); //PWM调速a==0-255
    analogWrite(ENB,min(255,abs(ROutput)+30));
}
/*
 void LCDD(){

       lcd.clear();
      lcd.setCursor(0,0);
      //lcd.print(m); 
      //lcd.setCursor(4,0);
      lcd.print(aax); 
      lcd.setCursor(5,0);
      lcd.print(aaxdot); 
       lcd.setCursor(12,0);
      lcd.print(Output);  
      lcd.setCursor(0,1);
      lcd.print(kp); 
      lcd.setCursor(4,1);
      lcd.print(ki); 
      lcd.setCursor(8,1);
      lcd.print(kd);
      lcd.setCursor(12,1);
      lcd.print(kpp);
        
       Serial.print("a/g:\t");
    Serial.print(ax/10); Serial.print("\t");
    Serial.print(ay/10); Serial.print("\t");
    Serial.print(az/10); Serial.print("\t");
    Serial.print(gx/10); Serial.print("\t");
    Serial.print(gy/10); Serial.print("\t");
    Serial.print(gz/10);Serial.print("\t");
    Serial.print(angleA); Serial.print("\t");
    Serial.print(omega); Serial.print("\t");
     Serial.print(angle); Serial.print("\t");
    Serial.print(angle_dot); Serial.print("\t");
    Serial.print(Output); Serial.print("\t");
        Serial.println(oommm); Serial.print("\t");
  
delay(10);
}
*/ 
