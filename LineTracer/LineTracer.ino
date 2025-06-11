#include <LiquidCrystal_I2C.h> 
#include <Wire.h> 

#define SensorLeft    3   //input pin of left sensor
#define SensorMiddle  4   //input pin of middle sensor
#define SensorRight   5   //input pin of right sensor
unsigned char SL;        //state of left sensor
unsigned char SM;        //state of middle sensor
unsigned char SR;        //state of right sensor

#define Lpwm_pin  10     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  11    //pin of controlling speed---- ENA of motor driver board
int pinLB=6;            //pin of controlling diversion----IN1 of motor driver board
int pinLF=7;            //pin of controlling diversion----IN2 of motor driver board
int pinRB=8;            //pin of controlling diversion----IN3 of motor driver board
int pinRF=9;            //pin of controlling diversion----IN4 of motor driver board

unsigned char Lpwm_val =180; //the speed of left wheel at 180 in initialization
unsigned char Rpwm_val = 180; //the speed of right wheel at 180 in initialization
int Car_state=0;             //state of car moving
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 

void LCD1602_init(void)
{
  lcd.init();                     
  lcd.backlight(); 
  lcd.clear();  
}

void Sensor_IO_Config()
{
  pinMode(SensorLeft,INPUT);
  pinMode(SensorMiddle,INPUT);
  pinMode(SensorRight,INPUT);
}

void Sensor_Scan(void)
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void M_Control_IO_config(void)//initialized function of IO of motor driver
{
  pinMode(pinLB,OUTPUT); // pin 2--IN1 of motor driver board
  pinMode(pinLF,OUTPUT); // pin 4--IN2 of motor driver board
  pinMode(pinRB,OUTPUT); // pin 7--IN3 of motor driver board
  pinMode(pinRF,OUTPUT); // pin 8--IN4 of motor driver board
  pinMode(Lpwm_pin,OUTPUT); // pin 5  (PWM) --ENA of motor driver board
  pinMode(Rpwm_pin,OUTPUT); // pin 10 (PWM) --ENB of motor driver board  
}

void Set_Speed(unsigned char Left,unsigned char Right)//setting function of speed
{
  analogWrite(Lpwm_pin,Left);   
  analogWrite(Rpwm_pin,Right);
}

void advance()     // going forwards
{
  digitalWrite(pinRB,LOW);  // making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  //  making motor move towards left rear
  digitalWrite(pinLF,HIGH); 
  Car_state = 1; 
  show_state();   
}
    
void turnR()        //turning on the right(dual wheels)
{
  digitalWrite(pinRB,LOW);  //making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);  //making motor move towards left front
  Car_state = 4;
  show_state();
}

void turnL()        //turning on the left(dual wheels)
{
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW );   //making motor move towards right front
  digitalWrite(pinLB,LOW);   //making motor move towards left rear
  digitalWrite(pinLF,HIGH);
  Car_state = 3;
  show_state();
}   
 
void stopp()         //stop
{
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  Car_state = 5;
  show_state();
}

void back()          //back
{
  digitalWrite(pinRB,HIGH);  //making motor move towards right rear
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  //making motor move towards left rear
  digitalWrite(pinLF,LOW);
  Car_state = 2;
  show_state() ;    
}

void show_state(void)  //showing current state of the car
{
  lcd.setCursor(0, 1); //showing from second row
  switch(Car_state)
  {
     case 1:lcd.print(" Go  ");Serial.print("\n GO");
     break;
     case 2:lcd.print("Back ");Serial.print("\n Back");
     break;
     case 3:lcd.print("Left ");Serial.print("\n Left");
     break;
     case 4:lcd.print("Right");Serial.print("\n Right");
     break;
     case 5:lcd.print("Stop ");Serial.print("\n Stop"); 
     break;
     default:
     break;
  }
}

void setup() 
{ 
   LCD1602_init();
   Sensor_IO_Config();
   M_Control_IO_config();        //motor controlling the initialization of IO
   Set_Speed(Lpwm_val,Rpwm_val); //setting initialization of speed
   lcd.clear();
   lcd.setCursor(0, 0);  //cursor set in first row and first column，
   lcd.print("  Wait  Signal  ");
   stopp();
} 

void loop() 
{ 
  Sensor_Scan();
  if (SM == HIGH)// middle sensor in black area
  {
    if (SL == LOW && SR == HIGH) // black on left, white on right, turn left
    {
      turnR(); //turnL
    }
    else if (SL == HIGH && SR == LOW) // white on left, black on right, turn right
    {
      turnL(); //turnR
    }
    else // white on both sides, going forward
    {
      advance();
    }
  }
  else // middle sensor on white area
  {
    if (SL== LOW && SR == HIGH)// black on left, white on right, turn left
    {
      turnR(); //turnL
    }
    else if (SL == HIGH && SR == LOW) // white on left, black on right, turn right
    {
      turnL(); //turnR
    }
    else // all white, stop
    {
      back();
      delay(100);
      stopp() ; 
    }
  }
} 
