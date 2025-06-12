#define SensorLeft    3   //input pin of left sensor
#define SensorMiddle  4   //input pin of middle sensor
#define SensorRight   5   //input pin of right sensor
unsigned char SL;        //state of left sensor
unsigned char SM;        //state of middle sensor
unsigned char SR;        //state of right sensor

#define Lpwm_pin  6     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  11    //pin of controlling speed---- ENA of motor driver board
int pinLB=7;            //pin of controlling diversion----IN1 of motor driver board
int pinLF=8;            //pin of controlling diversion----IN2 of motor driver board
int pinRB=9;            //pin of controlling diversion----IN3 of motor driver board
int pinRF=10;            //pin of controlling diversion----IN4 of motor driver board

unsigned char Lpwm_val =180; //the speed of left wheel at 180 in initialization
unsigned char Rpwm_val = 180; //the speed of right wheel at 180 in initialization

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
}
    
void softTurnL() 
{
  analogWrite(Lpwm_pin, 50); // замедлить левое колесо
  analogWrite(Rpwm_pin, 120); 
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
}

void softTurnR() 
{
  analogWrite(Lpwm_pin, 120);
  analogWrite(Rpwm_pin, 50); // замедлить правое колесо
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
}
   
 
void stopp()         //stop
{
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
}

void back()          //back
{
  digitalWrite(pinRB,HIGH);  //making motor move towards right rear
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  //making motor move towards left rear
  digitalWrite(pinLF,LOW);
}

void setup() 
{ 
   Sensor_IO_Config();
   M_Control_IO_config();        //motor controlling the initialization of IO
   Set_Speed(Lpwm_val,Rpwm_val); //setting initialization of speed
   stopp();
} 

void loop() 
{ 
  Sensor_Scan();
  if (SM == HIGH)// middle sensor in black area
  {
    if (SL == LOW && SR == HIGH) // black on left, white on right, turn left
    {
      softTurnR();
    }
    else if (SL == HIGH && SR == LOW) // white on left, black on right, turn right
    {
      softTurnL();
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
      softTurnR();
    }
    else if (SL == HIGH && SR == LOW) // white on left, black on right, turn right
    {
      softTurnL();
    }
    else // all white, stop
    {
      back();
      delay(100);
      stopp() ; 
    }
  }

  delay(10);
} 
