#define SensorLeft    3
#define SensorMiddle  4
#define SensorRight   5
unsigned char SL;
unsigned char SM;
unsigned char SR;

#define Lpwm_pin  6
#define Rpwm_pin 11
int pinLB = 7;
int pinLF = 8;
int pinRB = 9;
int pinRF = 10;

unsigned char Lpwm_val = 150;
unsigned char Rpwm_val = 150;

// PID переменные
float Kp = 25;
float Ki = 0;
float Kd = 15;
float error = 0, lastError = 0, integral = 0;

void Sensor_IO_Config() {
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);
}

void Sensor_Scan(void) {
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void M_Control_IO_config(void) {
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);  
}

void Set_Speed(int Left, int Right) {
  analogWrite(Lpwm_pin, constrain(Left, 0, 255));   
  analogWrite(Rpwm_pin, constrain(Right, 0, 255));
}

void forward() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH); 
}

void back() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
}

void stopp() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
}

void setup() { 
  Serial.begin(9600);
  Sensor_IO_Config();
  M_Control_IO_config();
  stopp();
  delay(1000);
}

void loop() {
  Sensor_Scan();

  // Определение ошибки
  if (SL == HIGH && SM == LOW && SR == LOW) error = -1;
  else if (SL == LOW && SM == HIGH && SR == LOW) error = 0;
  else if (SL == LOW && SM == LOW && SR == HIGH) error = 1;
  else if (SL == HIGH && SM == HIGH && SR == LOW) error = -0.5;
  else if (SL == LOW && SM == HIGH && SR == HIGH) error = 0.5;
  else if (SL == LOW && SM == LOW && SR == LOW) {
    back();
    delay(100);
    stopp();
    return;
  }

  // PID расчет
  float P = error;
  integral += error;
  float D = error - lastError;
  lastError = error;

  float correction = Kp * P + Ki * integral + Kd * D;

  int baseSpeed = 150;
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  Set_Speed(leftSpeed, rightSpeed);
  forward();

  delay(10); // пауза для стабильной работы
}
