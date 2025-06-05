#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

// === Пины ===
const int sensorLeft = 3;
const int sensorCenter = 4;
const int sensorRight = 5;

// Левые моторы (A1 + A2)
const int IN1 = 6;
const int IN2 = 7;
const int ENA = 10;

// Правые моторы (B1 + B2)
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 11;

const int IR_PIN = 2;
IRrecv irrecv(IR_PIN);
decode_results results;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int sensorL, sensorC, sensorR;

// === Настройки скорости ===
int speedValue = 50;
int speedStep = 5;     // шаг изменения
int minSpeed = 50;     // минимальная скорость
int maxSpeed = 200;    // максимальная скорость
bool speedUp = true;   // направление изменения

void setup() {
  Serial.begin(9600);

  // Датчики
  pinMode(sensorLeft, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorRight, INPUT);

  // Моторы 1
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  // 2
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Testing...");

  // IR
  irrecv.enableIRIn();
}

void loop() 
{
  
  // Вперёд сразу 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedValue);
  // 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedValue);

  // === Обновление скорости
  if (speedUp) {
    speedValue += speedStep;
    if (speedValue >= maxSpeed) speedUp = false;
  } else {
    speedValue -= speedStep;
    if (speedValue <= minSpeed) speedUp = true;
  }
  
  // Датчики линии
  sensorL = digitalRead(sensorLeft);
  sensorC = digitalRead(sensorCenter);
  sensorR = digitalRead(sensorRight);

  // Вывод на дисплей
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(sensorL);
  lcd.print(" C:");
  lcd.print(sensorC);
  lcd.print(" R:");
  lcd.print(sensorR);

  // IR-приём
  if (irrecv.decode(&results)) {
    Serial.print("IR: 0x");
    Serial.println(results.value, HEX);

    lcd.setCursor(0, 1);
    lcd.print("IR: 0x");
    lcd.print(results.value, HEX);
    lcd.print("     "); // затереть остаток

    irrecv.resume();
  }

  delay(200);
}
