#include <Wire.h>
//#include "I2Cdev.h"
#include "MPU6050.h"
#include <MsTimer2.h>
#include <Servo.h>
#define  Value 1.02101761  //计算周长的常量
#include <dht11.h>
#define DHT11PIN 3
#define tonePin 8
#define inputPin 4
#define outputPin 10

dht11 DHT11;

Servo servo;
int pos = 0;

int l_wheel = 0;
float l_velocity = 0;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

int ST_CP = 11;
int SH_CP = 12;
int DS = 13;
int hc595key = B00000000;

int motor1 = 6;
int motor2 = 5;
float leftRightSpeed = 0;
float goBackSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(SH_CP, OUTPUT);
  pinMode(DS, OUTPUT);
  pinMode(ST_CP, OUTPUT);
  pinMode(tonePin, OUTPUT);
  pinMode(inputPin, INPUT);
pinMode(outputPin, OUTPUT);
  Wire.begin();
  Serial.begin(115200);

  analogWrite(motor1, 0);
  analogWrite(motor2, 0);
  hc595(10);
  hc595(20);
  hc595Stop(B10001111);
  accelgyro.initialize();
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  attachInterrupt(0, LCount, FALLING);
  MsTimer2::set(500, flash); // 中断设置函数，每 0.5s 进入一次中断
 // MsTimer2::set(10,mpuSendValue);
  MsTimer2::start();

  servo.attach(9);
  servo.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()) {

    String keyWord = Serial.readStringUntil('*');

    if (keyWord.equals("BOTHSTOP")) {
      bothStop();

    }
    if (keyWord.equals("LEFTRIGHT")) {
      leftRightSpeed = Serial.parseFloat();
    }
    if (keyWord.equals("GOBACK")) {
      goBackSpeed = Serial.parseFloat();
    }
    updateCar(leftRightSpeed, goBackSpeed);
    if (keyWord.equals("WLEDON")) {
      hc595Start(B01000000);
    }
    if (keyWord.equals("WLEDOFF")) {
      hc595Stop(B10111111);
    }
    if (keyWord.equals("TONE")) {
      float toneValue = Serial.parseFloat();
      toneFun(toneValue);
    }
    
  }

  static unsigned long DHT11Timer = millis();     //every 2s update the temperature and humidity from DHT11 sensor
  if (millis() - DHT11Timer >= 1123.23) {
    DHT11Timer = millis();
    dht11SendValue();
  }
  
  static unsigned long lg = millis();     //every 2s update the temperature and humidity from DHT11 sensor
  if (millis() - lg >= 111) {
    lg = millis();
    mpuSendValue();
  }

  servoSendValue();

}
bool isClock;
static unsigned long se;
void servoSendValue() {
  if (pos == 0) {
    isClock = true;
  } else if (pos == 180) {
    isClock = false;
  }
  if (isClock) {
    //for (pos=0; pos < 180;) {
      if (millis() - se >= 56) {
        se = millis();
        int dis = readSr04();
         String c = "P:"+String(pos)+","+String(dis)+"@";
        if(dis>2){}
        Serial.print(c);
        //Serial.flush();
        
        servo.write(pos);
       
        pos++;
      }
   //}
  } else {

   // for (pos = 180; pos >= 1; ) {
      if (millis() - se >= 56) {
        se = millis();
        int dis = readSr04();
        String c = "P:"+String(pos)+","+String(dis)+"@";
        if(dis>2){
        Serial.print(c);
        //Serial.flush();
        }
        servo.write(pos);
        pos--;
      }

    //}
  }
}

int readSr04() {
  digitalWrite(outputPin, LOW); // 使发出发出超声波信号接口低电平2μs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // 保持发出超声波信号接口低电平
  int distance = pulseIn(inputPin, HIGH); // 读出脉冲时间
  distance = distance / 58; // 将脉冲时间转化为距离（单位：厘米）
  return distance;
}
void LCount() {
  l_wheel++;
}
float velocity(int n) {
  float vel = Value * (n / 20);
  return vel;
}
void flash() {
  int l;
  l = l_wheel;
  l_velocity = velocity(l);
  String s = "S:" + String(l_velocity) + "!";
  Serial.print(s);
  //Serial.flush();
  l_wheel = 0;
}
void toneFun(float toneValue) {
  if (toneValue == 0) {
    noTone(tonePin);
    return;
  } else {
    float sinVal = sin(map(toneValue, 0, 45, 0, 180) * 3.14159 / 180);
    int toneVal = 2000 + int(sinVal * 1000);
    tone(tonePin, toneVal);

  }
}
void mpuSendValue() {
  //accelgyro.getRotation(&gx, &gy, &gz);
  //accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  String mpuag = "A:" + String(map(gx, -35000, 35000, -99, 99)) + "," + String(map(gy, -35000, 35000, -99, 99)) + "," + String(map(gz, -35000, 35000, -99, 99)) + "," + String(map(ax, -16900, 16900, -50, 50)) + "," + String(map(ay, -16900, 16900, -50, 50)) + "," + String(map(mx, -20000, 20000, -99, 99)) + "!";

  //sprintf(mpug,"MPUG:%d,%d,%d",gx,gy,gz);
  Serial.print(mpuag);
  //Serial.flush();

}
void mpuaSendValue() {
  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.getAcceleration(&ax, &ay, &az);

  String mpua = "MPUA:" + String(map(ax, -16900, 16900, -50, 50)) + "," + String(map(ay, -16900, 16900, -50, 50));
  // sprintf(mpua,"MPUA:%d,%d",ax,ay);

  Serial.print(mpua);
  //Serial.flush();
}

void dht11SendValue() {
  int chk = DHT11.read(DHT11PIN);
  switch (chk)
  {
    case DHTLIB_OK:
      //Serial.println("OK");
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Checksum error");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("Time out error");
      break;
    default:
      Serial.println("Unknown error");
      break;
  }

  String humAndTem;
  float hum = (float)DHT11.humidity;
  float tem = (float)DHT11.temperature;
  //sprintf(humAndTem,"HUMANDTE:%f,%f",hum,tem);
  humAndTem = "HUM:" + String(hum) + "," + String(tem) + "!";
  Serial.print(humAndTem);
  //Serial.flush();
}
void bothStop() {
  analogWrite(motor1, 0);
  analogWrite(motor2, 0);
  hc595(10);
  hc595(20);
  hc595Stop(B11000111);
}
void updateCar(float leftRightSpeed, float goBackSpeed) {
  //xx
  if (leftRightSpeed == 0 && goBackSpeed == 0) {
    bothStop();
  }
  //goback
  else if (leftRightSpeed == 0 && goBackSpeed != 0) {
    if (goBackSpeed < 0) { //go
      analogWrite(motor1, -goBackSpeed);
      analogWrite(motor2, -goBackSpeed);
      hc595(10);
      hc595(20);
      hc595Stop(B11000111);

    }
    else if (goBackSpeed > 0) { //back
      analogWrite(motor1, 255 - goBackSpeed);
      analogWrite(motor2, 255 - goBackSpeed);
      hc595(11);
      hc595(21);
      hc595Start(B00100000);
      hc595Stop(B11100111);
    }
  }
  //leftright
  else if (leftRightSpeed != 0 && goBackSpeed == 0) {
    if (leftRightSpeed < 0) { //left
      analogWrite(motor1, 255 + leftRightSpeed);
      analogWrite(motor2, -leftRightSpeed);
      hc595(11);
      hc595(20);
      hc595Start(B00001000);
      hc595Stop(B11101111);
      hc595Stop(B11011111);
    }
    else if (leftRightSpeed > 0) { //right
      analogWrite(motor1, leftRightSpeed);
      analogWrite(motor2, 255 - leftRightSpeed);
      hc595(10);
      hc595(21);
      hc595Start(B00010000);
      hc595Stop(B11110111);
      hc595Stop(B11011111);
    }
  }
  //go turn left
  else if (leftRightSpeed < 0 && goBackSpeed < 0) {
    analogWrite(motor1, constrain((-goBackSpeed + leftRightSpeed), 0, 255));
    analogWrite(motor2, -goBackSpeed);
    hc595(10);
    hc595(20);
    hc595Start(B00001000);
    hc595Stop(B11101111);
    hc595Stop(B11011111);
  }
  //go turn right
  else if (leftRightSpeed > 0 && goBackSpeed < 0) {
    analogWrite(motor1,  -goBackSpeed);
    analogWrite(motor2, constrain((-goBackSpeed - leftRightSpeed), 0, 255));
    hc595(10);
    hc595(20);
    hc595Start(B00010000);
    hc595Stop(B11110111);
    hc595Stop(B11011111);
  }
  //back turn left
  else if (leftRightSpeed < 0 && goBackSpeed > 0) {
    analogWrite(motor1,  constrain((255 - goBackSpeed - leftRightSpeed), 0, 255));
    analogWrite(motor2, 255 - goBackSpeed);
    hc595(11);
    hc595(21);
    hc595Start(B00001000);
    hc595Stop(B11101111);
    hc595Start(B00100000);
  }
  //back turn right
  else if (leftRightSpeed > 0 && goBackSpeed > 0) {
    analogWrite(motor1,  255 - goBackSpeed);
    analogWrite(motor2, constrain((255 - goBackSpeed + leftRightSpeed), 0, 255));
    hc595(11);
    hc595(21);
    hc595Start(B00010000);
    hc595Stop(B11110111);
    hc595Start(B00100000);
  } else {
    bothStop();
  }

}
void hc595(int w) {
  switch (w) {

    case 10:
      hc595Stop(B11111101);
      break;
    case 11:
      hc595Start(B00000010);
      break;

    case 20:
      hc595Stop(B11111011);
      break;
    case 21:
      hc595Start(B00000100);
      break;
  }
}
//置0
void hc595Stop(int B) {
  hc595key = hc595key & (B);
  digitalWrite(ST_CP, LOW);
  shiftOut(DS, SH_CP, MSBFIRST, hc595key);
  digitalWrite(ST_CP, HIGH);
}
//置1
void hc595Start(int B) {
  hc595key = hc595key | (B);
  digitalWrite(ST_CP, LOW);
  shiftOut(DS, SH_CP, MSBFIRST, hc595key);
  digitalWrite(ST_CP, HIGH);
}


