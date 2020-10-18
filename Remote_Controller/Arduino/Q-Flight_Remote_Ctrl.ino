#include <LiquidCrystal_I2C.h>

#define TROTTLE A0
#define YAW A1
#define PITCH A2
#define ROLL A3
#define MAX_WAITING_TIME 20

#define FIRST_BTN 3
#define INDICATE_PIN 2

LiquidCrystal_I2C lcd(0x27, 16, 2);
int control_mode=1;
int throttle_old=0;
int yaw_old=0;
int pitch_old=0;
int roll_old=0;
long time=0;

int compare(int a,int b){
  if(abs(a-b)>3){
    return true;
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("SYSTEM SETUP");
  lcd.setCursor(0, 1);
  lcd.print("--By Quix ZQWEI");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Remote Controller");
  pinMode(TROTTLE,INPUT);
  pinMode(YAW,INPUT);
  pinMode(PITCH,INPUT);
  pinMode(ROLL,INPUT);
   for(int i=0;i<4;i++){
    pinMode(i+FIRST_BTN,INPUT_PULLUP);
  }
  time=millis();
}

int clean_data(int input_value){
  if(abs(input_value-50)<=1){
    return 50;
  }
  return input_value;
}

void loop() {
  // put your main code here, to run repeatedly:
  int throttle=(float)analogRead(A0)/1024*100;
  int yaw=(float)analogRead(YAW)/1024*100;
  int pitch=(float)analogRead(PITCH)/1024*100;
  int roll=(float)analogRead(ROLL)/1024*100;

  yaw=clean_data(yaw);
  pitch=clean_data(pitch);
  roll=clean_data(roll);
  
  if(compare(throttle,throttle_old)||compare(yaw,yaw_old)||
  compare(pitch,pitch_old)||compare(roll,roll_old)||
  millis()-time>MAX_WAITING_TIME){
    time=millis();
    if(true){
    Serial.write(0xC8);
    Serial.write(0xC1);
    Serial.write(throttle+1);
    Serial.write(yaw+1);
    Serial.write(pitch+1);
    Serial.write(roll+1);
    uint16_t sum=throttle+yaw+pitch+roll+0xC8+0XC1+4;
    sum=sum&0x00FF;
    Serial.write(sum);
    Serial.write(0xC9);
    }
    else{
    Serial.print(throttle);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.println();
    }
    throttle_old=throttle;
    yaw_old=yaw;
    pitch_old=pitch;
    roll_old=roll;
    delay(2);
  }

  for(int i=0;i<4;i++){
    if(digitalRead(i+FIRST_BTN)==LOW){
      switch(i){
        case 0:
//            Serial.write(0xC8);
//            Serial.write(0xC4);
//            Serial.write(0xCA);
//            Serial.write(31);
//            Serial.write(64);
//            Serial.write(181);
//            Serial.write(0xC9);

    Serial.write(200);
    Serial.write(196);
    Serial.write(203);
    Serial.write(87);
    Serial.write(201);
        break;
        case 1:
           Serial.write(0xC8);
            Serial.write(0xC4);
            Serial.write(0xC4);
            Serial.write(0x50);
            Serial.write(0xC9);
        break;
        case 2:
          if(control_mode==1){
            control_mode=2;
          }
          else{
            control_mode=1;
          }
            Serial.write(0xC8);
            Serial.write(0xC4);
            Serial.write(0xC7);
            Serial.write(control_mode);
            Serial.write(0x53+control_mode);
            Serial.write(0xC9);
        break;
        case 3:
            Serial.write(0xC8);
            Serial.write(0xC4);
            Serial.write(0xC7);
            Serial.write(0x01);
            Serial.write(0x54);
            Serial.write(0xC9);
        break;
        default:
        break;
      }
      while(digitalRead(i+FIRST_BTN)==LOW);
    }
  }
}
