#include "DHT.h"
#define ERA_DEBUG
#define DHTPIN 22      // dht22
#define DHTTYPE DHT22  //thư viện này dùng cho nhiều loại dht nên chọn loại dht22
DHT dht(DHTPIN, DHTTYPE);

#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "ee1435ed-85fb-43bb-9da5-6991ca13bfaa"

#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const char ssid[] = "Wokwi-GUEST";
const char pass[] = "";

ERaTimer timer;

#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
#define SDA_PIN 25
#define SCL_PIN 26

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

#define LIGHT 34  //

#define RELAY_SO 33              //LED
#define RELAY_phunsuong_quat 32  //

#define buttonPin 18        //MODE
#define BUTTON_PINquat 19   //NN QUAT
#define BUTTON_PINled 21    //NN LED
#define BUTTON_PINmaybom 35  //NN mb
#define BUTTON_PINso 23     //NN so

#define ena 13  //chân cắm vào l298n để đk tốc độ
#define enaled1 12
#define enaled2 14

#define in1 4  // chân 4,5 cắm vào l298n luôn để đk chiều quay
#define in2 5

#define in3 2
#define in4 15

int mode;
int countt = 0;

bool currentStateA = true;  // Trạng thái hiện tại: true = A, false = B
bool lastButtonState = HIGH;

bool currentStateBquat = true;
bool lastButtonStatequat = LOW;
bool actionInProgress = false;
bool end_motor_active = true;
bool end_motor_active_auto = false;
bool end_motor_active_auto_2 = false;
bool motorActive_opened = false; 
bool motorActive_closed = false; 
bool flag_opened = true;
bool flag_closed = false;


bool currentStateBled = true;
bool lastButtonStateled = HIGH;

bool currentStateBmaybom = true;
bool lastButtonStatemaybom = HIGH;

bool currentStateBso = true;
bool lastButtonStateso = HIGH;

unsigned long previousMillis = 0;
unsigned long previousMillis_closed = 0;

bool ledState = LOW;
bool start = false;

void setup() {
  /* Setup debug console */
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  ERa.begin(ssid, pass);

  lcd.init();
  lcd.backlight();

  pinMode(ena, OUTPUT);
  pinMode(enaled1, OUTPUT);
  pinMode(enaled2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(LIGHT, INPUT);
  pinMode(RELAY_SO, OUTPUT);
  pinMode(RELAY_phunsuong_quat, OUTPUT);
  dht.begin();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(BUTTON_PINquat, INPUT_PULLUP);
  pinMode(BUTTON_PINled, INPUT_PULLUP);
  pinMode(BUTTON_PINmaybom, INPUT_PULLUP);
  pinMode(BUTTON_PINso, INPUT_PULLUP);


  digitalWrite(RELAY_SO, HIGH);              //
  digitalWrite(RELAY_phunsuong_quat, HIGH);  //
  analogWrite(ena, 0);                       //
  analogWrite(enaled1, 0);
  analogWrite(enaled2, 0);

  digitalWrite(in1, HIGH);  //
  digitalWrite(in2, LOW);   //
  digitalWrite(in3, LOW);   //
  digitalWrite(in4, LOW);   //

  ERa.virtualWrite(V1, 0);
  ERa.virtualWrite(V2, 0);
  ERa.virtualWrite(V3, 0);  // NN MODE
  ERa.virtualWrite(V4, 0);
  ERa.virtualWrite(V5, 0);
  ERa.virtualWrite(V6, 0);
  ERa.virtualWrite(V7, 0);
  ERa.virtualWrite(V8, 0);
  ERa.virtualWrite(V9, 0);
  ERa.virtualWrite(V10, 0);
  ERa.virtualWrite(V11, 0);
  ERa.virtualWrite(V12, 0);

  lcd.setCursor(0, 0);
  lcd.print("Mode:MANU  MOTOR:OFF");
  lcd.setCursor(0, 1);
  lcd.print("LED_1:OFF  LED_2:OFF");
  lcd.setCursor(0, 2);
  lcd.print("SUONG:OFF");
  lcd.setCursor(0, 3);
  lcd.print("SO:OFF");
}

ERA_WRITE(V3) {
  int p2 = param.getInt();  //LẤY GIÁ TRỊ CỦA CHÂN ẢO
  if (p2 > 0 && !actionInProgress && end_motor_active) {
    currentStateA = true;
  } else {
    currentStateA = false;
  }
}

ERA_WRITE(V4) {
  int p2 = param.getInt();  //LẤY GIÁ TRỊ CỦA CHÂN ẢO

  if (p2 > 0) {
    countt++;
    if (countt > 2) {
      countt = 1;
    }
    previousMillis = millis();
  }
  if (p2 <= 0 && actionInProgress) {
    ERa.virtualWrite(V4, 1);
    ERa.virtualWrite(V8, 255);
  }
}

ERA_WRITE(V5) {
  int p2 = param.getInt();  //LẤY GIÁ TRỊ CỦA CHÂN ẢO
  if (p2 > 0) {
    currentStateBled = true;
  } else {
    currentStateBled = false;
  }
}

ERA_WRITE(V6) {
  int p2 = param.getInt();  //LẤY GIÁ TRỊ CỦA CHÂN ẢO
  if (p2 > 0) {
    currentStateBmaybom = true;
  } else {
    currentStateBmaybom = false;
  }
}

ERA_WRITE(V7) {
  int p2 = param.getInt();  //LẤY GIÁ TRỊ CỦA CHÂN ẢO
  if (p2 > 0) {
    currentStateBso = true;
  } else {
    currentStateBso = false;
  }
}

void processStateA() {
  //----------------------------------------------------------
  unsigned long currentMillis = millis();
  

  if (analogRead(LIGHT) >= 3500) {
    analogWrite(enaled1, 255);
    analogWrite(enaled2, 255);
    ERa.virtualWrite(V5, 1);
    ERa.virtualWrite(V9, 255);
    ERa.virtualWrite(V10, 255);
    lcd.setCursor(0, 1);
    lcd.print("LED_1:ON   LED_2:ON ");

    if (flag_opened){
      if (!end_motor_active_auto){
        if (!motorActive_opened) {
          previousMillis = currentMillis; 
          motorActive_opened = true;
        }

        if ((currentMillis - previousMillis) < 10000) {
          analogWrite(ena, 255);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          ERa.virtualWrite(V4, 1);
          ERa.virtualWrite(V8, 255);  
          flag_closed = false;
          lcd.setCursor(11, 0);
          lcd.print("MOTOR:ON ");
          Serial.println("__________________________________________1");
        } else {
          ERa.virtualWrite(V4, 0);
          ERa.virtualWrite(V8, 0);
          analogWrite(ena, 0);
          motorActive_opened = false;
          end_motor_active_auto = true;
          end_motor_active_auto_2 = false;
          flag_closed = true;
          lcd.setCursor(11, 0);
          lcd.print("MOTOR:OFF ");
        }
      }
    }
    
  
  } else if (analogRead(LIGHT) <= 2000){
    analogWrite(enaled1, 0);
    analogWrite(enaled2, 0);
    ERa.virtualWrite(V5, 0);
    ERa.virtualWrite(V9, 0);
    ERa.virtualWrite(V10, 0);
    lcd.setCursor(0, 1);
    lcd.print("LED_1:OFF  LED_2:OFF");

    if (flag_closed){
      if (!end_motor_active_auto_2){
        if (!motorActive_closed) {
          previousMillis_closed = currentMillis; 
          motorActive_closed = true;
        }

        if ((currentMillis - previousMillis_closed) < 10000) {
          analogWrite(ena, 255);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          ERa.virtualWrite(V4, 1);
          ERa.virtualWrite(V8, 255);  
          flag_opened = false;
          lcd.setCursor(11, 0);
          lcd.print("MOTOR:ON ");
          Serial.println("__________________________________________2");
        } else {
          ERa.virtualWrite(V4, 0);
          ERa.virtualWrite(V8, 0);
          analogWrite(ena, 0);
          motorActive_closed = false;
          end_motor_active_auto_2 = true;
          end_motor_active_auto = false;
          flag_opened = true;
          lcd.setCursor(11, 0);
          lcd.print("MOTOR:OFF ");
        }
      }
    }
  }
  //----------------------------------------------------------
  //dht22-------------------------------------------------
  //----------------------------------------------------------
  lcd.setCursor(0, 2);
  if (dht.readHumidity() <= 70) {  //nhiệt độ >30 là thì bật sò
    digitalWrite(RELAY_phunsuong_quat, HIGH);
    ERa.virtualWrite(V6, 1);
    ERa.virtualWrite(V11, 1);
    lcd.print("SUONG:ON ");
  } else if (dht.readHumidity() >= 90){
    digitalWrite(RELAY_phunsuong_quat, LOW);
    ERa.virtualWrite(V6, 0);
    ERa.virtualWrite(V11, 0);
    lcd.print("SUONG:OFF");
  }
  //----------------------------------------------------------
  lcd.setCursor(0, 3);
  // lcd.print("SUONG:OFF      ");
  if (dht.readTemperature() > 30) {  //nhiệt độ >30 là thì bật sò
    digitalWrite(RELAY_SO, HIGH);
    ERa.virtualWrite(V7, 1);
    ERa.virtualWrite(V12, 1);
    lcd.print("SO:ON ");

  } else if (dht.readTemperature() <= 25){
    digitalWrite(RELAY_SO, LOW);
    ERa.virtualWrite(V7, 0);
    ERa.virtualWrite(V12, 0);
    lcd.print("SO:OFF");
  }
  
  delay(200);
}

void loop() {
  ERa.run();
  timer.run();
  
  ERa.virtualWrite(V0, dht.readHumidity());
  ERa.virtualWrite(V1, analogRead(LIGHT));
  ERa.virtualWrite(V2, dht.readTemperature());

  char humStr[6];
  dtostrf(dht.readHumidity(), 4, 1, humStr);
  lcd.setCursor(11, 2);
  lcd.print("Humi:");
  lcd.setCursor(16, 2);
  lcd.print(humStr);

  char tempStr[6];
  dtostrf(dht.readTemperature(), 4, 1, tempStr);
  lcd.setCursor(11, 3);
  lcd.print("Temp:");
  lcd.setCursor(16, 3);
  lcd.print(tempStr);
  
  bool buttonState = !digitalRead(buttonPin);
  bool buttonStatequat = !digitalRead(BUTTON_PINquat);
  bool buttonStateled = !digitalRead(BUTTON_PINled);
  bool buttonStatemaybom = !digitalRead(BUTTON_PINmaybom);
  bool buttonStateso = !digitalRead(BUTTON_PINso);

  
  //----------------------------------------------------------

  if (buttonState == LOW && lastButtonState == HIGH) {
    currentStateA = !currentStateA;
    Serial.println(currentStateA ? "A" : "B");
  }
  if (currentStateA) {
    mode = 1;
    ERa.virtualWrite(V3, 1);
    delay(100);
    processStateA();
    lcd.setCursor(0, 0);
    lcd.print("Mode:Auto ");

  } else {
    mode = 0;
    ERa.virtualWrite(V3, 0);
    lcd.setCursor(0, 0);
    lcd.print("Mode:Manu ");
    //----------------------------------------------------------
    if (buttonStatequat == HIGH && !actionInProgress) {
      countt++;
      if (countt > 2) {
        countt = 1;
      }
      previousMillis = millis();
      actionInProgress = true;
    }
    
    if (countt == 1) {
      unsigned long currentMillis = millis();
      Serial.println(previousMillis);
      Serial.println(currentMillis);
      if ((currentMillis - previousMillis) < 10000) {
        analogWrite(ena, 255);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        ERa.virtualWrite(V4, 1);
        ERa.virtualWrite(V8, 255);
        lcd.setCursor(11, 0);
        lcd.print("MOTOR:ON ");
        end_motor_active = false;
        Serial.println("__________________________________________1");
      } else if ((currentMillis - previousMillis) >= 10000) {
        ERa.virtualWrite(V4, 0);
        ERa.virtualWrite(V8, 0);
        analogWrite(ena, 0);
        actionInProgress = false;
        end_motor_active = false;
        lcd.setCursor(11, 0);
        lcd.print("MOTOR:OFF");
      }
    } else if (countt == 2) {
      unsigned long currentMillis = millis();
      Serial.println(previousMillis);
      Serial.println(currentMillis);
      if ((currentMillis - previousMillis) < 10000) {
        analogWrite(ena, 255);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        ERa.virtualWrite(V4, 1);
        ERa.virtualWrite(V8, 255);
        lcd.setCursor(11, 0);
        lcd.print("MOTOR:ON ");
        Serial.println("__________________________________________2");
      } else if ((currentMillis - previousMillis) >= 10000) {
        ERa.virtualWrite(V4, 0);
        ERa.virtualWrite(V8, 0);
        analogWrite(ena, 0);
        actionInProgress = false;
        end_motor_active = false;
        lcd.setCursor(11, 0);
        lcd.print("MOTOR:OFF");
        end_motor_active = true;
      }
    }
    //----------------------------------------------------------


    //------------------------------------------------------------------
    if (buttonStateled == LOW && lastButtonStateled == HIGH) {
      currentStateBled = !currentStateBled;
    }
    lcd.setCursor(0, 1);
    if (currentStateBled) {
      Serial.println("A2");
      analogWrite(enaled1, 255);
      analogWrite(enaled2, 255);
      ERa.virtualWrite(V5, 1);
      ERa.virtualWrite(V9, 255);
      ERa.virtualWrite(V10, 255);
      lcd.print("LED_1:ON   LED_2:ON ");
    } else {
      Serial.println("b2");
      analogWrite(enaled1, 0);
      analogWrite(enaled2, 0);
      ERa.virtualWrite(V5, 0);
      ERa.virtualWrite(V9, 0);
      ERa.virtualWrite(V10, 0);
      lcd.print("LED_1:OFF  LED_2:OFF");
    }
    lastButtonStateled = buttonStateled;
    //------------------------------------------------------------------
    if (buttonStatemaybom == LOW && lastButtonStatemaybom == HIGH) {
      currentStateBmaybom = !currentStateBmaybom;
    }
    lcd.setCursor(0, 2);
    if (currentStateBmaybom) {
      Serial.println("A3");
      digitalWrite(RELAY_phunsuong_quat, HIGH);
      ERa.virtualWrite(V6, 1);
      ERa.virtualWrite(V11, 1);
      lcd.print("SUONG:ON ");
    } else {
      Serial.println("b3");
      digitalWrite(RELAY_phunsuong_quat, LOW);
      ERa.virtualWrite(V6, 0);
      ERa.virtualWrite(V11, 0);
      lcd.print("SUONG:OFF");
    }
    lastButtonStatemaybom = buttonStatemaybom;
    //------------------------------------------------------------------
    if (buttonStateso == LOW && lastButtonStateso == HIGH) {
      currentStateBso = !currentStateBso;
    }
    lcd.setCursor(0, 3);
    if (currentStateBso) {

      Serial.println("A4");
      digitalWrite(RELAY_SO, HIGH);
      ERa.virtualWrite(V7, 1);
      ERa.virtualWrite(V12, 1);
      lcd.print("SO:ON ");
    } else {
      Serial.println("b4");
      digitalWrite(RELAY_SO, LOW);
      ERa.virtualWrite(V7, 0);
      ERa.virtualWrite(V12, 0);
      lcd.print("SO:OFF");
    }
    lastButtonStateso = buttonStateso;
  }
  lastButtonState = buttonState;
  //------------------------------------------------------------------
  delay(200);
}
