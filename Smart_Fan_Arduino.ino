#include <DHT.h>               // ไลบรารีสำหรับเซนเซอร์ DHT11
#include <Wire.h>              // ไลบรารีสำหรับการเชื่อมต่อ I2C
#include <LiquidCrystal_I2C.h> // ไลบรารีสำหรับการควบคุมหน้าจอ LCD แบบ I2C
#include <EEPROM.h>            // ไลบรารีสำหรับการใช้งานหน่วยความจำ EEPROM
#include <avr/sleep.h>         // ไลบรารีสำหรับการจัดการโหมด Sleep ใน AVR
#include <avr/interrupt.h>     // ไลบรารีสำหรับการจัดการ Interrupt ใน AVR

#define DHTPIN A3           // ขาที่เชื่อมต่อกับ DHT11
#define DHTTYPE DHT11       // เซ็นเซอร์ DHT11
#define BUTTONPUSH_PIN 5    // ปุ่มสำหรับเปิด/ปิดพัดลมในโหมด 1 และโหมด 4
#define MODE_BUTTON_PIN 2   // ปุ่มสำหรับสลับโหมด (ใช้ pin 2 เพราะรองรับ interrupt)
#define TRIG_PIN 6          // ขา Trigger ของ ultrasonic sensor
#define ECHO_PIN 7          // ขา Echo ของ ultrasonic sensor
#define Up_BUTTONPUSH_PIN 13 // ปุ่มสำหรับเพิ่มเวลาใน mode 4
#define Down_BUTTONPUSH_PIN 12 // ปุ่มสำหรับลดเวลาใน mode 4
#define interruptBlynk 3 // สำหรับเปลี่ยน Mode ใน Blynk App (ใช้ pin 3 เพราะรองรับ interrupt)

// สร้างอ็อบเจ็กต์ dht เพื่อใช้เซนเซอร์ DHT 
DHT dht(DHTPIN, DHTTYPE); //โดยระบุขา (DHTPIN) และประเภทของเซนเซอร์ (DHTTYPE)

// Pins for motor driver (L298N)
const int enaPin = 11;
const int in1Pin = 10;
const int in2Pin = 9;

//ตั้งค่าให้กับ LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // ที่อยู่ของ I2C และขนาดของจอ LCD

volatile int State_mode = 1; // ทำให้เป็น volatile เพราะจะใช้ใน ISR
int fanSpeed = 0;   //ค่าเริ่มต้นของความเร็วพัดลม
long distance;      //กำหนดตัวแปรเก็บค่าระยะทาง

//สำหรับทำ debounce และ delay
unsigned long previousMillis = 0;       // ตัวแปรสำหรับหน่วงเวลา
const long interval = 300;              // หน่วงการแสดงผลหน้าจอทุก 300ms
unsigned long lastInterruptTime = 0;    // ตัวแปรเก็บเวลาของ interrupt ล่าสุด
const long debounceDelay = 700;         // หน่วงเวลา 700ms สำหรับ debounce
unsigned long lastButtonPushTime = 0;   // ตัวแปรเก็บเวลาของการกดปุ่มล่าสุด
const long buttonDebounceDelay = 100;   // หน่วงเวลา 100ms สำหรับ debounce
int StateSpeed = 0;                     //กำหนด state ของ mode 1
const long holdDelay = 1000;            // หน่วงเวลา 1000 มิลลิวินาทีสำหรับการเพิ่มหรือลบ
unsigned long lastFanUsageTime = 0;     // ตัวแปรเพื่อเก็บเวลาที่พัดลมถูกปิดหรือเปลี่ยนความเร็ว
unsigned long fanStartTime = 0;         // ตัวแปรเก็บเวลาที่เริ่มการทำงานของพัดลมในโหมด 4
unsigned long fanRunTime = 60000;       // ตั้งเวลาให้พัดลมทำงานเป็นเวลา 1 นาที (60000 มิลลิวินาที)

//เช็คสถานะของเเต่ละโหมด
bool lastButtonState = HIGH;        // ตัวแปรเก็บสถานะปุ่มก่อนหน้า
bool lastButtonAddTime = HIGH;      // ตัวแปรเก็บสถานะปุ่มก่อนหน้า
bool lastButtonReduceTime = HIGH;   // ตัวแปรเก็บสถานะปุ่มก่อนหน้า
bool fanRunning = false;            // ตัวแปรเก็บสถานะการทำงานของพัดลมในโหมด 4
bool resetFlag = false;             // สถานะรีเซ็ตเริ่มต้นเป็น false
bool checkSleep = false;            // สถานะของ Sleep mode
bool StateMode3 = false;            // สถานะ เปิด - ปิด distance
int StateSpeed = 0;                 //กำหนด state ของ mode 1

//ตัวแปรสำหรับแปลงค่าเวลา
unsigned int hours;
unsigned int minutes;
unsigned int seconds;

// ฟังก์ชันสำหรับอ่านระยะห่างจาก ultrasonic sensor
long readUltrasonicDistance() {
  long distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1; // คำนวณระยะทางในหน่วยเซนติเมตร
  delay(200);
  return distance;
}

// ฟังก์ชันสำหรับแปลงเวลาเป็นหน่วย h:m:s
void displayTime(unsigned long timeRemaining) {
  hours = timeRemaining / 3600000;        // คำนวณชั่วโมง
  minutes = (timeRemaining % 3600000) / 60000; // คำนวณนาที
  seconds = (timeRemaining % 60000) / 1000;    // คำนวณวินาที
}

void setup() {
  Serial.begin(9600); 

  dht.begin();                    // เริ่มต้นการทำงานของ DHT11                  
  
  pinMode(enaPin, OUTPUT);         
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(BUTTONPUSH_PIN, INPUT_PULLUP); // กำหนดขาปุ่มเป็น input พร้อม pull-up
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP); // กำหนดขาปุ่ม mode เป็น input พร้อม pull-up
  pinMode(Up_BUTTONPUSH_PIN, INPUT_PULLUP);
  pinMode(Down_BUTTONPUSH_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);       // กำหนดขา TRIG_PIN เป็น output
  pinMode(ECHO_PIN, INPUT);        // กำหนดขา ECHO_PIN เป็น input
  pinMode(interruptBlynk, INPUT_PULLUP); // เพิ่มการตั้งค่า INPUT_PULLUP สำหรับ interruptBlynk

  lcd.init();                      // เริ่มต้นการทำงานของจอ LCD
  lcd.backlight();                 // เปิด backlight ของจอ LCD                 
  lcd.setCursor(4, 0);
  lcd.print("Smart Fan");
  lcd.setCursor(3, 1);
  lcd.print("by Stardust");
  delay(2000);
  lcd.clear();

  // Animation พัดลมหมุนและโหลดหลอด
  char Frames[] = {'|', '/', '-', '\\'}; // แอนิเมชันพัดลม
  int Index = 0;
  int loadLength = 12;  // ความยาวของหลอดโหลด

  for (int i = 0; i < 50; i++) {  // แสดงแอนิเมชัน 50 รอบ
    // แสดงแอนิเมชันพัดลมหมุนทางซ้ายสุด
    lcd.setCursor(0, 0);
    lcd.print(Frames[Index]);  // พิมพ์สัญลักษณ์พัดลม
    Index = (Index + 1) % 4;   // เปลี่ยนไปที่สัญลักษณ์พัดลมถัดไป

    // แสดงแถบหลอดโหลดที่ค่อย ๆ เพิ่มทีละขั้น
    lcd.setCursor(2, 0);             // กำหนดตำแหน่งแถบหลอด
    lcd.print("[");                  // เริ่มต้นหลอดด้วย "["

    int progress = (i * loadLength) / 50;  // คำนวณความคืบหน้า

    for (int j = 0; j < loadLength; j++) {
      if (j < progress) {
        lcd.print("=");  // แสดงหลอดที่โหลด
      } else {
        lcd.print(" ");  // พื้นที่ว่างในหลอดที่ยังไม่เติม
      }
    }

    lcd.print("]");      // จบหลอดด้วย "]"

    // คำนวณเปอร์เซ็นต์
    int percentage = (i * 100) / 50;

    // แสดงผลเปอร์เซ็นต์ที่ด้านล่างของหลอดโหลด
    lcd.setCursor(0, 1);  // กำหนดตำแหน่งสำหรับแสดงเปอร์เซ็นต์
    lcd.print("loadding: ");
    lcd.print(percentage);
    lcd.print("% ");

    delay(200); // หน่วงเวลาให้แอนิเมชันดูสมูท
  }

  lcd.clear();  // เคลียร์จอ
  lastFanUsageTime = millis();

  // โหลดค่า mode จาก EEPROM
  State_mode = EEPROM.read(0);
  if (State_mode < 1 || State_mode > 4) { // ตรวจสอบว่าโหมดอยู่ในช่วงที่ถูกต้อง
    State_mode = 1;                 // ถ้าไม่ถูกต้อง ให้ตั้งค่าเริ่มต้นเป็น mode 1
  }

  // ตั้งค่า Interrupt แบบเซ็ตบิต
  cli(); // ปิดการ Interrupt ก่อนทำการตั้งค่า

  // ตั้งค่า INT0 (พิน 2) สำหรับ FALLING edge
  EICRA |= (1 << ISC01); // ISC01 = 1, ISC00 = 0 สำหรับ FALLING edge
  EICRA &= ~(1 << ISC00);
  
  // ตั้งค่า INT1 (พิน 3) สำหรับ RISING edge
  EICRA |= (1 << ISC11) | (1 << ISC10); // ISC11 = 1, ISC10 = 1 สำหรับ RISING edge
  
  // เปิดใช้งาน INT0 และ INT1
  EIMSK |= (1 << INT0) | (1 << INT1);
  
  sei(); // เปิดการ Interrupt หลังจากตั้งค่าเสร็จ

}

void loop() {
  float temperature;
  
  bool buttonState = digitalRead(BUTTONPUSH_PIN); // อ่านสถานะปุ่มเปิด/ปิดพัดลมในโหมด 1 ,2, 3 และโหมด 4
  bool buttonAddTime = digitalRead(Up_BUTTONPUSH_PIN); //อ่านค่าสถานะจากปุ่มเพิ่มเวลา
  bool buttonReduceTime = digitalRead(Down_BUTTONPUSH_PIN); //อ่านค่าสถานะจากปุ่มลดเวลา

  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');  // รับข้อมูลจาก NodeMCU
    receivedData.trim();  // ลบช่องว่างออก

    if (receivedData.startsWith("Mode: ")) {
      State_mode = receivedData.substring(6).toInt();  // แยกข้อมูลโหมด
      Serial.print("Received Mode: ");
      Serial.println(State_mode);

      // เก็บค่าโหมดลงใน EEPROM
      EEPROM.update(0, State_mode);
      Serial.println("Mode saved to EEPROM");
    }

    if (receivedData.startsWith("Manual Mode: ")) {
      buttonState = receivedData.substring(13).toInt();  // แยกข้อมูลโหมด
      Serial.print("Received Manaul: ");
      Serial.println(buttonState);
      Serial.println("Manaul saved state");
    }

    if (receivedData.startsWith("UP Mode: ")) {
      buttonAddTime = receivedData.substring(9).toInt();  // แยกข้อมูลโหมด
      Serial.print("Received Up Time: ");
      Serial.println(buttonAddTime);
      Serial.println("UpTime saved state");
    }

    if (receivedData.startsWith("DOWN Mode: ")) {
      buttonReduceTime = receivedData.substring(11).toInt();  // แยกข้อมูลโหมด
      Serial.print("Received Down Time: ");
      Serial.println(buttonReduceTime);
      Serial.println("DownTime saved state");
    }
  }

  // เช็คว่าอยู่ใน Mode 1, 2, 3, 4 และพัดลมถูกปิด
  if (State_mode == 1 || State_mode == 2 || State_mode == 3 || State_mode == 4) {
    if (fanSpeed > 0) {
      // ถ้า fanSpeed มากกว่า 0 (พัดลมทำงาน) ให้รีเซ็ต lastFanUsageTime
      lastFanUsageTime = millis();
    } else {
      // ถ้าพัดลมปิดและเวลาผ่านไปมากกว่า 15 วินาทีให้เข้าสู่ sleep mode
      if (millis() - lastFanUsageTime > 15000) {
        enterSleep(); // เข้าสู่ sleep mode
      }
    }
  } else {
    lastFanUsageTime = millis();
  }

  if(StateMode3 == true){
    distance = readUltrasonicDistance();
  }
  else{
    distance = 0;
    delay(200);
  }

  if (distance < 50) { // ถ้าระยะห่างน้อยกว่า 50 ซม.
    if (State_mode == 1) {
      // Mode 1: ควบคุมพัดลมด้วยปุ่ม
      if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        lastButtonPushTime = millis(); // อัปเดตเวลาของการกดปุ่มล่าสุด

        if (StateSpeed < 3) {
          StateSpeed += 1;
        } else {
          StateSpeed = 0;
        }
      }
      lastButtonState = buttonState; // อัปเดตสถานะปุ่มก่อนหน้า
      
      switch (StateSpeed) {
        case 0:
          fanSpeed = 0; // ปิดพัดลม
          break;
        case 1:
          fanSpeed = 150; // เปิดพัดลมระดับ 1
          break;
        case 2:
          fanSpeed = 200; // เปิดพัดลมระดับ 2
          break;
        case 3:
          fanSpeed = 255; // เปิดพัดลมด้วยความเร็วสูงสุด
          break;
      }

    } else if (State_mode == 2) {
      // Mode 2: ควบคุมความเร็วพัดลมตามอุณหภูมิ
      temperature = dht.readTemperature(); // อ่านอุณหภูมิจากเซ็นเซอร์ DHT11
      Serial.print("temperature: ");
      Serial.println(temperature);

      if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        lastButtonPushTime = millis(); // อัปเดตเวลาของการกดปุ่มล่าสุด

        if (StateSpeed < 1) {
          StateSpeed += 1;
        } else {
          StateSpeed = 0;
        }
      }
      lastButtonState = buttonState; // อัปเดตสถานะปุ่มก่อนหน้า

      if(StateSpeed == 1){
        if (temperature < 25) {
          fanSpeed = 0;
        } else if (temperature >= 25 && temperature < 30) {
          fanSpeed = 128;
        } else if (temperature >= 30 && temperature < 35) {
          fanSpeed = 255;
        } else {
          fanSpeed = 255;
        }
      } else if(StateSpeed == 0){
        fanSpeed = 0;
      }
      
    } else if (State_mode == 3) {
      // Mode 3: ควบคุมพัดลมตามระยะห่างที่ตรวจพบจาก ultrasonic sensor

      if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        lastButtonPushTime = millis(); // อัปเดตเวลาของการกดปุ่มล่าสุด

        if (StateMode3 == false) {
          StateMode3 = true;
        } else {
          StateMode3 = false;
        }
      }
      lastButtonState = buttonState; // อัปเดตสถานะปุ่มก่อนหน้า

    } else if (State_mode == 4) {
      // Mode 4: ควบคุมพัดลมตามเวลาที่ตั้งค่า
      displayTime(fanRunTime);
      if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        lastButtonPushTime = millis(); // อัปเดตเวลาของการกดปุ่มล่าสุด
        fanStartTime = millis();       // ตั้งเวลาที่เริ่มทำงานพัดลม

        if (fanRunning) {
          // ถ้าพัดลมกำลังทำงาน ให้รีเซ็ตและหยุดพัดลม
          fanRunning = false;
          resetFlag = true;   // เปิดสถานะรีเซ็ต
          fanSpeed = 0;  // ปิดพัดลม
        } else {
          // ถ้าพัดลมไม่ได้ทำงาน ให้เริ่มทำงาน
          if (resetFlag) {
            fanStartTime = millis();   // รีเซ็ตเวลาเริ่มต้นใหม่
            resetFlag = false;  // ปิดสถานะรีเซ็ต
          }
          fanRunning = true;  // เปิดพัดลม
        }
      }
      lastButtonState = buttonState; // อัปเดตสถานะปุ่มก่อนหน้า

      if (buttonAddTime == LOW && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        if (lastButtonAddTime == HIGH) {
          // เมื่อปุ่มถูกกดครั้งแรก
          lastButtonPushTime = millis();
          fanRunTime += 1800000; // เพิ่มเวลา 30 นาที
        } else if (millis() - lastButtonPushTime > holdDelay) {
          // เมื่อปุ่มถูกกดค้างไว้และครบหน่วงเวลา
          fanRunTime += 1800000; // เพิ่มเวลา 30 นาที
          lastButtonPushTime = millis(); // รีเซ็ตเวลาหน่วง
        }
        if(fanRunTime > 86400000 && fanRunTime >= 0) fanRunTime = 0; // จำกัดไม่ให้เกิน 24 ชั่วโมง
      }
      lastButtonAddTime = buttonAddTime; // อัปเดตสถานะปุ่มเพิ่มเวลา

      // ตรวจสอบการกดปุ่มลดเวลา
      if (buttonReduceTime == LOW && (millis() - lastButtonPushTime > buttonDebounceDelay)) {
        if (lastButtonReduceTime == HIGH) {
          // เมื่อปุ่มถูกกดครั้งแรก
          lastButtonPushTime = millis();
          fanRunTime -= 1800000; // ลดเวลา 30 นาที
        } else if (millis() - lastButtonPushTime > holdDelay) {
          // เมื่อปุ่มถูกกดค้างไว้และครบหน่วงเวลา
          fanRunTime -= 1800000; // ลดเวลา 30 นาที
          lastButtonPushTime = millis(); // รีเซ็ตเวลาหน่วง
        }
        if(fanRunTime >= 86400000 && fanRunTime > 0) fanRunTime = 86400000; // จำกัดไม่ให้ต่ำกว่า 0
      }
      lastButtonReduceTime = buttonReduceTime; // อัปเดตสถานะปุ่มลดเวลา

      if (fanRunning) {
        unsigned long elapsedTime = millis() - fanStartTime;
        unsigned long timeRemaining = fanRunTime - elapsedTime;

        if (elapsedTime <= fanRunTime) {
          fanSpeed = 255; // เปิดพัดลมที่ความเร็วสูงสุด
          displayTime(timeRemaining); // แสดงเวลาที่เหลือในหน่วย h:m:s
        } else {
          fanSpeed = 0;   // ปิดพัดลมเมื่อครบเวลา
          fanRunning = false; // หยุดพัดลมจนกว่าจะกดปุ่มอีกครั้ง
        }
      }

    }
  } else {
    fanSpeed = 0; // ปิดพัดลม
  }
  setFan(fanSpeed);

  // ใช้ millis() เพื่อลดการกระพริบของหน้าจอ
  if (millis() - previousMillis >= interval) {
    previousMillis = millis(); // อัปเดตเวลา

    if(distance < 50){
      if (State_mode == 1) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Speed: ");
        lcd.print(map(fanSpeed, 0, 255, 0, 100));
        lcd.print("% ");
        lcd.print(StateSpeed > 0 ? "On" : "Off");

        lcd.setCursor(0, 1);
        lcd.print("Mode:1 - Manual");
        
      } else if (State_mode == 2) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T: ");
        lcd.print(temperature);
        lcd.print("C");
        lcd.setCursor(10, 0);
        lcd.print("S:");
        lcd.print(map(fanSpeed, 0, 255, 0, 100));
        lcd.print("%");

        lcd.setCursor(0, 1);
        lcd.print("Mode:2 - Temp(C)");

      } else if (State_mode == 3) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(StateMode3 == true ? "Distance:" : "fan : Off ");
        lcd.print(distance);
        lcd.print(" cm");
        lcd.setCursor(0, 1);
        lcd.print("Mode:3 - Auto");

      } else if (State_mode == 4){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Time: ");
        lcd.setCursor(6, 0);
        lcd.print(hours);
        lcd.print(":");
        if (minutes < 10) lcd.print("0"); // แสดงผลเป็น 2 หลัก
        lcd.print(minutes);
        lcd.print(":");
        if (seconds < 10) lcd.print("0"); // แสดงผลเป็น 2 หลัก
        lcd.print(seconds);
        lcd.setCursor(0, 1);
        lcd.print("Mode:4 - Timer");
      }
    }
    else{
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Distance:");
      lcd.print(distance);
      lcd.print(" cm");
      lcd.setCursor(0, 1);
      lcd.print("Fan: - off");
    }
  }
}

void setFan(int speed) {
  if (speed == 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enaPin, 0);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enaPin, speed);
  }
}

// ISR สำหรับ INT0 (MODE_BUTTON_PIN)
ISR(INT0_vect) {
  unsigned long currentInterruptTime = millis(); // อ่านเวลาปัจจุบัน
  // ตรวจสอบว่าผ่านเวลา debounce ไปแล้วหรือยัง
  if (currentInterruptTime - lastInterruptTime > debounceDelay) {
    lastFanUsageTime = millis();
    StateSpeed = 0;
    fanSpeed = 0;

    if(checkSleep == false){
      if(State_mode > 0 && State_mode < 4){
        State_mode += 1;  // สลับโหมดระหว่าง 1 ถึง 4
        EEPROM.update(0, State_mode);  // บันทึกค่าโหมดลงใน EEPROM
        Serial.print("Mode: ");
        Serial.println(State_mode);
      } else {
        State_mode = 1;
        EEPROM.update(0, State_mode);  // บันทึกค่าโหมดลงใน EEPROM
        Serial.print("Mode: ");
        Serial.println(State_mode);
      }

    } 
    else if (checkSleep == true){
      int Lastmode = State_mode;  // สลับโหมดระหว่าง 1 ถึง 4
      EEPROM.write(0, Lastmode);  // บันทึกค่าโหมดลงใน EEPROM
      Serial.print("Mode: ");
      Serial.println(State_mode);
      sleep_disable();
      checkSleep = false;
    }
    lastInterruptTime = currentInterruptTime; // อัปเดตเวลาของ interrupt ล่าสุด
  }
}

// ISR สำหรับ INT1 (interruptBlynk)
ISR(INT1_vect) {
  unsigned long currentInterruptTime = millis(); // อ่านเวลาปัจจุบัน
  // ตรวจสอบว่าผ่านเวลา debounce ไปแล้วหรือยัง
  if (currentInterruptTime - lastInterruptTime > debounceDelay) {
    lastFanUsageTime = millis();
    StateSpeed = 0;
    fanSpeed = 0;

    if(checkSleep == false){
      if(State_mode > 0 && State_mode < 4){
        State_mode += 1;  // สลับโหมดระหว่าง 1 ถึง 4
        EEPROM.update(0, State_mode);  // บันทึกค่าโหมดลงใน EEPROM
        Serial.print("Mode: ");
        Serial.println(State_mode);
      } else {
        State_mode = 1;
        EEPROM.update(0, State_mode);  // บันทึกค่าโหมดลงใน EEPROM
        Serial.print("Mode: ");
        Serial.println(State_mode);
      }

    } 
    else if (checkSleep == true){
      int Lastmode = State_mode;  // สลับโหมดระหว่าง 1 ถึง 4
      EEPROM.write(0, Lastmode);  // บันทึกค่าโหมดลงใน EEPROM
      Serial.print("Mode: ");
      Serial.println(State_mode);
      sleep_disable();
      checkSleep = false;
    }
    lastInterruptTime = currentInterruptTime; // อัปเดตเวลาของ interrupt ล่าสุด
  }
}

// ฟังก์ชันสำหรับตั้งค่า sleep mode
void enterSleep() {

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // ตั้งค่า sleep mode ให้เป็นโหมดประหยัดพลังงานสูงสุด (SLEEP_MODE_PWR_DOWN)
  sleep_enable();                       // เปิดการทำงานของ sleep mode
  checkSleep = true;                    // กำหนดตัวแปร checkSleep เป็น true เพื่อบอกว่าระบบกำลังเข้าสู่โหมด sleep
  lastFanUsageTime = millis();          // เก็บเวลาล่าสุดก่อนเข้าสู่ sleep mode เพื่อตรวจสอบการใช้งานของพัดลม
  
  lcd.clear();                          // ล้างหน้าจอ LCD

  lcd.setCursor(0, 0);                  // ตั้งตำแหน่ง cursor ของ LCD ไปที่แถว 0 คอลัมน์ 0
  lcd.print("Entering sleep mode");     // แสดงข้อความ "Entering sleep mode" บนหน้าจอ

  lcd.clear();                          // ล้างหน้าจอ LCD อีกครั้ง
  lcd.noBacklight();                    // ปิดแสงพื้นหลังของ LCD เพื่อประหยัดพลังงาน
  delay(1000);                          // หน่วงเวลา 1 วินาที

  sleep_mode();                         // เข้าสู่ sleep mode จริง ๆ

  sleep_disable();                      // หลังจากตื่นแล้ว ปิด sleep mode

  String loading = "Waking up...";      // ข้อความที่จะแสดงตอนระบบตื่น
  int loadIndex = 0;                    // ตัวแปรสำหรับควบคุมการแสดงข้อความทีละตัวอักษร

  lcd.backlight();                      // เปิดแสงพื้นหลังของ LCD เมื่อระบบตื่นขึ้นมา
  lcd.clear();                          // ล้างหน้าจอ LCD เพื่อเตรียมแสดงข้อความใหม่

  for (int i = 0; i < 18; i++) {        // วนลูปเพื่อแสดงข้อความ "Waking up..."
    lcd.setCursor(2, 0);                // ตั้ง cursor ไปที่ตำแหน่งแถว 0 คอลัมน์ 2
    lcd.print(loading.substring(0, loadIndex + 1));  // แสดงข้อความทีละตัวอักษร

    loadIndex = (loadIndex + 1) % (loading.length() + 1); // อัพเดทตัวเลข loadIndex เพื่อแสดงข้อความถัดไป

    delay(200);                         // หน่วงเวลา 0.2 วินาทีระหว่างการแสดงข้อความ

  }
}

