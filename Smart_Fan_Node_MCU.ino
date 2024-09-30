#define BLYNK_TEMPLATE_ID "TMPL6dSvjeCw5"  ///ระบุ ID ของTemplaye ที่ใช้ของ Bltnk
#define BLYNK_TEMPLATE_NAME "SmartFan"     //ระบุชื่อของ Blynk
#define BLYNK_AUTH_TOKEN "qPuQaUNvI844erMPpU4LnY6pOfitXunO" // Token ที่ระบุการใช้ Blynk

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// ข้อมูลการเชื่อมต่อ WiFi
char ssid[] = "NUTTY"; 
char pass[] = "asd12345";

// ขาเสมือนใน Blynk
#define MODE_VPIN V0
#define Manul_VPIN V1
#define READM_PIN V2
#define UP_VPIN V3
#define DOWN_VPIN V4
#define MODE_PIN D0 

// ตัวแปรสถานะ
bool ButtonMode = LOW; //สถานะการกดปุ่มโหมด
int mode = 1; //การเริ่มนับโหมด
bool previousMode = LOW; //สถานะของโหมดก่อนหน้า
bool ManaulButton = LOW; //สถานะการกดปุ่มควบคุม
int manaul = 0; //การเริ่มนับ manaul
bool previosManaul = LOW; //สถานะของปุ่มควบคุมก่อนหน้า
float Temp; //เก็บอุณหภูมิ
unsigned long previousMillis = 0; //สร้างการนับเวลาก่อนหน้า
const long interval = 3000; // ตั้งค่าเวลาตรวจสอบอุณหภูมิทุก 3 วินาที
bool UPButton = LOW; //ตั่งค่าปุ่มกด
bool DOWNButton = LOW; //ตั่งค่าปุ่มกด
bool previosUP = LOW; //ตั่งค่าปุ่มกด
bool previosDOWN = LOW; //ตั่งค่าปุ่มกด
int up = 0; //การเริ่มนับโหมด
int down = 0; //การเริ่มนับโหมด

// การตั้งค่า SoftwareSerial
SoftwareSerial mySerial(D5, D6); // RX=D5, TX เชื่อมต่อกับ Arduino UNO
SoftwareSerial MCUSerial(D2, D3); // RX =D2 , TX =D3 เชื่อมต่อกับ Odroid C4

// การตั้งค่า Servo
Servo myServo;
const int servoPin = D1;  //กำหนดขาของ Servo motor
int currentAngle = 90; //ระบุตำแหน่งเริ่มต้นของ servo เป็น 90 องศา
int targetAngleX = 90; //ระบุตำแหน่งเริ่มต้นของ faceX เป็น 90 องศา
int targetAngleY = 90; //ระบุตำแหน่งเริ่มต้นของ faceY เป็น 90 องศา
const int tolerance = 5; //เป็นการป้องกันการคาดเคลื่อน 5 องศา
bool isRotating = false; //ตรวจสอบการหมุน
unsigned long lastReceivedTime = 0;  //ใช้ในการตรวจสอบค่าสุดท้ายที่ได้รับมาเพื่อดูว่า servo จะหยุดหรือไม่
const unsigned long timeoutDuration = 5000;
bool servoActive = true; // ตัวแปรเพื่อระบุว่าเซอร์โวมอเตอร์ยังคงทำงานอยู่หรือไม่
int lastKnownAngle = 90; // ตัวแปรเพื่อเก็บตำแหน่งสุดท้ายที่ทราบของเซอร์โวมอเตอร์
int keepmode; //ตัวเก็บค่าของโหมด

void setup() {
  Serial.begin(115200); //ตรวจสอบ การส่งและรับของ Node MCU
  mySerial.begin(9600); // เริ่มต้น Serial ติดต่อกับ Arduino UNO
  MCUSerial.begin(19200); // เริ่มต้น Serial ติดต่อกับ Arduino UNO

  myServo.attach(servoPin); // เปิดการทำงาน Servo ด้วยขา servoPin(D1)
  myServo.write(currentAngle); //การระบุมุมเริ่มต้นของ Servo ในที่นี้คือ 90 องศา

  pinMode(MODE_PIN, OUTPUT); //ตั้งให้อ่าน Mode
  digitalWrite(MODE_PIN, LOW); 

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); //เปิดรับการทำงานของ Blynk
  Serial.println("รอข้อมูลจาก Odroid...");
}

void loop() {
  Blynk.run();
  unsigned long currentTime = millis(); //ทำการสร้าง current Time มาเก็บค่าเวลาตั้งแต่เริ่มทำงาน

  String receivedData = ""; 

  if (mySerial.available()) { //เมื่อมีการรับค่าจาก Blynk
    String receivedData = mySerial.readStringUntil('\n');  // อ่านข้อมูลจนถึง newline
    receivedData.trim();  // ลบช่องว่างและ newline

    if (receivedData.startsWith("Mode: ")) {  // ตรวจสอบว่าข้อมูลที่ได้รับเริ่มด้วย "Mode: "
      String modeString = receivedData.substring(6);  // ตัดเอาตัวเลขโหมด
      int newMode = modeString.toInt();  // แปลงข้อความเป็นตัวเลข

      if (newMode != mode) {
        mode = newMode;  // อัปเดตค่าโหมดใน NodeMCU ให้ตรงกับ Arduino
        keepmode = mode; //เก็บค่าโหมด
        Serial.print("Mode updated from Arduino: ");
        Serial.println(mode);
        Blynk.virtualWrite(READM_PIN, keepmode);

        // อัปเดตค่าโหมดใน Blynk เพื่อให้ข้อมูลสอดคล้องกัน
        Blynk.virtualWrite(MODE_VPIN, mode);
      }
    }
    if (receivedData.startsWith("temperature: ")) {
      Temp = receivedData.substring(13).toFloat();  // แยกข้อมูลโหมด
      Serial.print("Received Temp: ");
      Serial.println(Temp);
      mySerial.print("Temp: ");
      mySerial.println(Temp); //รับอุณหภูมิจาก Arduino uno
      MCUSerial.print("Temperature: ");
      MCUSerial.println(Temp);   //ส่งค่าอุณหภูมิไปยัง Odroid c4
      previousMillis = currentTime; //บันทึกเวลาก่อนหน้ากับค่าหลังทำเสร็จ

    }
  }
  // ตรวจสอบข้อมูลจาก Odroid และจัดการการเคลื่อนที่ของเซอร์โวมอเตอร์
  if (MCUSerial.available()) {
    String receivedData = MCUSerial.readStringUntil('\n');
    receivedData.trim();
    lastReceivedTime = currentTime;

    Serial.print("ได้รับข้อมูล: ");
    Serial.println(receivedData);

    // จัดการข้อมูล faceX
    if (receivedData.startsWith("FaceX: ")) {
      int faceXvalue = receivedData.substring(7).toInt(); //แปลงข้อความหลัง FaceX เป้นตัวเลข
      targetAngleX = map(faceXvalue, 0, 400, 180, 0); //เปลี่ยนค่าการตรวจสอบจาก 0-400  ให้กลายเป็น 180 - 0 องศา
      Serial.print("อัปเดต Face X: ");
      Serial.println(targetAngleX); 
    }

    // จัดการข้อมูล faceY
    if (receivedData.startsWith("FaceY: ")) {
      int faceYvalue = receivedData.substring(7).toInt(); //แปลงข้อความหลัง FaceY เป้นตัวเลข
      targetAngleY = map(faceYvalue, 0, 400, 180, 0); //เปลี่ยนค่าการตรวจสอบจาก 0-400  ให้กลายเป็น 180 - 0 องศา
      Serial.print("อัปเดต Face Y: ");
      Serial.println(targetAngleY); 
    }

    servoActive = true; // เปิดใช้งานเซอร์โวมอเตอร์เมื่อได้รับข้อมูล
  }


  // หากไม่มีการรับข้อมูลภายในเวลาที่กำหนด, หยุดเซอร์โวมอเตอร์และเก็บตำแหน่งสุดท้าย
  if (currentTime - lastReceivedTime > timeoutDuration) {
    //Serial.println("ไม่ได้รับข้อมูล, กำลังหยุดเซอร์โวมอเตอร์...");
    currentAngle = lastKnownAngle; // เก็บตำแหน่งสุดท้ายที่ทราบ
    myServo.write(currentAngle);
    delay(20);
    Serial.print("เซอร์โวมอเตอร์หยุดที่ตำแหน่ง: ");
    Serial.println(currentAngle);
    servoActive = false; // ปิดการใช้งานเซอร์โวมอเตอร์
  } else {
    lastKnownAngle = currentAngle; // อัปเดตตำแหน่งสุดท้ายที่ทราบขณะที่ยังทำงาน

    // จัดการการหมุนของเซอร์โวมอเตอร์ระหว่างมุมเป้าหมาย
    if (servoActive) {
      // ตรวจสอบว่ากำลังหมุนไปยัง faceX หรือ faceY
      if (abs(targetAngleX - targetAngleY) <= tolerance) { 
        isRotating = true; //เมื่อมีการหมุน
        Serial.println("กำลังหมุนไปยัง FaceX...");
        currentAngle = targetAngleX;  //ทำการระบุตำแหน่งของ targetAngleX
        myServo.write(currentAngle); //ทำการหมุน ไปยังตำแหน่ง targetAngleX
        delay(20); // ชะลอการหมุนเพื่อให้เซอร์โวมอเตอร์เคลื่อนที่ไปยังตำแหน่ง
        isRotating = false; //หมุนถึงตำแหน่งจะทำการเปลี่ยนเป็น false
        Serial.println("การหมุนไปยัง FaceX เสร็จสิ้น.");
      } else {
        // ถ้าไม่เท่ากัน, หมุนระหว่าง FaceX และ FaceY
        isRotating = true;
        Serial.println("เริ่มหมุนจาก FaceX ไปยัง FaceY...");

        // หมุนเซอร์โวมอเตอร์จาก FaceX ไปยัง FaceY
        for (currentAngle = targetAngleX; currentAngle != targetAngleY; ) {
          if (currentAngle < targetAngleY) currentAngle++; //หมุนไปตำแหน่ง Y
          else if (currentAngle > targetAngleY) currentAngle--; 
          myServo.write(currentAngle);
          delay(20);

          // ตรวจสอบข้อมูลใหม่ระหว่างการหมุนที่ถูกส่งมาจาก Odroid C4
          if (MCUSerial.available()) {
            String receivedData = MCUSerial.readStringUntil('\n');
            receivedData.trim();

            // จัดการข้อมูล faceX
            if (receivedData.startsWith("FaceX: ")) {
              int faceXvalue = receivedData.substring(7).toInt();
              targetAngleX = map(faceXvalue, 0, 400, 180, 0);
            }

            // จัดการข้อมูล faceY
            if (receivedData.startsWith("FaceY: ")) {
              int faceYvalue = receivedData.substring(7).toInt();
              targetAngleY = map(faceYvalue, 0, 400, 180, 0);
            }
          }
        }

        // กลับไปยัง FaceX
        for (currentAngle = targetAngleY; currentAngle != targetAngleX; ) { //หมุนไปตำแหน่ง X
          if (currentAngle < targetAngleX) currentAngle++;
          else if (currentAngle > targetAngleX) currentAngle--;
          myServo.write(currentAngle);
          delay(20);

          // ตรวจสอบข้อมูลใหม่ระหว่างการหมุน
          if (MCUSerial.available()) {
            String receivedData = MCUSerial.readStringUntil('\n');
            receivedData.trim();

            // จัดการข้อมูล faceX
            if (receivedData.startsWith("FaceX: ")) {
              int faceXvalue = receivedData.substring(7).toInt();
              targetAngleX = map(faceXvalue, 0, 400, 180, 0);
            }

            // จัดการข้อมูล faceY
            if (receivedData.startsWith("FaceY: ")) {
              int faceYvalue = receivedData.substring(7).toInt();
              targetAngleY = map(faceYvalue, 0, 400, 180, 0);
            }
          }
        }

        isRotating = false; //เมื่อเสร็จ 1 รอบจะทำการหยุดหมุน
        Serial.println("การหมุนเสร็จสิ้น, พร้อมรับข้อมูลใหม่.");
      }
    }
  }
}

// ตัวจัดการปุ่ม Blynk
BLYNK_WRITE(MODE_VPIN) {
  ButtonMode = param.asInt(); // รับค่าโหมดจาก Blynk
  if (ButtonMode == HIGH && previousMode == LOW) {
    mode = (mode % 4) + 1;
    Serial.print("Mode changed to: ");
    Serial.println(mode);

    // ส่งโหมดใหม่ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("Mode: ");
    mySerial.println(mode);

    digitalWrite(MODE_PIN, HIGH);  // Set pin HIGH to trigger the interrupt
    delay(100);  // Short delay to ensure the Arduino detects the signal
    digitalWrite(MODE_PIN, LOW);  // Reset the pin back to LOW
    keepmode = mode;
    Blynk.virtualWrite(READM_PIN, keepmode);
  } 
  previousMode = ButtonMode;
}

BLYNK_WRITE(Manul_VPIN) {
  ManaulButton = param.asInt(); // รับค่าโหมดจาก Blynk

  if (ManaulButton == HIGH && previosManaul == LOW) {
    manaul = 1; // เมื่อกด ON ส่งค่า 1
    Serial.println("Manual Mode: ON");
    
    // ส่งสถานะ ON ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("Manual Mode: ");
    mySerial.println(manaul);
  } 
  else if (ManaulButton == LOW && previosManaul == HIGH) {
    manaul = 0; // เมื่อกด OFF ส่งค่า 0
    Serial.println("Manual Mode: OFF");

    // ส่งสถานะ OFF ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("Manual Mode: ");
    mySerial.println(manaul);
  }
  
  previosManaul = ManaulButton; // อัปเดตสถานะปุ่ม
}
BLYNK_WRITE(UP_VPIN) {
  UPButton = param.asInt(); // รับค่าโหมดจาก Blynk

  if (UPButton == HIGH && previosUP == LOW) {
    up = 1; // เมื่อกด ON ส่งค่า 1
    Serial.println("UP Mode: ON");
    
    // ส่งสถานะ ON ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("UP Mode: ");
    mySerial.println(up);
  } 
  else if (UPButton == LOW && previosUP == HIGH) {
    up = 0; // เมื่อกด OFF ส่งค่า 0
    Serial.println("UP Mode: OFF");

    // ส่งสถานะ OFF ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("UP Mode: ");
    mySerial.println(up);
  }
  
  previosUP = UPButton; // อัปเดตสถานะปุ่ม
}
BLYNK_WRITE(DOWN_VPIN) {
  DOWNButton = param.asInt(); // รับค่าโหมดจาก Blynk

  if (DOWNButton == HIGH && previosDOWN == LOW) {
    down = 1; // เมื่อกด ON ส่งค่า 1
    Serial.println("DOWN Mode: ON");
    
    // ส่งสถานะ ON ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("DOWN Mode: ");
    mySerial.println(down);
  } 
  else if (DOWNButton == LOW && previosDOWN == HIGH) {
    down = 0; // เมื่อกด OFF ส่งค่า 0
    Serial.println("DOWN Mode: OFF");

    // ส่งสถานะ OFF ไปยัง Arduino (โดยใช้ SoftwareSerial)
    mySerial.print("DOWN Mode: ");
    mySerial.println(down);
  }
  
  previosDOWN = DOWNButton; // อัปเดตสถานะปุ่ม
}