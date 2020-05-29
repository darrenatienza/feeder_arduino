

#include <Servo.h>
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>              // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h> //uncomment when using a I2C
#include <DS1307RTC.h>

/** <GSM Variables> **/
#define BAUDRATE 9600

#define PHONE_NUMBER "+639480245911" //CELLPHONE NUMBER
//#define PHONE_NUMBER "+639501743188"

#define MESSAGE "" //COMPOSE YOUR MESSAGE HERE
#define MESSAGE_LENGTH 160

//SERIAL PIN ASSIGNMENT, BAUDRATE, PHONE NUMBER, MESSAGE
//positive and negative must be close to sim800l module to obtain good startup powering
#define PIN_TX 2
#define PIN_RX 3
GPRS GSMTEST(PIN_TX, PIN_RX, BAUDRATE); //RX,TX,BAUDRATE
char message[MESSAGE_LENGTH];
int messageIndex = 0;
char phone[16];
char datetime[24];
bool enableSMS = true; // whether sms is enable or disable

int resend_sms_delay_count = 0;
int count_to_resend = 60;
bool is_ready_to_send_sms = true; // always true because feed level sensor to wait

/** </GSM Variables> **/

// create servo object to control a servo
// twelve servo objects can be created on most boards
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int buttonPin = 7;
int cur_rtc_hrs = 0;
int cur_rtc_mins = 0;
int pos = 0; // variable to store the servo position
bool readyForFeeding = false;
int prev_hrs = 0;
int feedingCount = 0;

int morningTime = 6;
int afternoonTime = 17;

int servoPin1 = 11;
int servoPin2 = 10;
int servoPin3 = 6;
int servoPin4 = 5;

int relayPin = 12;
//lcd varibles
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Uncomment when using a I2C

void setup()
{
  initRelay();
  initLCD();
  delay(1000);
  intSMS();
  delay(1000);
  initServos();
}

void loop()
{

  if (digitalRead(buttonPin) == HIGH && readyForFeeding == false)
  {
    readyForFeeding = true;
  }
  else if (digitalRead(buttonPin) == HIGH && readyForFeeding == true)
  {
    readyForFeeding = false;
  }

  if (readyForFeeding)
  {
    lcd.clear();
    lcd.setCursor(0, 0); //Start at character 4 on line 0
    lcd.print("Feeding starts after 5 sec");
    delay(5000);
    processFeeding();

    //sms_try_to_send();
  }
  else
  {
    readRTCValue();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Unused Storage:");
    int unusedStorageCount = 4 - feedingCount;
    lcd.print(unusedStorageCount);

    lcd.setCursor(0, 1);
    lcd.print("Dispatch Time:");
    lcd.setCursor(0, 2);
    lcd.print(morningTime);
    lcd.print("hr");
    lcd.print(",");
    lcd.print(afternoonTime);
    lcd.print("hr");
    printTime();
    delay(500);
  }
}

/** 
 * Start - Initialization
 * 
*/
void initLCD()
{
  lcd.begin(20, 4);
  lcd.backlight(); //Uncomment when using a I2C
}
void initRelay()
{
  pinMode(relayPin, OUTPUT);
}
void initServos()
{

  digitalWrite(relayPin, HIGH);
  //show lcd init on screen
  //-------- Write characters on the display ------------------
  // NOTE: Cursor Position: (CHAR, LINE) start at 0
  //Start at character 4 on line 0
  lcd.setCursor(0, 0);
  lcd.print("Initializing Servos");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

  //attachAllServo();

  initServo(servo1, servoPin1);
  delay(500);
  setServoInitialPosition(servo1, false, 50, 180);
  delay(1000);
  initServo(servo2, servoPin2);
  delay(500);
  setServoInitialPosition(servo2, true, 0, 103);
  delay(1000);
  initServo(servo3, servoPin3);
  delay(500);
  setServoInitialPosition(servo3, true, 0, 103);
  delay(1000);
  initServo(servo4, servoPin4);
  delay(500);
  setServoInitialPosition(servo4, false, 50, 180);
  delay(1000);
  lcd.setCursor(0, 4);
  lcd.print("Done!");
  digitalWrite(relayPin, LOW);
  detachAllServo();
}
void initServo(Servo servo, int pin)
{
  servo.attach(pin); // attaches the servo on pin 9 to the servo object
}
void attachAllServo()
{
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
}
void detachAllServo()
{
  servo1.detach();
  //servo2.detach();
  servo3.detach();
  servo4.detach();
}
void intSMS()
{
  if (enableSMS)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing GSM Module");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mobile # to send:");
    lcd.setCursor(0, 1);
    lcd.print(PHONE_NUMBER);
    delay(2000);
    while (!GSMTEST.init())
    {
      delay(1000);
      lcd.clear();
      lcd.setCursor(4, 2);
      lcd.print("GSM INIT ERROR");
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GSM INIT SUCCESSFUL");
    delay(2000);
  }
}
/** End */

/** Operations */
void sms_try_to_send()
{
  if (!is_ready_to_send_sms)
  { //if false (means sent sms done) delay by 30 cur_mins (1800 secs)
    if (resend_sms_delay_count <= count_to_resend)
    {
      resend_sms_delay_count++;
    }
    else
    {
      is_ready_to_send_sms = true;
      resend_sms_delay_count = 0;
    }
  }
  //sms delay part ends here
  if (is_ready_to_send_sms)
  { //if true(means ready to send sms) send message now
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending SMS Notif...");
    delay(2000); // delete this
    sendSMS();
    lcd.setCursor(0, 1);
    lcd.print("SMS Sent");
    delay(1000);
    //is_ready_to_send_sms = false;
  }
}
void sendSMS()
{
  if (enableSMS)
  {
    String sms_message = "Storage ";
    sms_message += feedingCount;
    sms_message += " has been emptied!";
    char charBuf[50];
    sms_message.toCharArray(charBuf, 50);
    GSMTEST.sendSMS(PHONE_NUMBER, charBuf); //DEFINE PHONE NUMBER AND TEXT
  }
}
void sendSMS(String message)
{
  if (enableSMS)
  {
    char charBuf[70];
    message.toCharArray(charBuf, 70);
    GSMTEST.sendSMS(PHONE_NUMBER, charBuf); //DEFINE PHONE NUMBER AND TEXT
  }
}
void readRTCValue()
{

  tmElements_t tm;
  if (RTC.read(tm))
  {
    //printDate(5,1,tm);
    cur_rtc_hrs = tm.Hour;
    cur_rtc_mins = tm.Minute;
  }
  else
  {
    if (RTC.chipPresent())
    {
      //The DS1307 is stopped. Please run the SetTime
    }
    else
    {
      //DS1307 read error! Please check the circuitry
    }
    delay(9000);
  }
}
void printTime()
{
  String seconds, minutes;
  lcd.setCursor(0, 3);
  lcd.print("Time: ");
  lcd.print(cur_rtc_hrs);
  lcd.print(":");
  if (cur_rtc_mins < 10)
  {
    minutes = "0" + String(cur_rtc_mins);
    lcd.print(minutes);
  }
  else
  {
    lcd.print(cur_rtc_mins);
  }
}
/** 
 * run processFeeding if current time was indicated by user;
 * 
*/
void checkOperationTime()
{
  // hour value to operate
  int feed_hrs[] = {morningTime, afternoonTime};
  // loop the hour values
  for (int hr = 0; hr < sizeof(feed_hrs); hr++)
  {
    // set current hour value
    int _hr = feed_hrs[hr];
    //check if current hour value is equal to current rtc hour time
    if (_hr == cur_rtc_hrs)
    {
      //check if previous hour is not equeal to current rtc hour
      //if not it indicates that the current time is feeding time
      if (prev_hrs != cur_rtc_hrs)
      {
        readyForFeeding = true;
        prev_hrs = cur_rtc_hrs;
      }
    }
  }
}

void operateServo(Servo servo)
{
  for (pos = 60; pos <= 180; pos += 2)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);
    // tell servo to go to position in variable 'pos'
    delay(15); // waits 15ms for the servo to reach the position
  }
  delay(5000);
  for (pos = 180; pos >= 60; pos -= 2)
  {                   // goes from 180 degrees to 0 degrees
    servo.write(pos); // tell servo to go to position in variable 'pos'

    delay(15); // waits 15ms for the servo to reach the position
  }
  readyForFeeding = false;
}
/** 
 * Setting Operting Position of Servo
 * Params:
 * Servo= Servo pin
 * isClockWise = rotation of the servo
 * maxAngle = maximum angle where servo stopped normally 180 degree
 * minAngle = minimum angle where servo stopped normally 0 degree
 * Note: if isClockwise is false then max angle must be 180 and min angle must provide
 * if isClockwise is true then min angle must be 0 and max angle must provide
*/
void operateServo(Servo servo, bool isClockWise, int minAngle, int maxAngle)
{
  if (!isClockWise)
  {
    for (pos = minAngle; pos <= maxAngle; pos += 2)
    { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo.write(pos);
      // tell servo to go to position in variable 'pos'
      delay(15); // waits 15ms for the servo to reach the position
    }
    delay(5000);
    for (pos = maxAngle; pos >= minAngle; pos -= 2)
    {                   // goes from 180 degrees to 0 degrees
      servo.write(pos); // tell servo to go to position in variable 'pos'

      delay(15); // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = maxAngle; pos >= minAngle; pos -= 2)
    {                   // goes from 180 degrees to 0 degrees
      servo.write(pos); // tell servo to go to position in variable 'pos'

      delay(15); // waits 15ms for the servo to reach the position
    }
    delay(5000);
    for (pos = minAngle; pos <= maxAngle; pos += 2)
    { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo.write(pos);
      // tell servo to go to position in variable 'pos'
      delay(15); // waits 15ms for the servo to reach the position
    }
  }

  readyForFeeding = false;
}
void setServoInitialPosition(Servo servo)
{
  for (pos = 180; pos >= 60; pos -= 2)
  {                   // goes from 180 degrees to 0 degrees
    servo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);        // waits 15ms for the servo to reach the position
  }
}
void setServoInitialPositionCW(Servo servo)
{
  for (pos = 0; pos <= 110; pos += 2)
  {                   // goes from 180 degrees to 0 degrees
    servo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);        // waits 15ms for the servo to reach the position
  }
}
/** 
 * Setting Initial Position of Servo
 * Params:
 * Servo= Servo pin
 * isClockWise = rotation of the servo
 * maxAngle = maximum angle where servo stopped
 * minAngle = minimum angle where servo stopped
 * Note: if isClockwise is false then max angle must be 180 and min angle must provide
 * if isClockwise is true then min angle must be 0 and max angle must provide
*/
void setServoInitialPosition(Servo servo, bool isClockwise, int minAngle, int maxAngle)
{
  if (!isClockwise)
  {
    for (pos = maxAngle; pos >= minAngle; pos -= 2)
    {                   // goes from 180 degrees to 0 degrees
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);        // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = minAngle; pos <= maxAngle; pos += 2)
    {                   // goes from 180 degrees to 0 degrees
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);
    }
  }
}
/** */

void processFeeding()
{

  switch (feedingCount)
  {
  case 0:
    // counter clock wise
    // rotate servo
    servo1.attach(servoPin1);
    digitalWrite(relayPin, HIGH);
    operateServo(servo1, false, 50, 180);
    digitalWrite(relayPin, LOW);
    feedingCount += 1;
    delay(10000);
    servo1.detach();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending SMS Notif...");
    delay(10000);
    sendSMS("Storage 1 has been emptied!");
    lcd.setCursor(0, 1);
    lcd.print("SMS Sent");

    //delay(1000);
    break;

  case 1:
    //clock wise
    servo2.attach(servoPin2);
    operateServo(servo2, true, 0, 103);
    delay(10000);
    //servo2.detach();
    feedingCount += 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending SMS Notif...");
    delay(10000); // delete this
    sendSMS("Storage 2 has been emptied!");
    lcd.setCursor(0, 1);
    lcd.print("SMS Sent");
    //delay(1000);

    break;
  case 2:
    // clock wise
    servo3.attach(servoPin3);
    operateServo(servo3, true, 0, 103);
    delay(10000);
    servo3.detach();
    feedingCount += 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending SMS Notif...");
    delay(10000); // delete this
    sendSMS("Storage 3 has been emptied!");
    lcd.setCursor(0, 1);
    lcd.print("SMS Sent");
    //delay(1000);

    break;
  case 3:
    // counter clock wise
    //return feeding to 0
    servo4.attach(servoPin4);
    operateServo(servo4, false, 50, 180);
    feedingCount = 0;
    delay(10000);
    servo4.detach();
    servo2.detach();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending SMS Notif...");
    delay(10000); // delete this
    sendSMS("Storage 4 has been emptied!Please refill all storage!");
    lcd.setCursor(0, 1);
    lcd.print("SMS Sent");
    //delay(1000);
    //setup();
    break;
  }
}
