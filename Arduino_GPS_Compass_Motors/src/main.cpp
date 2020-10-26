#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long lastUpdateTime = 0;
unsigned int satelliteCount = 0;


//compass guiddnce

int *compassHeadingX = 0;
int *compassHeadingY = 0 ;
const int compassOffset = 10;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorFrontLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRearLeft = AFMS.getMotor(2);
Adafruit_DCMotor *motorRearRight = AFMS.getMotor(3);
Adafruit_DCMotor *motorFrontRight = AFMS.getMotor(4);

// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void motorStop()
{
  motorFrontLeft->run(RELEASE);
  motorFrontRight->run(RELEASE);
  motorRearLeft->run(RELEASE);
  motorRearRight->run(RELEASE);
}

void moveForward()
{
  motorFrontLeft->setSpeed(255);
  motorFrontRight->setSpeed(255);
  motorRearLeft->setSpeed(255);
  motorRearRight->setSpeed(255);

  motorFrontLeft->run(FORWARD);
  motorFrontRight->run(FORWARD);
  motorRearLeft->run(FORWARD);
  motorRearRight->run(FORWARD);
}

void moveBackward()
{
  motorFrontLeft->setSpeed(255);
  motorFrontRight->setSpeed(255);
  motorRearLeft->setSpeed(255);
  motorRearRight->setSpeed(255);

  motorFrontLeft->run(BACKWARD);
  motorFrontRight->run(BACKWARD);
  motorRearLeft->run(BACKWARD);
  motorRearRight->run(BACKWARD);
}

void moveLeft()
{
  motorFrontLeft->setSpeed(255);
  motorFrontRight->setSpeed(255);
  motorRearLeft->setSpeed(255);
  motorRearRight->setSpeed(255);

  motorFrontLeft->run(BACKWARD);
  motorFrontRight->run(FORWARD);
  motorRearLeft->run(BACKWARD);
  motorRearRight->run(FORWARD);
}

void moveRight()
{
  motorFrontLeft->setSpeed(255);
  motorFrontRight->setSpeed(255);
  motorRearLeft->setSpeed(255);
  motorRearRight->setSpeed(255);

  motorFrontLeft->run(FORWARD);
  motorFrontRight->run(BACKWARD);
  motorRearLeft->run(FORWARD);
  motorRearRight->run(BACKWARD);
}

void moveLeftLow()
{
  motorFrontLeft->setSpeed(155);
  motorFrontRight->setSpeed(255);
  motorRearLeft->setSpeed(155);
  motorRearRight->setSpeed(255);

  motorFrontLeft->run(BACKWARD);
  motorFrontRight->run(FORWARD);
  motorRearLeft->run(BACKWARD);
  motorRearRight->run(FORWARD);
}
void moveRightLow()
{
  motorFrontLeft->setSpeed(255);
  motorFrontRight->setSpeed(155);
  motorRearLeft->setSpeed(255);
  motorRearRight->setSpeed(155);

  motorFrontLeft->run(FORWARD);
  motorFrontRight->run(BACKWARD);
  motorRearLeft->run(FORWARD);
  motorRearRight->run(BACKWARD);
}

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  AFMS.begin(); // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed
}

void loop()
{

  //This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
  {
    const long TARGET_LATITUDE = 33.585072;
    const long TARGET_LONGITUE = -101.874796;
    gps.encode(ss.read());
    motorStop();

    if (millis() - lastUpdateTime >= 3000)
    {
      lastUpdateTime = millis();
      Serial.println(String(lastUpdateTime));

      unsigned long distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LATITUDE, TARGET_LONGITUE);
      double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LATITUDE, TARGET_LONGITUE);
      const char *cardinalDestination = TinyGPSPlus::cardinal(courseToDestination);
      int heading = (int)(360 + courseToDestination - gps.course.deg()) % 360;

      /*sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = ((atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi)+5.88;
  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  delay(500);
*/
      Serial.print("Lat: ");
      Serial.println(gps.location.lat());
      Serial.println("Long: ");
      Serial.println(gps.location.lng());
      Serial.print("Course to destination: ");
      Serial.println(courseToDestination);
      Serial.println("Current Course: ");
      Serial.print(gps.course.deg());
      Serial.print("Cardinal Direction: ");
      Serial.println(cardinalDestination);
      Serial.print("Adjustment needed: ");
      Serial.println(heading);
      Serial.print("Speed in kmph: ");
      Serial.println(gps.speed.kmph());

      if (distanceToDestination <= 1)
      {
        Serial.println("Destination Reached");
        exit(1);
      }

      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");

      if (gps.speed.kmph() < .1)
      {
        Serial.print("Head ");
        Serial.print(cardinalDestination);
        Serial.println(".");
        return;
      }

      if (heading >= 345 || heading < 15)
      {
        motorStop();
        delay(1000);
        moveForward();
      }
      else if (heading >= 315 && heading < 345)
      {
        Serial.println("Veer slightly to the left.");
        motorStop();
        delay(1000);
        moveLeftLow();
      }
      else if (heading >= 15 && heading < 45)
      {
        Serial.println("Veer slightly to the right.");
        motorStop();
        delay(1000);
        moveRightLow();
      }
      else if (heading >= 255 && heading < 315)
      {
        Serial.println("Turn to the left.");
        motorStop();
        delay(1000);
        moveLeft();
      }
      else if (heading >= 45 && heading < 105)
      {
        Serial.println("Turn to the right.");
        motorStop();
        delay(1000);
        moveRight();
      }
      else
      {
        Serial.println("Turn completely around.");
        motorStop();
        delay(1000);
        moveRight();
        delay(3000);
        motorStop();
      }
    }
  }
}