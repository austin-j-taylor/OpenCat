#ifdef MODE_MEGALO_TRACKING

/*
   Choose communication mode define here:
      I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
      SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
*/
#define I2C_MODE
//#define SERIAL_MODE


/*
   Choose MU address here: 0x60, 0x61, 0x62, 0x63
          default address: 0x60
*/
#define MU_ADDRESS        0x50 //in later versions we set the I2C device to 0x50, 0x51, 0x52, 0x53
#define ALT_MU_ADDRESS    0x60

#include <Arduino.h>
#include <MuVisionSensor.h>

#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif

MuVisionSensor *Mu;
MuVisionSensor Mu0(MU_ADDRESS);
MuVisionSensor Mu1(ALT_MU_ADDRESS);


int xCoord, yCoord; //the x y returned by the sensor
int xDiff, yDiff; //the scaled distance from the center of the frame
int currentX = 0, currentY = 0; //the current x y of the camera's direction in the world coordinate
int range = 100; //the frame size 0~100 on X and Y direction
int skip = 1, counter; //an efforts to reduce motion frequency without using delay. set skip >1 to take effect
int i2cdelay = 3;

void setup_BallTracking() {
  // put your setup code here, to run once:
  uint8_t err = 0;
#ifdef I2C_MODE
  Wire.begin();
  // initialized MU on the I2C port
  err = Mu0.begin(&Wire);
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  // initialized MU on the soft serial port
  err = Mu0.begin(&mySerial);
#endif
  if (err == MU_OK) {
    Serial.println("MU initialized");
    Mu = &Mu0;
  } else {
    Serial.println("fail to initialize");
    err = Mu1.begin(&Wire);
    if (err == MU_OK) {
      Serial.println("MU initialized");
      Mu = &Mu1;
    }
    delay(1000);
  }

  // configure MU board LEDs
  (*Mu).LedSetMode(kLed1, 0, 1);
  (*Mu).LedSetMode(kLed2, 0, 1);
  
  (*Mu).LedSetColor(kLed1, kLedClose, kLedClose, 1);
  (*Mu).LedSetColor(kLed2, kLedBlue, kLedClose, 1);

  // enable vision: ball
  (*Mu).VisionBegin(VISION_COLOR_DETECT);
  (*Mu).write(VISION_COLOR_DETECT, kLabel, MU_COLOR_PURPLE);

  counter = 0;
}

void loop_BallTracking() {
  // put your main code here, to run repeatedly:
  long time_start = millis();
  counter++;
  static bool isBlue = false;

  // read result
  if ((*Mu).GetValue(VISION_COLOR_DETECT, kStatus)) {                   // update vision result and get status, 0: undetected, other: detected
    xCoord = (int)(*Mu).GetValue(VISION_COLOR_DETECT, kXValue);
    yCoord = (int)(*Mu).GetValue(VISION_COLOR_DETECT, kYValue);
    //delay(i2cdelay);
    if (!(counter % skip)) {
      xDiff = max(min((xCoord - range / 2) / 4, 30), -30);
      yDiff = max(min((yCoord - range / 2) / 4, 20), -20);

      currentX = max(min(currentX - min(xDiff, 40), 80), -90);
      currentY = max(min(currentY - min(yDiff, 30), 60), -75);

      int a[DOF] = {currentX / 1.2, 0, 0, 0, \
                    0, 0, 0, 0, \
                    60 - currentY / 2 + currentX / 6, 60 - currentY / 2 - currentX / 6, 90 + currentY / 3 - currentX / 8, 90 + currentY / 3 + currentX / 8, \
                    15 + currentY / 1.2  - currentX / 3, 15 + currentY / 1.2 + currentX / 3, -30 - currentY / 3 + currentX / 4, -30 - currentY / 3 - currentX / 4\
                   };
      transform(a, 1, 4);
    }
    if(!playingSong) {
      playingSong = true;
    }
    // if drop has hit
//    if(counter % 2 == 0) {
//      isBlue = !isBlue;
//      if(isBlue) {
//        Serial.println("Blue");
//        (*Mu).LedSetColor(kLed2, kLedBlue, kLedClose, 1);
//      } else {
//        Serial.println("Yellow");
//        (*Mu).LedSetColor(kLed2, kLedYellow, kLedClose, 1);
//      }
//    }
    
  } else {
    if(playingSong) {
      endSong();
    }
  }
}

#endif
