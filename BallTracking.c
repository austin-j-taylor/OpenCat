#ifdef MODE_MEGALO_TRACKING

#undef GYRO

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

#define DROP_MEASURE 7

MuVisionSensor *Mu;
MuVisionSensor Mu0(MU_ADDRESS);
MuVisionSensor Mu1(ALT_MU_ADDRESS);


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
//    Serial.println("MU initialized");
    Mu = &Mu0;
  } else {
//    Serial.println("fail to initialize");
    err = Mu1.begin(&Wire);
    if (err == MU_OK) {
//      Serial.println("MU initialized");
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
}

void loop_BallTracking(char* token, char* newCmd, byte* newCmdIdx) {
  // put your main code here, to run repeatedly:
  long time_start = millis();
  static int counter = 0;
  static bool isBlue = false;
  static int currentX = 0, currentY = 0; //the current x y of the camera's direction in the world coordinate
  static int state = 0;

  bool target_found = (*Mu).GetValue(VISION_COLOR_DETECT, kStatus); // update vision result and get status, 0: undetected, other: detected

  // Transitions
  switch(state) {
    case 0: // No object found
      if (target_found) {
        state = 1;
        (*Mu).LedSetColor(kLed2, kLedBlue, kLedClose, 1);
      }
      break;
    case 1: // Object found. singing. drop has not yet hit.
      if(motifIndex > DROP_MEASURE) { // start dancing
        state = 2;
        (*Mu).LedSetColor(kLed2, kLedBlue, kLedBlue, 1);
        *token = T_SKILL;
        strcpy(newCmd, "ck");
        *newCmdIdx = 3;
      } else if (!target_found) {
        state = 0;
//        *token = T_REST;
//        *newCmdIdx = 3;
        endSong();
      } else {
        if(motifIndex > 3) {
          counter++;
          if(counter % 2 == 0) {
            isBlue = !isBlue;
            if(isBlue) {
              (*Mu).LedSetColor(kLed2, kLedBlue, kLedClose, 1);
            } else {
              (*Mu).LedSetColor(kLed2, kLedYellow, kLedClose, 1);
            }
          }
        }
        playingSong = true;
      
        int xCoord = (int)(*Mu).GetValue(VISION_COLOR_DETECT, kXValue);
        int yCoord = (int)(*Mu).GetValue(VISION_COLOR_DETECT, kYValue);
      
        int xDiff = max(min((xCoord - 100 / 2) / 4, 30), -30);// 100 = the frame size 0~100 on X and Y direction
        int yDiff = max(min((yCoord - 100 / 2) / 4, 20), -20);
      
        currentX = max(min(currentX - min(xDiff, 40), 80), -90);
        currentY = max(min(currentY - min(yDiff, 30), 60), -75);
      
        int a[DOF] = {currentX / 1.2, 0, 0, 0, \
                      0, 0, 0, 0, \
                      60 - currentY / 2 + currentX / 6, 60 - currentY / 2 - currentX / 6, 90 + currentY / 3 - currentX / 8, 90 + currentY / 3 + currentX / 8, \
                      15 + currentY / 1.2  - currentX / 3, 15 + currentY / 1.2 + currentX / 3, -30 - currentY / 3 + currentX / 4, -30 - currentY / 3 - currentX / 4\
                     };
        transform(a, 1, 4);
      }
      break;
    case 2: // Drop has hit. waiting for song to end. Flash eye.
      if(!playingSong) {
        Serial.println("Song has finished. resting.");
        state = 0;
        *token = T_REST;
        *newCmdIdx = 3;
      } else {
        Serial.println("Dancing...");
        *token = T_SKILL;
        strcpy(newCmd, "ck");
        *newCmdIdx = 3;
      }
      break;
    default:
      break;
  }
}

#endif
