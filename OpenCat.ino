/* Main Arduino sketch for OpenCat, the bionic quadruped walking robot.
   Updates should be posted on GitHub: https://github.com/PetoiCamp/OpenCat

   Rongzhong Li
   Jan.3rd, 2021
   Copyright (c) 2021 Petoi LLC.

   This sketch may also includes others' codes under MIT or other open source liscence.
   Check those liscences in corresponding module test folders.
   Feel free to contact us if you find any missing references.

   The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#define MAIN_SKETCH
#include "WriteInstinct/OpenCat.h"

/* MUSIC INCLUDES AND DEFINES */
//#define MODE_MUSIC
#ifdef MODE_MUSIC
// used to describe how fast the Arduino calls the interrupt. Higher means slower update and less accurate frequencies; lower means faster update and more accurate frequency. Anything lower than ~20 is corruptive.
#define SPEEDCONST 48
#define REST 126

#include "Music.h"
#include <math.h>

/* MUSIC CLASSES USED IN FORMATION OF SONGS */
// a single note to be played in a Motif
class Note {
  public:
    uint8_t duration; // durations: 64 = whole note, 32 = half note, 16 = quarter note, 8 = eighth note, 4 = sixteenth note
    int8_t distance; // "distance" from the root note in semitones. REST (REST) for silence.
    Note(char note, int nDur) : duration(nDur), distance(note) {}
};

// an array of Notes
class Motif {
  public:
    uint8_t length; // number of notes
    Note* notes;
    Motif(Note* nNotes,  unsigned char nLength) : length(nLength), notes(nNotes) {}
};

// an array of Motifs
class Song {
  public:
    uint8_t length;
    //uint8_t defaultTempo; // in "(1 / noteIsOneBeat)-notes per minute" i.e. "quarter notes per minute" or "eighth notes per minute"       NUMERATOR of time signature
    uint8_t defaultNoteIsOneBeat; // note that is one beat                                                                                DENOMINATOR of time signature
    double beatInMilliseconds;
    Motif** motifs;
    Song(Motif** nMotifs,  unsigned char nLength,  unsigned char tempo,  unsigned char beat) : motifs(nMotifs), length(nLength), defaultNoteIsOneBeat(beat) {
        beatInMilliseconds = getBeatInMilliseconds(tempo);
    }
};

// SONG LIBRARY AND CLASSES
// Some songs may be commented out, as the Arduino doesn't have enough memory to store them.
class Songbook {
    // declaration of Notes, Motifs, and Songs.
    // motifs' number postfix show the order in the song they first appear
    /*
      PREFIX KEY:
      n_          ...   Note[]  ---> Motif
      n_...REST#  ...   Note[] with only rests
      m_          ...   Motif
      s_          ...   Motif[] ---> Song
      [nothing]   ...   Song
    */
  private:
//    static Note n_BM1[6];
//    static Note n_BM2[6];
//    static Note n_BM3[6];
//    static Note n_BM4[6];
//    static Note n_BM5[7];
//    static Note n_BM6[4];
//    static Note n_BM7[6];
//    static Note n_BM8[8];
//    static Note n_BM9[7];
//    static Note n_BM10[8];
//    static Note n_BM11[8];
//    static Note n_BM12[10];
//    static Note n_BM13[5];

    static Note n_MEGAREST1[];
    static Note n_MEGAMelody[];
    static Note n_MEGA1[];
    static Note n_MEGA2[];
    static Note n_MEGA3[];
    static Note n_MEGA4[];
    static Note n_MEGA_Main1[];
    static Note n_MEGA_Main2[];
    
    // ----------------------------------
//    static Motif* m_BM1;
//    static Motif* m_BM2;
//    static Motif* m_BM3;
//    static Motif* m_BM4;
//    static Motif* m_BM5;
//    static Motif* m_BM6;
//    static Motif* m_BM7;
//    static Motif* m_BM8;
//    static Motif* m_BM9;
//    static Motif* m_BM10;
//    static Motif* m_BM11;
//    static Motif* m_BM12;
//    static Motif* m_BM13;

    static Motif* m_MEGAMelody;
    static Motif* m_MEGA1;
    static Motif* m_MEGA2;
    static Motif* m_MEGA3;
    static Motif* m_MEGA4;
    static Motif* m_MEGA_Main1;
    static Motif* m_MEGA_Main2;
    // ----------------------------------
//    static Motif* s_BanneredMare[16];
    static Motif* s_Megalovania[];
    // ----------------------------------
  public:
//    static Song BanneredMare;
    static Song Megalovania;
};
/* SONG DEFINITIONS */
// NOTE ARRAYS
//Note Songbook::n_BM1[6] = {
//  Note(15, 48), Note(12, 16), Note(14, 32), Note(15, 48), Note(17, 16), Note(15, 32) // x6
//};
//Note Songbook::n_BM2[6] = {
//  Note(14, 48), Note(10, 16), Note(12, 32), Note(14, 48), Note(15, 16), Note(14, 32) // x6
//};
//Note Songbook::n_BM3[6] = {
//  Note(12, 48), Note(8, 16), Note(10, 32), Note(12, 48), Note(14, 16), Note(12, 32) // x6
//};
//Note Songbook::n_BM4[6] = {
//  Note(11, 48), Note(12, 16), Note(9, 32), Note(11, 48), Note(REST, 16), Note(7, 32)// x6
//};
//Note Songbook::n_BM5[7] = {
//  Note(12, 48), Note(14, 16), Note(12, 16),  Note(REST, 16), Note(12, 48), Note(14, 16), Note(12, 32) // x7
//};
//Note Songbook::n_BM6[4] = {
//  Note(11, 64), Note(14, 32), Note(12, 64), Note(REST, 32) // x4
//};
//Note Songbook::n_BM7[6] = {
//  Note(19, 48), Note(15, 16), Note(17, 32), Note(19, 48), Note(20, 16), Note(19, 32) // x6
//};
//Note Songbook::n_BM8[8] = {
//  Note(17, 48), Note(14, 16), Note(15, 32), Note(17, 48), Note(19, 16), Note(17, 16), Note(15, 8), Note(99, 8) // x8
//};
//Note Songbook::n_BM9[7] = {
//  Note(14, 32 / 3), Note(15, 32 / 3), Note(14, 32 / 3 + 48 + 8), Note(REST, 8), Note(14, 64), Note(10, 24), Note(REST, 8) // x7
//};
//Note Songbook::n_BM10[8] = {
//  Note(19, 64), Note(15, 16), Note(17, 16), Note(19, 32), Note(20, 16), Note(19, 16), Note(17, 16), Note(15, 16) // x8
//};
//Note Songbook::n_BM11[8] = {
//  Note(17, 64), Note(14, 16), Note(15, 16), Note(17, 32), Note(19, 16), Note(17, 16), Note(15, 16), Note(14, 16) // x8
//};
//Note Songbook::n_BM12[10] = {
//  Note(15, 32), Note(12, 16), Note(14, 16), Note(15, 24), Note(REST, 8), Note(15, 32), Note(17, 16), Note(15, 16), Note(14, 16), Note(12, 16) // x10
//};
//Note Songbook::n_BM13[5] = {
//  Note(14, 32), Note(10, 32), Note(14, 32), Note(12, 64), Note(REST, 32) // x5
//};

Note Songbook::n_MEGAMelody[] = {
     Note(12, 4), Note(REST, 4),  Note(7, 4), Note(REST, 8),
     Note(6, 4), Note(REST, 4), Note(5, 4), Note(REST, 4), Note(3, 8),
     Note(0, 4), Note(3, 4), Note(5, 4) 
};
Note Songbook::n_MEGA1[] = {
     Note(0, 4), Note(0, 4)
};
Note Songbook::n_MEGA2[] = {
     Note(-2, 4), Note(-2, 4)
};
Note Songbook::n_MEGA3[] = {
     Note(-3, 4), Note(-3, 4)
};
Note Songbook::n_MEGA4[] = {
     Note(-4, 4), Note(-4, 4)
};
Note Songbook::n_MEGA_Main1[] = {
     Note(2, 1), Note(3, 3), Note(REST, 4), Note(3, 4), Note(3, 4), Note(REST, 4), Note(3, 4), Note(REST, 4), Note(2, 1), Note(3, 7), Note(0, 4), Note(REST, 4), Note(0, 12), Note(REST, 8)
};
Note Songbook::n_MEGA_Main2[] = {
     Note(3, 4), Note(REST, 4), Note(3, 4), Note(3, 4), Note(REST, 4), Note(5, 4), Note(REST, 4), Note(6, 8), Note(5, 2), Note(6, 2), Note(5, 4), Note(0, 4), Note(3, 4), Note(5, 4), Note(REST, 8)
};

// MOTIFS
//Motif* Songbook::m_BM1 = new Motif(n_BM1, 6);
//Motif* Songbook::m_BM2 = new Motif(n_BM2, 6);
//Motif* Songbook::m_BM3 = new Motif(n_BM3, 6);
//Motif* Songbook::m_BM4 = new Motif(n_BM4, 6);
//Motif* Songbook::m_BM5 = new Motif(n_BM5, 7);
//Motif* Songbook::m_BM6 = new Motif(n_BM6, 4);
//Motif* Songbook::m_BM7 = new Motif(n_BM7, 6);
//Motif* Songbook::m_BM8 = new Motif(n_BM8, 8);
//Motif* Songbook::m_BM9 = new Motif(n_BM9, 7);
//Motif* Songbook::m_BM10 = new Motif(n_BM10, 8);
//Motif* Songbook::m_BM11 = new Motif(n_BM11, 8);
//Motif* Songbook::m_BM12 = new Motif(n_BM12, 10);
//Motif* Songbook::m_BM13 = new Motif(n_BM13, 5);
//Motif* Songbook::s_BanneredMare[16] = {
//  m_BM1, m_BM2, m_BM3, m_BM4,
//  m_BM1, m_BM2, m_BM5, m_BM6,
//  m_BM7, m_BM8, m_BM1, m_BM9,
//  m_BM10, m_BM11, m_BM12, m_BM13
//};

Motif* Songbook::m_MEGAMelody = new Motif(n_MEGAMelody, 12);
Motif* Songbook::m_MEGA1 = new Motif(n_MEGA1, 2);
Motif* Songbook::m_MEGA2 = new Motif(n_MEGA2, 2);
Motif* Songbook::m_MEGA3 = new Motif(n_MEGA3, 2);
Motif* Songbook::m_MEGA4 = new Motif(n_MEGA4, 2);
Motif* Songbook::m_MEGA_Main1 = new Motif(n_MEGA_Main1, 14);
Motif* Songbook::m_MEGA_Main2 = new Motif(n_MEGA_Main2, 15);
Motif* Songbook::s_Megalovania[] = {
    m_MEGA1, m_MEGAMelody, m_MEGA2, m_MEGAMelody, m_MEGA3, m_MEGAMelody, m_MEGA4, m_MEGAMelody, 
    
    m_MEGA1, m_MEGAMelody, m_MEGA2, m_MEGAMelody, m_MEGA3, m_MEGAMelody, m_MEGA4, m_MEGAMelody,
    
    m_MEGA_Main1, m_MEGA_Main2
};
//Motif* Songbook::s_Megalovania[] = {
//    m_MEGA_Main1, m_MEGA_Main2
//};

// SONGS
//Song Songbook::BanneredMare = Song(s_BanneredMare, 16, 90, 8);
Song Songbook::Megalovania = Song(s_Megalovania, 18, 60, 4);

/* MUSIC VARIABLES */
// Timer reload value, globally available
unsigned int tcnt2;

// Toggle HIGH or LOW digital write
bool musicPulseHigh = 0;

// holds the current frequency to play (actually holds the note's half-period in 64-microseconds) (i.e. 0 = 0 us, 1 = 64 us, 2 = 2 * 64, etc.
uint8_t freq = REST;

// keeps track of when each note needs to be switched, used to describe half-period in 64-microseconds; increases with time up to the count1/2/etc.'s respective freq[0]/2/etc.
unsigned int musicFreqCounter = 0;

unsigned int musicNoteCounter = 0;

bool playingSong = false;

// holds the time to play the current note in units of 64-microseconds
int duration = 0;

// holds the index of the current Motif of the Song to play
uint8_t motifIndex = 0;

// holds the index of the current Note of a Motif to play
uint8_t noteIndex = 0;

double key = 220.0;
// Current song and note
Song* currentSong;
Note* currentNote;

/* MUSIC ISR SETUP */
// Installs the interrupt Service Routine (ISR) for Timer2, executed every SPEEDCONST microseconds
ISR(TIMER2_OVF_vect) {
  /* Reload the timer */
  TCNT2 = tcnt2;
  /* Note the lack of a for-loop for iterating through each channel. The notes sound slightly more grainy if the commands are executed through a for-loop. */

  // goes through each channel, sees if it's playing a song
  
    if (playingSong) {
      // Play the frequency, changing the pulse from HIGH to LOW when needed
      if (freq == REST) {
        musicFreqCounter = 0;
      } else {
        musicFreqCounter++;
        if (freq == musicFreqCounter) {
          musicPulseHigh = !musicPulseHigh;
          digitalWrite(BUZZER, musicPulseHigh ? HIGH : LOW);
          musicFreqCounter = 0;
        }
      }

      // Progress through the song, changing the note when needed
      if (musicNoteCounter == duration) {
        musicNoteCounter = musicFreqCounter = 0;
        if (noteIndex < currentSong->motifs[motifIndex]->length) { // same motif as before, next note
        } else { // next motif
          motifIndex++;
          noteIndex = 0;
        }
        if (motifIndex == currentSong->length) { // end of song
          endSong();
        } else {
          currentNote = &(currentSong->motifs[motifIndex]->notes[noteIndex]);
          duration = (currentNote->duration) * currentSong->beatInMilliseconds / currentSong->defaultNoteIsOneBeat * 1000 / SPEEDCONST;
          freq = (currentNote->distance ==  REST) ? REST : getMicros(getFreq(currentNote->distance, key)); // REST? rest if true, normal frequency if not
          noteIndex++;
        }
      } else {
        musicNoteCounter++;
      }
  }
}

// resets variables used for tracking notes, etc.
void endSong() {
  musicPulseHigh = 0;
  freq = REST;
  musicFreqCounter = 0;
  musicNoteCounter = 0;
  playingSong = false;
  duration = 0;
  noteIndex = 0;
  motifIndex = 0;
}

#endif
/* END MUSIC */

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128

//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using (1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may be broken
// and the reading will be unpredictable. it should be replaced with previous reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLag[HISTORY][3];

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[PACKET_SIZE]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//
//// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
//#include <IRremote.h>
///*-----( Declare objects )-----*/
//IRrecv irrecv(IR_RECEIVER);     // create instance of 'irrecv'
//decode_results results;      // create instance of 'decode_results'
//
//String translateIR() // takes action based on IR code received
//// describing Remote IR codes.
//{
//  switch (results.value) {
//    //IR signal    key on IR remote           //key mapping
//    case 0xFFA25D: /*PTLF(" CH-");   */       return (F(K00));
//    case 0xFF629D: /*PTLF(" CH");  */         return (F(K01));
//    case 0xFFE21D: /*PTLF(" CH+"); */         return (F(K02));
//
//    case 0xFF22DD: /*PTLF(" |<<"); */         return (F(K10));
//    case 0xFF02FD: /*PTLF(" >>|"); */         return (F(K11));
//    case 0xFFC23D: /*PTLF(" >||"); */         return (F(K12));
//
//    case 0xFFE01F: /*PTLF(" -");   */         return (F(K20));
//    case 0xFFA857: /*PTLF(" +");  */          return (F(K21));
//    case 0xFF906F: /*PTLF(" EQ"); */          return (F(K22));
//
//    case 0xFF6897: /*PTLF(" 0");  */          return (F(K30));
//    case 0xFF9867: /*PTLF(" 100+"); */        return (F(K31));
//    case 0xFFB04F: /*PTLF(" 200+"); */        return (F(K32));
//
//    case 0xFF30CF: /*PTLF(" 1");  */          return (F(K40));
//    case 0xFF18E7: /*PTLF(" 2");  */          return (F(K41));
//    case 0xFF7A85: /*PTLF(" 3");  */          return (F(K42));
//
//    case 0xFF10EF: /*PTLF(" 4");  */          return (F(K50));
//    case 0xFF38C7: /*PTLF(" 5");  */          return (F(K51));
//    case 0xFF5AA5: /*PTLF(" 6");  */          return (F(K52));
//
//    case 0xFF42BD: /*PTLF(" 7");  */          return (F(K60));
//    case 0xFF4AB5: /*PTLF(" 8");  */          return (F(K61));
//    case 0xFF52AD: /*PTLF(" 9");  */          return (F(K62));
//
//    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");
//
//    default: {
//        //Serial.println(results.value, HEX);
//      }
//      return ("");                      //Serial.println("null");
//  }// End Case
//  //delay(100); // Do not get immediate repeat //no need because the main loop is slow
//
//  // The control could be organized in another way, such as:
//  // forward/backward to change the gaits corresponding to different speeds.
//  // left/right key for turning left and right
//  // number keys for different postures or behaviors
//}


char token;
char lastToken;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
byte hold = 0;
int8_t offsetLR = 0;
bool checkGyro = true;
int8_t skipGyro = 2;

#define COUNT_DOWN 60

uint8_t timer = 0;
//#define SKIP 1
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstMotionJoint;
byte jointIdx = 0;

//bool Xconfig = false;


unsigned long usedTime = 0;
void getFIFO() {//get FIFO only without further processing
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
}

void getYPR() {//get YPR angles from FIFO data, takes time
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) ;
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    //PTL(fifoCount);
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) { //1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)

      // -- RzLi --
#ifdef FIX_OVERFLOW
#ifdef DEVELOPER
      PTLF("reset FIFO!");//FIFO overflow! Using last reading!
#endif
      lag = (lag - 1 + HISTORY) % HISTORY;
#endif
      // --
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      getFIFO();

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#ifdef MPU_YAW180
      ypr[2] = -ypr[2];
      ypr[1] = -ypr[1];

#endif
#endif
      for (byte g = 1; g < 3; g++)
        ypr[g] *= degPerRad;        //ypr converted to degree

      // overflow is detected after the ypr is read. it's necessary to keep a lag record of previous reading.  -- RzLi --
#ifdef FIX_OVERFLOW
      for (byte g = 1; g < 3; g++) {
        yprLag[lag][g] = ypr[g];
        ypr[g] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] ;
      }
      lag = (lag + 1) % HISTORY;
#endif

#ifdef DEVELOPER
      PT(ypr[0]);
      PTF("\t");
      PT(ypr[1]);
      PTF("\t");
      PTL(ypr[2]);
#endif
    }
  }
}

void checkBodyMotion()  {
  //if (!dmpReady) return;
  getYPR();
  // --
  //deal with accidents
  if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//wait until stable
    if (!hold)
      for (byte w = 0; w < 50; w++) {
        getYPR();
        delay(10);
      }
    if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//check again
      if (!hold) {
        token = T_SKILL;
        if (fabs(ypr[2]) > LARGE_ROLL) {
          strcpy(newCmd, "rc");
          newCmdIdx = 4;
        }
        else {
          strcpy(newCmd, ypr[1] < LARGE_PITCH ? "lifted" : "dropped");
          newCmdIdx = 1;
        }
      }
      hold = 10;
    }
  }

  // recover
  else if (hold) {
    if (hold == 1) {
      token = T_SKILL;
      strcpy(newCmd, "balance");
      newCmdIdx = 1;
    }
    hold --;
    if (!hold) {
      char temp[CMD_LEN];
      strcpy(temp, newCmd);
      strcpy(newCmd, lastCmd);
      strcpy(lastCmd, temp);
      newCmdIdx = 1;
      meow();
    }
  }
  //calculate deviation
  for (byte i = 0; i < 2; i++) {
    RollPitchDeviation[i] = ypr[2 - i] - motion.expectedRollPitch[i]; //all in degrees
    //PTL(RollPitchDeviation[i]);
    RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0);//filter out small angles
  }

  //PTL(jointIdx);
}

void setup() {
  pinMode(BUZZER, OUTPUT);
#ifdef PIXEL_PIN
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(3, pixels.Color(0, 0, LIT_ON));

  pixels.show();   // Send the updated pixel colors to the hardware.
#endif
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  PTLF("\n* Start *");
  PTLF("Initialize I2C");
  PTLF("Connect MPU6050");
  mpu.initialize();
  //do
  {
    delay(500);
    // verify connection
    PTLF("Test connection");
    PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.
  } //while (!mpu.testConnection());

  // load and configure the DMP
  do {
    PTLF("Initialize DMP");
    devStatus = mpu.dmpInitialize();
    delay(500);
    // supply your own gyro offsets here, scaled for min sensitivity

    for (byte i = 0; i < 4; i++) {
      PT(EEPROMReadInt(MPUCALIB + 4 + i * 2));
      PT(" ");
    }
    PTL();
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      PTLF("Enable DMP");
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      PTLF("Enable interrupt");
      attachInterrupt(INTERRUPT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      PTLF("DMP ready!");
      //dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);

  //opening music
#if WALKING_DOF == 8
  playMelody(MELODY);
#endif

//  //IR
//  {
//    //PTLF("IR Receiver Button Decode");
//    irrecv.enableIRIn(); // Start the receiver
//  }

  assignSkillAddressToOnboardEeprom();
  PTL();

  // servo
  { pwm.begin();

    pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
    delay(200);

    //meow();
    strcpy(lastCmd, "rest");
    motion.loadBySkillName(lastCmd);
    for (int8_t i = DOF - 1; i >= 0; i--) {
      pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      //PTL(SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i] * rotationDirection(i) );
      calibratedPWM(i, motion.dutyAngles[i]);
      delay(20);
    }
    randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
    shutServos();
    token = T_REST;
  }
  beep(30);

  pinMode(BATT, INPUT);
  pinMode(BUZZER, OUTPUT);
  meow();

#ifdef MODE_MUSIC
  /* MUSIC SETUP */
//  setKey('D', &key); // C D E F G A B
  /* TIMER STUFF */
  /* First disable the timer overflow interrupt*/
  TIMSK2 &= ~(1 << TOIE2);

  /* Configure timer2 in normal mode (no PWM) */
  TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
  TCCR2B &= ~(1 << WGM22);

  /* Select clock source: internal I/O clock */
  ASSR &= ~(1 << AS2);

  /* Disable Compare Match A interrupt (only overflow) */
  TIMSK2 &= ~(1 << OCIE2A);

  /* Configure the prescaler to CPU clock divided by 128 */
  TCCR2B |= (1 << CS22)  | (1 << CS20); // Set bits
  TCCR2B &= ~(1 << CS21);           // Clear bit

  /* We need to calculate a proper value to load the counter.
    The following loads the value 248 into the Timer 2 counter
    The math behind this is:
    (Desired period) = SPEEDCONSTus.
    (CPU frequency) / (prescaler value) = 125000 Hz -> 8us.
    SPEEDCONSTus / 8us
    MAX(uint8) - SPEEDCONSTus / 8us = 252;
  */
  /* Save value globally for later reload in ISR */
  tcnt2 = 256 - SPEEDCONST / 8;

  /* Finally load end enable the timer */
  TCNT2 = tcnt2;
  TIMSK2 |= (1 << TOIE2);
  
  // MUSIC SETUP DONE
  // PLAY SONG
  currentSong = &Songbook::Megalovania;
  playingSong = true;
  #endif
}

void loop() {
  float voltage = analogRead(BATT);
  if (voltage <
#ifdef NyBoard_V0_1
      650
#else
      300
#endif
     ) { //give the cat a break when voltage drops after sprint
    //adjust the thresholds according to your batteries' voltage
    //if set too high, the robot will keep crying.
    //If too low, Nybble may faint due to temporary voltage drop
    PTL("check battery");
    PTL(voltage);//relative voltage
//    meow();
    delay(1000);
  }
  else {
    newCmd[0] = '\0';
    newCmdIdx = 0;


    // input block
    //else if (t == 0) {
//    if (irrecv.decode(&results)) {
//      String IRsig = irParser(translateIR());
//      //PTL(IRsig);
//      if (IRsig != "") {
//        strcpy(newCmd, IRsig.c_str());
//        if (strlen(newCmd) == 1)
//          token = newCmd[0];
//        else
//          token = T_SKILL;
//        newCmdIdx = 2;
//      }
//      irrecv.resume(); // receive the next value
//    }
    if ( Serial.available() > 0) {
      token = Serial.read();
      newCmdIdx = 3;
    }

    // MPU block
    {
#ifdef GYRO //if opt out the gyro, the calculation can be really fast
      if (checkGyro) {
        if (!(timer % skipGyro)) {
          checkBodyMotion();

        }
        else if (mpuInterrupt || fifoCount >= packetSize)
        {
          // reset interrupt flag and get INT_STATUS byte
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
          getFIFO();
        }
      }
#endif
    }
    // accident block
    //...
    //...
    //for obstacle avoidance and auto recovery
    if (newCmdIdx) {
      PTL(token);
      beep(newCmdIdx * 4);
      // this block handles argumentless tokens
      switch (token) {
        //        case T_HELP: {
        //            PTLF("* info *");// print the help document. not implemented on NyBoard Vo due to limited space
        //            break;
        //          }
        case T_REST: {
            strcpy(lastCmd, "rest");
            skillByName(lastCmd);
            break;
          }
        case T_GYRO: {
            if (!checkGyro)
              checkBodyMotion();
            //            countDown = COUNT_DOWN;
            checkGyro = !checkGyro;
            token = T_SKILL;
            break;
          }
        case T_PAUSE: {
            tStep = !tStep;
            if (tStep)
              token = T_SKILL;
            else
              shutServos();
            break;
          }
        //        case T_RAMP: { //if the robot is on a ramp, flip the adaption direction
        //            ramp = (ramp == 1) ? -1.5 : 1;
        //            token = T_SKILL;
        //            break;
        //          }
        case T_SAVE: {
            PTLF("save offset");
            saveCalib(servoCalibs);
            break;
          }
        case T_ABORT: {
            PTLF("aborted");
            for (byte i = 0; i < DOF; i++) {
              servoCalibs[i] = servoCalib( i);
            }
            break;
          }

        // this block handles array like arguments
        case T_INDEXED: //indexed joint motions: joint0, angle0, joint1, angle1, ...
        case T_LISTED: //list of all 16 joint: angle0, angle2,... angle15
          //case T_MELODY: //for melody
          {
            String inBuffer = Serial.readStringUntil('~');
            int8_t numArg = inBuffer.length();
            char* list = inBuffer.c_str();
            char *targetFrame = new char [DOF];
            for (int i = 0; i < DOF; i += 1) {
              targetFrame[i] = currentAng[i];
            }
            if (token == T_INDEXED) {
              for (int i = 0; i < numArg; i += 2) {
                targetFrame[list[i]] = list[i + 1];
              }
            }
            else if (token == T_LISTED) {
              for (int i = 0; i < DOF; i += 1) {
                targetFrame[i] = list[i];
              }
            }
            transform(targetFrame, 1, 3); //need to add angleDataRatio if the angles are large
            delete [] targetFrame;

            //            if (token == T_INDEXED) {
            //              for (int i = 0; i < numArg; i += 2) {
            //                calibratedPWM(list[i], list[i + 1]);
            //              }
            //            }
            //            else if (token == T_LISTED) {
            //              allCalibratedPWM(list);
            //            }

            break;
          }
        case T_JOINTS: { //show the list of current joint anles
            printRange(DOF);
            printList(currentAng);
            break;
          }
        case T_CALIBRATE: //calibration
        case T_MOVE: //move jointIndex to angle
        case T_MEOW: //meow (repeat, increament)
        case T_BEEP: //beep(tone, duration): tone 0 is pause, duration range is 0~255
          {
            String inBuffer = Serial.readStringUntil('\n');
            char* temp = new char[30];
            strcpy(temp, inBuffer.c_str());
            char *pch;
            pch = strtok (temp, " ,");
            do {  //it supports combining multiple commands at one time
              //for example: "m8 40 m8 -35 m 0 50" can be written as "m8 40 8 -35 0 50"
              //the combined commands should be less than four. string len <=30 to be exact.
              int target[2] = {};
              byte inLen = 0;
              for (byte b = 0; b < 2 && pch != NULL; b++) {
                target[b] = atoi(pch);
                pch = strtok (NULL, " ,\t");
                inLen++;
              }
              PT(token);
              printList(target, 2);
              float angleInterval = 0.2;
              int angleStep = 0;
              if (token == T_CALIBRATE) {
                //PTLF("calibrating [ targetIdx, angle ]: ");

                if (strcmp(lastCmd, "c")) { //first time entering the calibration function
                  strcpy(lastCmd, "c");
                  motion.loadBySkillName("calib");
                  transform( motion.dutyAngles);
                  checkGyro=false;
                }
                if (inLen == 2)
                  servoCalibs[target[0]] = target[1];
                PTL();
                printRange(DOF);
                printList(servoCalibs);
                //yield();
                int duty = SERVOMIN + PWM_RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + motion.dutyAngles[target[0]]) * pulsePerDegree[target[0]] * rotationDirection(target[0]);
                pwm.setPWM(pin(target[0]), 0,  duty);
              }
              else if (token == T_MOVE) {
                //SPF("moving [ targetIdx, angle ]: ");
                angleStep = floor((target[1] - currentAng[target[0]]) / angleInterval);
                for (int a = 0; a < abs(angleStep); a++) {
                  int duty = SERVOMIN + PWM_RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + currentAng[target[0]] + a * angleInterval * angleStep / abs(angleStep)) * pulsePerDegree[target[0]] * rotationDirection(target[0]);
                  pwm.setPWM(pin(target[0]), 0,  duty);
                }
                currentAng[target[0]] = motion.dutyAngles[target[0]] = target[1];
              }
              else if (token == T_MEOW) {
                meow(target[0], 0, 50, 200, 1 + target[1]);
              }
              else if (token == T_BEEP) {
                beep(target[0], (byte)target[1]);
              }

              delay(50);
            } while (pch != NULL);
            delete []pch;
            delete []temp;
            break;
          }

        //        case T_XLEG: {
        //            for (byte s = 0; s < 23; s++) {
        //
        //              motion.loadDataFromProgmem(steps[s]);
        //              transform(motion.dutyAngles, 1,2);
        //            }
        //            skillByName("balance");
        //            break;
        //          }


        default: if (Serial.available() > 0) {
            String inBuffer = Serial.readStringUntil('\n');
            strcpy(newCmd, inBuffer.c_str());
          }
      }
      while (Serial.available() && Serial.read()); //flush the remaining serial buffer in case the commands are parsed incorrectly
      //check above
      if (strcmp(newCmd, "") && strcmp(newCmd, lastCmd) ) {
        //      PT("compare lastCmd ");
        //      PT(lastCmd);
        //      PT(" with newCmd ");
        //      PT(token);
        //      PT(newCmd);
        //      PT("\n");
        if (token == T_UNDEFINED) {}; //some words for undefined behaviors

        if (token == T_SKILL) { //validating key

          motion.loadBySkillName(newCmd);

          char lr = newCmd[strlen(newCmd) - 1];
          offsetLR = (lr == 'L' ? 15 : (lr == 'R' ? -15 : 0));

          //motion.info();
          timer = 0;
          if (strcmp(newCmd, "balance") && strcmp(newCmd, "lifted") && strcmp(newCmd, "dropped") )
            strcpy(lastCmd, newCmd);
          // Xconfig = strcmp(newCmd, "wkX") ? false : true;

#ifdef POSTURE_WALKING_FACTOR
          postureOrWalkingFactor = (motion.period == 1 ? 1 : POSTURE_WALKING_FACTOR);
#endif
          // if posture, start jointIdx from 0
          // if gait, walking DOF = 8, start jointIdx from 8
          //          walking DOF = 12, start jointIdx from 4
          firstMotionJoint = (motion.period <= 1) ? 0 : DOF - WALKING_DOF;
          //          if (Xconfig) { //for smooth transition
          //            int *targetAngle;
          //            targetAngle = new int [WALKING_DOF];
          //            for (byte i = 0; i < WALKING_DOF; i++)
          //              targetAngle[i] = motion.dutyAngles[i];
          //            targetAngle[WALKING_DOF - 6] = motion.dutyAngles[WALKING_DOF - 2] -5;
          //            targetAngle[WALKING_DOF - 5] = motion.dutyAngles[WALKING_DOF - 1] -5;
          //            targetAngle[WALKING_DOF - 2] = motion.dutyAngles[WALKING_DOF - 2] + 180;
          //            targetAngle[WALKING_DOF - 1] = motion.dutyAngles[WALKING_DOF - 1] + 180;
          //            transform( targetAngle,  1,2, firstMotionJoint);
          //            delete [] targetAngle;
          //          }
          //          else

          if (motion.period < 1) {
            int8_t repeat = motion.loopCycle[2] - 1;
            byte frameSize = 20;
            for (byte c = 0; c < abs(motion.period); c++) { //the last two in the row are transition speed and delay
              transform(motion.dutyAngles + c * frameSize, motion.angleDataRatio, motion.dutyAngles[16 + c * frameSize] / 4.0);
#ifdef GYRO //if opt out the gyro, the calculation can be really fast
              if (motion.dutyAngles[18 + c * frameSize]) {
                int triggerAxis = motion.dutyAngles[18 + c * frameSize];
                int triggerAngle = motion.dutyAngles[19 + c * frameSize];

                float currentYpr = ypr[abs(triggerAxis)];
                float previousYpr = currentYpr;
                while (1) {
                  getYPR();
                  currentYpr = ypr[abs(triggerAxis)];
                  PT(currentYpr);
                  PTF("\t");
                  PTL(triggerAngle);
                  if ((180 - fabs(currentYpr) > 2)  //skip the angle when the reading jumps from 180 to -180
                      && (triggerAxis * currentYpr < triggerAxis * triggerAngle && triggerAxis * previousYpr > triggerAxis * triggerAngle )
                     ) //the sign of triggerAxis will deterine whether the current angle should be larger or smaller than the trigger angle 
                    break;
                  previousYpr = currentYpr;
                }
              }
              else
#endif
              {
                delay(motion.dutyAngles[17 + c * frameSize] * 50);
              }
              if (c == motion.loopCycle[1] && repeat > 0) {
                c = motion.loopCycle[0] - 1;
                repeat--;
              }
            }
            skillByName("balance", 1, 2, false);
            strcpy(lastCmd, "balance");
            for (byte a = 0; a < DOF; a++)
              currentAdjust[a] = 0;
            hold = 0;
          }
          else {
            transform( motion.dutyAngles, motion.angleDataRatio, 1, firstMotionJoint);
          }
          jointIdx = 3;//DOF; to skip the large adjustment caused by MPU overflow. joint 3 is not used.
          if (!strcmp(newCmd, "rest")) {
            shutServos();
            token = T_REST;
          }
        }
        else {
          lastCmd[0] = token;
          memset(lastCmd + 1, '\0', CMD_LEN - 1);
        }
      }
    }

    //motion block
    {
      if (token == T_SKILL) {
        if (jointIdx == DOF) {
#ifdef SKIP
          if (updateFrame++ == SKIP) {
            updateFrame = 0;
#endif
            // timer = (timer + 1) % abs(motion.period);
            timer += tStep;
            if (timer == abs(motion.period)) {
              timer = 0;
              //              if (countDown == 0)
              //                checkGyro = true;
              //              else countDown--;
            }
            else if (timer == 255)
              timer = abs(motion.period) - 1;
#ifdef SKIP
          }
#endif
          jointIdx =
#ifdef HEAD  //start from head
            0;
#else
#ifdef TAIL
            2;
#else
            DOF - WALKING_DOF;
#endif
#endif
        }
#ifndef TAIL
        if (jointIdx == 1)
          jointIdx = DOF - WALKING_DOF;
#endif
        if (jointIdx < firstMotionJoint && abs(motion.period) > 1) {
          calibratedPWM(jointIdx, (jointIdx != 1 ? offsetLR : 0) //look left or right
                        + 10 * sin (timer * (jointIdx + 2) * M_PI / abs(motion.period)) //look around
#ifdef GYRO
                        + (checkGyro ? adjust(jointIdx) : 0)
#endif
                       );
        }
        else if (jointIdx >= firstMotionJoint) {
          int dutyIdx = timer * WALKING_DOF + jointIdx - firstMotionJoint;
          calibratedPWM(jointIdx, motion.dutyAngles[dutyIdx]*motion.angleDataRatio//+ ((Xconfig && (jointIdx == 14 || jointIdx == 15)) ? 180 : 0)
#ifdef GYRO
                        + (checkGyro ?
                           (!(timer % skipGyro)  ?
                            adjust(jointIdx)
                            : currentAdjust[jointIdx])
                           : 0)
                        //+ (checkGyro ? ((!(timer % skipGyro) && countDown == 0) ? adjust(jointIdx) : currentAdjust[jointIdx]) : 0)
#endif
                       );
          //          if (jointIdx == 8) {
          //            PT(currentAdjust[jointIdx]); PT("\t"); PTL( adjust(jointIdx) );
          //          }
        }
        jointIdx++;
      }
      else
        timer = 0;
    }
  }
}
