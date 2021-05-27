#if defined(MODE_MUSIC) || defined(MODE_MEGALO_TRACKING)
/* MUSIC INCLUDES AND DEFINES */

#include <math.h>
#include "Music.h"
#include "Songs.h"


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

double key = KEY;
// Current song and note
Song* currentSong = &Songbook::Megalovania;;
Note* currentNote;

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

#endif
/* END MUSIC */
