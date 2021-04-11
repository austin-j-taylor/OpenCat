// used in calculation of frequencies
// Frequency 'n' semitones away from Base Frequency = Base Frequency * ((2^(1/12))^n)
#define twototheonetwelfth pow(2, (1 / 12.0))
// used to describe conversion from Hz to microseconds for Arduino
#define MICROC 1000000.0 / 2 / SPEEDCONST

/* FUNCTION DECLARATIONS */
//void setKey(char, double*);
double getBeatInMilliseconds(char);
double getFreq(uint8_t, double);
char getMicros(double);

/* FUNCTION DEFINITIONS */

// tempo is in units of "(1 / noteIsOneBeat)-notes per minute"
// beatInMilliseconds is the period of one beat in milliseconds (of course)
double getBeatInMilliseconds(char tempo) {
  return (1.0 / (tempo / 60.0 / 1000.0) / 8);
}
double getFreq(uint8_t distance, double key) {
  return key * pow(twototheonetwelfth, distance);
}
char getMicros(double freq) {
  return (char) (MICROC / freq + .5);
}
