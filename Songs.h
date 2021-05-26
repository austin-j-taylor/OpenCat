#if defined(MODE_MUSIC) || defined(MODE_MEGALO_TRACKING)

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
    static Note n_MEGA_Main3[];
    static Note n_MEGA_Main4[];
    static Note n_MEGA_Main5[];
    
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
    static Motif* m_MEGA_Main3;
    static Motif* m_MEGA_Main4;
    static Motif* m_MEGA_Main5;
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
Note Songbook::n_MEGA_Main3[] = {
     Note(3, 4), Note(REST, 4), Note(3, 4), Note(3, 3), Note(REST, 5), Note(5, 4), Note(REST, 4), Note(6, 4), Note(REST, 4), Note(7, 4), Note(REST, 4), Note(10, 4), Note(REST, 4), Note(7, 12),
	 Note(12, 8), Note(12, 7), Note(REST, 1), Note(12, 4), Note(7, 4), Note(12, 4), Note(10, 19), Note(REST, 1), Note(10, 16)
};
Note Songbook::n_MEGA_Main4[] = {
     Note(7, 4), Note(REST, 4), Note(7, 4), Note(7, 4), Note(REST, 4), Note(7, 4), Note(REST, 4), Note(6, 1), Note(7, 7), Note(5, 4), Note(REST, 4), Note(5, 12), Note(REST, 8),
	 Note(7, 4), Note(REST, 4), Note(7, 4), Note(REST, 4), Note(7, 8), Note(REST, 4), Note(5, 4), Note(REST, 4), Note(7, 4), Note(REST, 4), Note(12, 8), Note(7, 4), Note(5, 8)
};
Note Songbook::n_MEGA_Main5[] = {
	 Note(12, 8), Note(7, 8), Note(5, 8), Note(3, 8), Note(10, 8), Note(5, 8), Note(3, 8), Note(2, 8),
	 Note(-4, 8), Note(0, 4), Note(2, 8), Note(3, 8), Note(10, 36)
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
Motif* Songbook::m_MEGA_Main3 = new Motif(n_MEGA_Main3, 23);
Motif* Songbook::m_MEGA_Main4 = new Motif(n_MEGA_Main4, 26);
Motif* Songbook::m_MEGA_Main5 = new Motif(n_MEGA_Main5, 13);

Motif* Songbook::s_Megalovania[] = {
    m_MEGA1, m_MEGAMelody, m_MEGA2, m_MEGAMelody, m_MEGA3, m_MEGAMelody, m_MEGA4, m_MEGAMelody, 
    
    m_MEGA1, m_MEGAMelody, m_MEGA2, m_MEGAMelody, m_MEGA3, m_MEGAMelody, m_MEGA4, m_MEGAMelody,
    
    m_MEGA_Main1, m_MEGA_Main2, m_MEGA_Main3, m_MEGA_Main4, m_MEGA_Main5
};
// Motif* Songbook::s_Megalovania[] = {
   // m_MEGA_Main1, m_MEGA_Main2, m_MEGA_Main3, m_MEGA_Main4, m_MEGA_Main5
// };

// SONGS
//Song Songbook::BanneredMare = Song(s_BanneredMare, 16, 90, 8);
Song Songbook::Megalovania = Song(s_Megalovania, 21, 60, 4);

#endif
