/*  OpenPipe Breakout samples example
 *
 *  Play sampled sounds (44100 Hz @ 8bit) using PWM output.
 *  Samples are defined in samples.h file, generated using the provided samples.py script
 *
 *  Connect the OpenPipe Breakout wires to Arduino as follows:
 *  GREEN-> A5 (SCL)
 *  WHITE-> A4 (SDA)
 *  BLACK -> A3 (GND) (Grey in old openpipes)
 *  RED -> A2 (VCC) (White in old openpipes)
 *
 *  Connect a speaker to PINs 11 & 12
 *
 *  © OpenPipe Labs. 2016
 *  www.openpipe.cc
 *
 *  This example code is in the public domain.
 */

#include <Wire.h> 
#include <OpenPipe.h> 
#include "samples.h"
#include <SFE_BMP180.h>

SFE_BMP180 pressure;
#define ALTITUDE 800.0 


#define GAITA_GALEGA

//#define ENABLE_DRONE

#ifdef GAITA_GALEGA
  #define INSTRUMENT INSTRUMENT_GAITA_GALEGA
#endif

#define SAMPLE_RATE 44100

unsigned long fingers, previous_fingers;
char note;
char playing;
unsigned char previous_sample, sample, drone_sample;
unsigned char sample_index, drone_index;
unsigned long int drone_sample_length;
sample_t* samples_table;
char status;
// medida de presion, temperatura, y demas del sensor
double T,P,p0,a;
double Pinicial;
// umbral de fol presionado, deberia ser 20, approx.
double umbral;
double debugmode;

void setup(){
  debugmode = 0;    // 1 debug activado, 0 desactivado
  umbral = -30;       // umbral < 0, sin fol; umbral >0, con fol
  Serial.begin(9600);
  Serial.println("OpenPipe SAMPLES");
  OpenPipe.power(A2, A3); 
  OpenPipe.config();
  Serial.println("config ok");
  pinMode(12,OUTPUT);
  digitalWrite(12, LOW);
  Serial.println("speaker ok");
  startPlayback();
  Serial.println("playback ok");  
  fingers=0;
  previous_fingers=0xFF;
  playing=0;
samples_table=INSTRUMENT;
#ifdef ENABLE_DRONE
//  NOTA DEL RONCO
  drone_sample=note_to_sample(48);
  drone_sample_length=samples_table[drone_sample].len;
#endif
  Serial.println("setup ok");  
    if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail\n\n");
//    while(1);
  }
// LECTURA INICIAL DE LA PRESION
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
//          Serial.print("absolute pressure: ");
//          Serial.println(P,2);
         Serial.print("Presion inicial:");
         Serial.println(P,2);
         Pinicial = P;
        }
//        else Serial.println("error retrieving pressure measurement\n");
      }
//      else Serial.println("error starting pressure measurement\n");
    }
//    else Serial.println("error retrieving temperature measurement\n");
  }
//  else Serial.println("error starting temperature measurement\n");
  
// -- LECTURA INICIAL DE LA PRESION


}

void loop(){
// PRESION
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
//          if (debugmode==1){
//            Serial.print("absolute pressure: ");
//            Serial.println(P,2);
//          }
        }
//        else Serial.println("error retrieving pressure measurement\n");
      }
//      else Serial.println("error starting pressure measurement\n");
    }
//    else Serial.println("error retrieving temperature measurement\n");
  }
//  else Serial.println("error starting temperature measurement\n");
  
// --PRESION


  fingers=OpenPipe.readFingers();
  
  
// Posiciones de programacion
// 1024 + 2048, para activar/desactivar el sensor de presion
  if (fingers == 3072) { 
    if (umbral >0) {
      umbral = -30;
    } else {
      umbral = 30;
    }
  }
// 7168, (tres pulsados) para activar/desactivar debug mode
  if (fingers == 7168) {
    if (debugmode ==1) {
      debugmode =0;
    } else {
      debugmode =1;
    }
  }
  if (fingers!=previous_fingers){
    if (debugmode == 1) {
      Serial.println(fingers); 
    }
//    Serial.println(previous_fingers);
    previous_fingers=fingers;  
//    OpenPipe.printFingers();
    // Check the low right thumb sensor
//    if (OpenPipe.isON() and P>940) {      
    if (P>(Pinicial+umbral)) {      
      playing=1;
      note=dimeNota(fingers);
      sample=note_to_sample(note);
//      Serial.print(" NOTE: ");
//      Serial.print(note, DEC);
//      Serial.print(" SAMPLE: ");
//      Serial.print(sample);
//      Serial.println();
    }else{
      sample=0xFF;
      playing=0;
//      Serial.println(" SILENCE");
    }      
  }
}


int dimeNota(int fingers){
// una funcion para obviar el problema de las tablas de digitacion con los arduinos viejos.
// el uso de esta funcion implica eliminar todas las referencias a fingering y fingering tables.
// notas sacadas de http://www.electronics.dit.ie/staff/tscarff/Music_technology/midi/midi_note_numbers_for_octaves.htm
// fingerings sacadas de debugging de estado, con el toque pechado 

  switch(fingers) {
    case 2431: return 71; 
    case 2238: return 72;
    case 2430: return 72;
    case 2429: return 73;
    case 2428: return 74;
    case 2426: return 75;
    case 2424: return 76;
    case 2427: return 76;
    case 2422: return 77;
    case 2416: return 77;
    case 2419: return 77;
    case 2418: return 77;
    case 2414: return 79;
    case 2406: return 79;
    case 2415: return 79;
    case 2398: return 80;
    case 2396: return 80;
    case 2382: return 81;
    case 2350: return 82;
    case 2318: return 83;
    case 2175: return 83;
    case 2334: return 83;
    case 2366: return 84;
    case 2362: return 84;
    case 2174: return 84;
    case 2364: return 84;
    case 2360: return 84;
    case 2358: return 84;
    case 2363: return 84;
    case 2173: return 85;
    case 2172: return 86;
    case 2168: return 88;
    default: return 0xFF;
  }
}

// Search sound sample index based on MIDI note
// Return sample index if found, 0xFF otherwise
int note_to_sample(int note){
  int i=0;
  while(samples_table[i].note!=0xFF){
    if (samples_table[i].note==note){
      return i;
    }
    i++;
  }
  return 0xFF;
}


///////////////////////////////////////////////////////////////
// PWM AUDIO FUNCTIONS
///////////////////////////////////////////////////////////////


// configure PWM for sound generation
void startPlayback()
{
  pinMode(11, OUTPUT);

  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.

  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  // Do non-inverting PWM on pin OC2A (p.155)
  // On the Arduino this is pin 11.
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));

  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set initial pulse width to the first sample.
  OCR2A = 127;

  // Set up Timer 1 to send a sample every interrupt.
  cli();

  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);

  sei();
}

// stop PWM sound
void stopPlayback()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);

  // Disable the per-sample timer completely.
  TCCR1B &= ~_BV(CS10);

  // Disable the PWM timer.
  TCCR2B &= ~_BV(CS10);

  digitalWrite(11, LOW);
}

/* PWM AUDIO CODE : This is called at SAMPLE_RATE Hz to load the next sample. */
ISR(TIMER1_COMPA_vect) {
  
  // STOP SOUND
  if ((P<(Pinicial+umbral))){
//  if (!(OpenPipe.isON())){
    OCR2A=127;
    return;
  }
  
  // PLAY PREVIOUS SAMPLE IF THE CURRENT ONE IS NOT FOUND
  if (sample==0xFF){
    //OCR2A=0;
    //return;
    sample=previous_sample;
  }
  
  // WAIT FOR THE SAMPLE TO FINISH IN ORDER TO AVOID 'CLICKS'
  if (previous_sample!=sample && sample_index==0){
    previous_sample=sample;
    //sample_index=0;
  }
  
  // LOOP SAMPLE
  if (sample_index==samples_table[previous_sample].len){
    sample_index=0;
  }else{
    sample_index++;
  }
  
#ifdef ENABLE_DRONE

  // LOOP DRONE SAMPLE
  if (drone_index==drone_sample_length){
    drone_index=0;
  }else{
    drone_index++;
  }
  
  // MIX NOTE AND DRONE SAMPLES
  int16_t out;
  out=0;
  out=pgm_read_byte_near(((uint8_t*)samples_table[drone_sample].sample) + drone_index)*2;
  out+=pgm_read_byte_near(((uint8_t*)samples_table[previous_sample].sample) + sample_index)*8;
  out=out>>4;
  OCR2A=out;
#else
  // UPDATE PWM
  OCR2A=pgm_read_byte_near(((uint8_t*)samples_table[previous_sample].sample) + sample_index);
#endif

}

