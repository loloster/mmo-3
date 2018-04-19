
// MMO-3
// based on arduino duo
//
// main file
// --------------------------------------------------------------------------
// This file is part of the MMO-3 firmware.
//
//    MMO-3 firmware is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    MMO-3 firmware is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MMO-3 firmware. If not, see <http://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------

#include <Arduino.h>
//#include <SPI.h>

#include "ARS.h"
#include "conf.h"

#pragma GCC optimize ("-O3")

// global variables
int32_t audio_outR, audio_outL, audio_out2, audio_inR2, audio_inL, audio_inR;

// in_ADC
uint32_t adc_value16[33];

// in keyboard
uint32_t KEY_LOCAL_goal, NOTE_ON;
uint32_t KEY_LOCAL, KEY_global;
uint32_t flash_lock_bit;

// in MIDI
uint32_t MIDI_gate, MIDI_pitch;
uint32_t MIDI_fader[33];

// portamento
uint32_t portamento;

// module VCO

// module VCA
int32_t  MIX_outL, MIX_outR, VCA_outL, VCA_outR;

// module ADSR
uint32_t ADSR_out;

// module LFO
uint32_t LFO1_phase, LFO1_increment;

// module LFO2 et 3
uint32_t LFO3_mode;

// modulation
int32_t modulation_data[16]; // 
int32_t modulation_data_AM[16]; // positif only : seulement pour la modulation AM et WS
uint32_t modulation_index[16]; // pointeur vers les données des 9 faders de modulations + joystick
uint32_t modulation_type_all;

// leds 
uint32_t led1_time; // time in data loop number
uint32_t blink_time, blink_led;

// config
uint32_t GATE_mode, ADSR_mode, audio_out_mode;

//joystick
int32_t MIDI_joystick_X, MIDI_joystick_Y;

// env follower
uint32_t envelope;

// DAC
uint32_t dac_on;

#ifdef serialout
  // Loop Counter
  uint16_t loopc;
  
  // main loop "jumper"
  uint16_t slowloop=0;

  // 2nd Loop Counter
  uint8_t compteur_cc = 0;

  // for incoming serial data  
  int incomingByte = 0xFF;

  // shotgun
  //const byte szshtgn=4;
  byte shotgun[4];
  uint8_t shotguncounter=2;
  
#endif

void setup() {
  uint32_t i;

  #ifdef serialout
  /*for (i=0;i<szshtgn;i++){
    shotgun[i]=0xFF;
    }
    */
    shotgun[0]=0xFF;
    shotgun[1]=0xFF;
    shotgun[2]=0xFF;
    shotgun[3]=0xFF;
  #endif
  
  REG_PMC_PCER0 = 1 << 11; // enable la clock du port PIO A pour les entree
  REG_PMC_PCER0 = 1 << 12; // enable la clock du port PIO B pour les entree
  REG_PMC_PCER0 = 1 << 13; // enable la clock du port PIO C pour les entree
  REG_PMC_PCER0 = 1 << 14; // enable la clock du port PIO D pour les entree

  REG_SUPC_SMMR = 0x0000110B; // suply monitor reset at 3V
  REG_SUPC_MR = 0xA5005A00;

  //EFC0->EEFC_FMR = 0X00000400; // mandatory to keep program speed when loading the dueFlashStorage library. go wonder why.

  init_dac();

  init_debug();
  init_led();
  init_analog_out();
  init_random();
  init_analog_in();
  init_keyboard();
  init_joystick(); 
  init_midi();
  init_save();
  
  init_VCO();
  init_LFO1();
  init_LFO2();
  init_LFO3();
  init_ADSR();
  init_VCA();
  //init_ENV();
  
  VCO1_freq();
  VCO2_freq();
  VCO3_freq();
  PORTAMENTO_update();
  VCA_update();
  
  test(); // hardware test mode
 
  start_dac();

  #ifdef serialout
    Serial.begin(9600);
    //Serial.begin(57600);
    //SerialUSB.begin(9600);
    //SerialUSB.begin(57600);
    SerialUSB.begin(115200);
  //SPI.begin(4);
    Serial.println("Hey! Hey!");
    Serial.println("MMO-3!");
    /*Serial.write((byte)0x00);
    Serial.write((byte)0xFF);
    Serial.write("M");
    Serial.write("M");
    Serial.write("O");
    Serial.write("-");
    Serial.write("3");
    Serial.write((byte)0x00);
    */
   #endif
        
  while (true) main_loop(); // faster than arduino loop
}

inline void main_loop() { // as fast as possible
  uint32_t compteur, tmpU32;
  int32_t tmp32;/*=0;*/
  uint8_t i;/*, j;*/
  
  
  
  #ifdef syncro_out
    test2_on();
  #endif
  analog_start_1();
  
  if (flash_lock_bit == 0) 
    keyboard_in();
  else
    if (efc_perform_command_is_ready(EFC1))
      save_conf0();
        
  VCO1_freq();
  VCO2_freq();    
  VCO3_freq();    
  #ifdef syncro_out
    test2_off();
  #endif  
  analog_in();
  MIDI_in();

  LFO1_modulation();
  //if (//Serial.availableForWrite()){ 
  // Serial.print("LFO1_phase:");
  // //Serial.println(LFO1_phase);
  //}
  //if (//Serial.availableForWrite()){ 
  // //Serial.print("LFO1_increment:");
  // //Serial.println(LFO1_increment);
  //}

  LFO2_modulation();
  LFO3_modulation();
  PORTAMENTO_update();
  ADSR_update();
  update_ext(); // modulation from external analog value
  // MIX_update(); // put in the analog convertion
  // VCA_update(); // idem
  // joystick();   // idem
  update_leds(); // gate and midi leds

  #ifdef serialout

//SerialUSB.print > https://forum.arduino.cc/index.php?topic=410584.0
  if ((SerialUSB.available() > 0)){ // || (Serial.available() > 0)){
      //read the incoming byte:
      /*if (SerialUSB.available() > 0)*/ incomingByte = SerialUSB.read();
        //else incomingByte = Serial.read();
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
      if ((incomingByte < 100) || (incomingByte == 0xF0) || (incomingByte == 0xF1) || (incomingByte == 0xF2)){
        shotguncounter=2;
        shotgun[0]=incomingByte;
      } else if (incomingByte > 0)
        if (incomingByte == 0xFF) /*for (i=0;i<szshtgn;i++)*/ {
          /*shotgun[i]=0xFF;*/
           shotgun[0]=0xFF;
           shotgun[1]=0xFF;
           shotgun[2]=0xFF;
           shotgun[3]=0xFF;
           slowloop=0;
        }
        else { // setting up continuous dump messages
         if (incomingByte ==  shotgun[1]) {
          shotgun[1]=shotgun[2];
          shotgun[2]=shotgun[3];
          shotgun[3]=0xFF;
         }
         else {
          if (incomingByte == shotgun [2]) {
           shotgun[2]=shotgun[3];
           shotgun[3]=0xFF;
          }
          else {
           if (incomingByte == shotgun [3]) {
            shotgun[3]=0xFF;
           }
           else {
            shotgun[3]=shotgun[2];
            shotgun[2]=shotgun[1];
            shotgun[1]=incomingByte;
            slowloop = ((slowloop == 1) && (shotgun[2] == 0xFF)) ? 0 : !(slowloop) ? 1 : slowloop;
           }
          }
         }
        }
        /*{ tmp32=0;
          for (i=1;i<szshtgn;i++) {
            if (incomingByte == shotgun[i]) {
              for (j=i;j<szshtgn-1;j++) {
                shotgun[j]=shotgun[j+1];
              }
              shotgun[szshtgn-1]=0xFF;
              tmp32=1;
            }
          }
          if (!tmp32) {
            for (j=szshtgn;j>1;--j){
              shotgun[j]=shotgun[j-1];
            }
            shotgun[1]=incomingByte;
          }
        }*/
/*        Serial.print("shotgun[0]:");
        Serial.println(shotgun[0]);
        Serial.print("shotgun[1]:");
        Serial.println(shotgun[1]);
        Serial.print("shotgun[2]:");
        Serial.println(shotgun[2]);
        Serial.print("shotgun[3]:");
        Serial.println(shotgun[3]);
*/      /*for (i=0;i<szshtgn;i++) {
        Serial.print("shotgun[");
        Serial.print(i);
        Serial.print("]:");
        Serial.println(shotgun[i]);
      }*/
  } // else incomingByte=0xFF;
  
  if (!(loopc++ < slowloop)){
    loopc=0;
    /*if (SerialUSB.available() > 0) {
      //read the incoming byte:
      incomingByte = SerialUSB.read();
                // say what you got:
        //        SerialUSB.print("I received: ");
        //        SerialUSB.println(incomingByte, DEC);
    }*/
    if (incomingByte != 0xFF) {
    //if (Serial.availableForWrite()){
      //switch ((compteur_cc++ % 32)) {
      //for (i=0;i<szshtgn;i++){
      for (i=0;i<4;i++){
        switch (shotgun[i]) {
      //switch (incomingByte) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
        case 23:
        case 24:
        case 25:
        case 26:
        case 27:
        case 28:
        case 29:
        case 30:
        case 31:
        if (0 < shotguncounter) {
        SerialUSB.write(0xFF);
        SerialUSB.write((byte)shotgun[i]);
//        SerialUSB.write((byte)0x00);
//        SerialUSB.write((byte)0x00);
        SerialUSB.write(adc_value16[shotgun[i]] >>  8 & 0xFF);
        SerialUSB.write(adc_value16[shotgun[i]] & 0xFF);
        shotguncounter--;
        }
        break;

        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
        case 0xA9:
        case 0xAA:
        case 0xAB:
        case 0xAC:
        SerialUSB.write(0xFF);
        SerialUSB.write((byte)shotgun[i]);
//        SerialUSB.write(modulation_data[shotgun[i]-0xA0] >> 24 & 0xFF);
//        SerialUSB.write(modulation_data[shotgun[i]-0xA0] >> 16 & 0xFF);
        SerialUSB.write(modulation_data[shotgun[i]-0xA0] >>  8 & 0xFF);
        SerialUSB.write(modulation_data[shotgun[i]-0xA0] & 0xFF);
      
        break;
        
        case 0xF0:
        if (0 < shotguncounter) {
        SerialUSB.write(0xFF);
        SerialUSB.write(0xF0);
//        SerialUSB.print("MMO3");
        SerialUSB.print("M3");
        shotguncounter--;
        }
        break;

        case 0xF1:
        if (0 < shotguncounter) {
          if (!(SerialUSB.available() > 0))
            slowloop++;
            else slowloop = SerialUSB.read();

        Serial.print("Slowing down shotgun:");
        Serial.println(slowloop);
        shotguncounter=0;
        }
        break;
        
        case 0xF2:
        if (0 < shotguncounter) {
          if (!(SerialUSB.available() > 0))
            slowloop = (1 < slowloop) ? --slowloop : (shotgun[2] == 0xFF) ? 0 : 1;
            else slowloop = SerialUSB.read();
        
        Serial.print("Speeding up shotgun:");
        Serial.println(slowloop);
        shotguncounter=0;
        }
        break;

        }
      }
    }
  }
  #endif
  
  //analog_out_1((modulation_data[modulation_index[index_VCO1_MOD1]]<<16)^0x80000000);
  //analog_out_2((modulation_data_AM[modulation_index[index_VCO1_MOD1]]<<16));
}

void loop() {
  //not used
}

inline void compute_audio_sample() {
  uint32_t VCO1_out, VCO2_out, VCO3_out, VCO4_out;
  uint32_t ADSR_tmp;
  sound stereo;
  uint32_t modulation_type_local;  
  modulation_type_local = modulation_type_all;
  
  modulation_audio();
  PORTAMENTO();
  VCO1_out = VCO1(modulation_type_local); // moins de 2µs
  VCO2_out = VCO2(modulation_type_local);
  VCO3_out = VCO3(modulation_type_local);
  ADSR_tmp = ADSR();
  stereo = MIX(VCO1_out, VCO2_out, VCO3_out, ADSR_tmp); // 2.4 µs
  stereo = VCA(stereo); // 0.6µs
  LFO1_audio(); // 0.3µs
  LFO2_audio(); // 0.3µs
  LFO3_audio(); 
  //ENVELOPE();
  joystick_audio(); // 1.8µs
  audio_outR = stereo.U32[0];
  audio_outL = stereo.U32[1];
}

void SSC_Handler(void){
uint32_t status;

  #ifdef syncro_out
    test1_on();
  #endif
    
  if (!(REG_SSC_SR & SSC_IER_TXSYN)) {
    REG_SSC_THR = REG_SSC_RHR; // just to initialise properlly (not to invert R and L)
    NVIC_ClearPendingIRQ(SSC_IRQn); 
  }
  else {
    audio_inL = REG_SSC_RHR;
    REG_SSC_THR = audio_out2;
    audio_out2 = audio_outL; // Why is that mandatory to have the L and R in sync???
    
    compute_audio_sample();
    
    NVIC_ClearPendingIRQ(SSC_IRQn); // next sample is allready here, no need to go to an other interuption to get it (it save time)
    
    status = REG_SSC_SR;
    audio_inR = audio_inR2; // to get the L and R in phase
    audio_inR2 = REG_SSC_RHR;
    REG_SSC_THR = audio_outR;
  }
  #ifdef syncro_out
    test1_off();
  #endif
}

