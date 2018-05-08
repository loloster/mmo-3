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

// DATA format :
// audio data : uint32_t : (32 significant bit)
//
// modulation coef : int32_t
// 0 for no mudulation
// 0x0000FFFF for highest modulation (16 significant bits + sign)
// 0x8000FFFF for lowest modulation
//
// pitch : uint32_t
// 0 for lowest frequency
// 0x0FFFFFFF for highest freq (28 significat bits)
// half note every 0xO0100000
// 256 half tone (twice midi range)
// midi note range : -126 to 130;
//       from 0.005Hz to about 15KHz
//       69 is 440Hz


union uint64 {
  uint64_t U64;
  uint32_t U32[2];
};

union sound {
  int32_t U32[2];
};

enum adc_pos {
  VCO1_FQ,
  VCO1_MOD1,
  VCO1_MOD2,
  VCO1_MOD3,
  PORTAMENTO_VALUE,
  VCO2_FQ,
  VCO2_MOD1,
  VCO2_MOD2,
  VCO2_MOD3,
  VCO3_FQ,
  VCO3_MOD1,
  VCO3_MOD2,
  VCO3_MOD3,
  LFO1_FQ,
  LFO1_WF,
  LFO1_SYM,
  ADSR_A,
  LFO2_1,
  LFO2_2,
  LFO2_3,
  ADSR_D,
  LFO3_1,
  LFO3_2,
  LFO3_3,
  ADSR_S,
  MIX_1,
  MIX_2,
  MIX_3,
  ADSR_R,
  VCA_GAIN,
  XY_X,
  XY_Y,
  EXT_1
};

enum modulation_value {
  mod_VCO1,               //0xA0  1
  mod_VCO2,               //0xA1  2
  mod_VCO3,               //0xA2  3
  mod_LFO1,               //0xA3  4
  mod_LFO2,               //0xA4  5
  mod_LFO3,               //0xA5  6
  mod_ADSR,               //0xA6  7
  mod_EXT1, // mod "CV"   //0xA7  8
  mod_LR, // mod "LINE IN"//0xA8  9
  mod_XY, // joystick     //0xA9  10
  mod_L, // audio In Left //0xAA  11
  mod_R,                  //0xAB  12
  mod_1, // Automodulation ?? //0xAC  13
  mod_0, // not used, just to have 16 value in this list //0xAD
  mod_00,                 //0xAE
  mod_000                 //0xAF
};

enum modulation_adresse {
  index_VCO1_MOD1,
  index_VCO1_MOD2,
  index_VCO1_MOD3,
  index_VCO2_MOD1,
  index_VCO2_MOD2,
  index_VCO2_MOD3,
  index_VCO3_MOD1,
  index_VCO3_MOD2,
  index_VCO3_MOD3,
  index_Xm,
  index_Xp,
  index_Ym,
  index_Yp,
  index_0, // not used, just to have 16 value in this list
  index_00,
  index_000
};

