#include <Arduino.h>
#include "wavetablesFixedPoint.h"
#include "Adafruit_FreeTouch.h"
Adafruit_FreeTouch qt_1 = Adafruit_FreeTouch(A1, OVERSAMPLE_16, RESISTOR_50K, FREQ_MODE_NONE);
Adafruit_FreeTouch qt_6 = Adafruit_FreeTouch( A6, OVERSAMPLE_16, RESISTOR_50K, FREQ_MODE_NONE);
Adafruit_FreeTouch qt_7 = Adafruit_FreeTouch(A7, OVERSAMPLE_16, RESISTOR_50K, FREQ_MODE_NONE);
Adafruit_FreeTouch qt_8 = Adafruit_FreeTouch( A8, OVERSAMPLE_16, RESISTOR_50K, FREQ_MODE_NONE);

#define SAMPLE_RATE 48000
#define WAVE_TABLE_SIZE 8192

const uint32_t maxAnalogIn = 4095;
const uint32_t shiftfactor = 1024;

struct OscData {
  // everything phase related
  uint32_t inc = 0;
  uint32_t phase_accumulator = 0;
  
  // the rest
  uint32_t waveform1 = maxAnalogIn;
  uint32_t waveform2 = 0;

  uint32_t crossFMint = 0;
  uint32_t crossFM = 0;
  uint32_t volume = 0;
};

volatile struct OscData osc1, osc2;
const unsigned int wave_max = WAVE_TABLE_SIZE - 1;
const unsigned int wave_max_shifted = WAVE_TABLE_SIZE  * shiftfactor;
uint32_t sample2 = 0;

// ======================================================
// SETUP STUFF
// ======================================================

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);

  Serial.begin(115200);
  
  tcConfigure(SAMPLE_RATE);
  
  qt_1.begin();
  qt_6.begin();
  qt_7.begin();
  qt_8.begin();
}

void tcConfigure(int sampleRate) {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |  // Connect GCLK0 at 48MHz as a clock source for TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;

  NVIC_SetPriority(TC4_IRQn, 0);  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);       // Connect the TC4 timer to the Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;     // Enable compare overflow (OVF) interrupt
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_WAVEGEN_MFRQ;  // Set-up TC4 timer for Match Frequency mode (MFRQ)
  TC4->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;  // Enable timer TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
}

// ======================================================
// AUDIO GENERATION INTERRUPT
// ======================================================

void TC4_Handler() {
  
    uint32_t value1, value2;
    uint32_t tableIndex;
    
    uint32_t sample1 = 0.f;
    uint32_t accumulator;
    
    // -------- osc 1 ---------------------
    accumulator = osc1.phase_accumulator;    
    accumulator += osc1.inc;
    
    if (accumulator >= wave_max_shifted) {
      accumulator -= wave_max_shifted;
    }

    //if (osc2.crossFM > 2) {
      accumulator += ((sample2 * osc2.crossFM)/maxAnalogIn) << 10;//* shiftfactor;
      if (accumulator >= wave_max_shifted) {
        accumulator -= wave_max_shifted;
      }
    //}

    tableIndex = accumulator >> 10;// / shiftfactor;
    value1 = sinetable[tableIndex];
    value2 = sine2table[tableIndex];
    
    osc1.phase_accumulator = accumulator;
    sample1 = ( ((value1 * osc1.waveform1) >> 6 ) + ((value2 * osc1.waveform2) >> 6)) / maxAnalogIn;

    // -------- osc 2 ---------------------
    accumulator = osc2.phase_accumulator;    
    accumulator += osc2.inc;
    
    if (accumulator >= wave_max_shifted) {
      accumulator -= wave_max_shifted;
    }

    //if (osc1.crossFM > 2) {
      
      accumulator += ((sample1 * osc1.crossFM)/maxAnalogIn) << 10;//*shiftfactor;
      if (accumulator >= wave_max_shifted) {
        accumulator -= wave_max_shifted;
      }      
    //}

    tableIndex = accumulator >> 10; // / shiftfactor;
    value1 = sinetable[tableIndex];  
    value2 = sawtable[tableIndex];
    
    osc2.phase_accumulator = accumulator;
    sample2 = (((value1 * osc2.waveform1) >> 6) + ((value2 * osc2.waveform2) >> 6)) / maxAnalogIn;
     
    DAC->DATA.reg =  ((sample1*osc1.volume + sample2*osc2.volume) / maxAnalogIn) >> 1;

    TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

// ======================================================
// MAIN LOOP
// ======================================================

uint16_t counter = 0;
//const uint16_t incFactor = WAVE_TABLE_SIZE / SAMPLE_RATE;

void loop() {
  counter++;

  if (counter > 1000) { // we not so often sample the inputs but I don't want to use delay() which might interact with the interrupt
    // ===================================================
    // reading the inputs
    // ===================================================
    int QT1 = qt_1.measure(); //A1
    QT1 = map(QT1,650,920,0,4095);
    if (QT1 <=0) QT1 =0;
    if (QT1 >=4095) QT1=4095;

    int QT6 = qt_6.measure(); //A6
    QT6 = map(QT6,420,680,0,4095);
    if (QT6 <=0) QT6 =0;
    if (QT6 >=4095) QT6 =4095;

    int QT7 = qt_7.measure(); //A7
    QT7 = map(QT7,380,720,0,4095);
    if (QT7 <=0) QT7 =0;
    if (QT7 >=4095) QT7 =4095;

    int QT8 = qt_8.measure(); //A8
    QT8 = map(QT8,410,990,0,4095);
    if (QT8 <=0) QT8 =0;
    if (QT8 >=4095) QT8 =4095;
    

    //QT1           QT8
    //   A5  A2  A9
    //    A5   A3
    // QT6      QT7
    osc1.inc = ((((analogRead(A5) << 13) >> 1) + 0) / SAMPLE_RATE) << 10;

    uint32_t waveform = QT1;
    osc1.waveform2 = waveform;
    osc1.waveform1 = maxAnalogIn - waveform;
    osc1.crossFM  = analogRead(A4);
    osc1.volume = QT6;

    //osc2.inc = ((frequency * WAVE_TABLE_SIZE) / SAMPLE_RATE) * shiftfactor;
    osc2.inc = ((((QT8 << 13) >> 1) + 0) / SAMPLE_RATE) << 10; // <<13 is the same as * WAVE_TABLE_SIZE
    
    waveform = analogRead(A9);
    osc2.waveform2 = waveform;
    osc2.waveform1 = maxAnalogIn - waveform;
    osc2.crossFM = analogRead(A3);    
    osc2.volume= QT7;
    
    Serial.print(QT1);
    Serial.print("\t");
    Serial.print(QT6);
    Serial.print("\t");
    Serial.print(QT7);
    Serial.print("\t");
    Serial.println(QT8);
    
    counter = 0;
  }
}