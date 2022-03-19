#include <Arduino.h>
#include <U8g2lib.h>
#include <assert.h>
#include <string>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx){
  constexpr std::uint8_t mask0{ 1 << 0 }; // 0000 0001
  constexpr std::uint8_t mask1{ 1 << 1 }; // 0000 0010
  constexpr std::uint8_t mask2{ 1 << 2 }; // 0000 0100
  uint8_t r0 = rowIdx & mask0, r1 = rowIdx & mask1, r2 = rowIdx & mask2;
  digitalWrite(RA0_PIN,r0);
  digitalWrite(RA1_PIN,r1);
  digitalWrite(RA2_PIN,r2);
}

uint8_t readCols(){
  digitalWrite(REN_PIN,HIGH);
  uint8_t c0 = digitalRead(C0_PIN), c1 = digitalRead(C1_PIN), c2 = digitalRead(C2_PIN), c3 = digitalRead(C3_PIN);
  return c0 + (c1<<1) + (c2 << 2) + (c3 <<3);
  digitalWrite(REN_PIN,LOW);
}

//defining step size for diferrent notes, Starting at C and ends at B
const int32_t stepSizes[] = {50953930,54113197,573309352,60740010,6435179,68178356,72232452,76527617,81078186,85899345,105662681,129973076};

volatile int32_t currentStepSize;

void updateKeyArray(uint8_t* keyArray){
  //assertions to be added 
  for ( int i = 0; i < 8 ; i++ ){
    setRow(i);
    delayMicroseconds(3);
    keyArray[i] = readCols();
  }
}

const char* getNote(){
    const char* notes[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    
}

const char* checkKeyPress(uint8_t* keyArray){
  const char* notes[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

  uint8_t CtoEb = keyArray[0];
  uint8_t EtoAb = keyArray[1];
  uint8_t AbtoB = keyArray[2];

  uint16_t CtoB = CtoEb + (EtoAb<<4) + (AbtoB<<8);
  const char* res = "None";
  if (CtoB != 0x0FFF){
    for(int note = 0 ; note < 12; note++){
      uint16_t mask = 0x1 << note;
      if (!(mask & CtoB)){
        currentStepSize = stepSizes[note];
        res = notes[note];
        break;
      }else{
        continue;
      }
    }
  }else{
    currentStepSize = 0;
  }
  return res;
}


void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    uint8_t *keyArray = new uint8_t[8];
    updateKeyArray(keyArray);
    const char* note = checkKeyPress(keyArray);
    u8g2.drawStr(2,30,note);
    u8g2.setCursor(32,30);
    u8g2.print(currentStepSize);
    uint8_t keys = readCols();
    for ( int i = 0; i < 8;i++){
      u8g2.setCursor(2+(int)i*10,20);
      u8g2.print(keyArray[i],HEX);
    }

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link. 
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();
}