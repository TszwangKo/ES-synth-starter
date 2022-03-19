#include <Arduino.h>
#include <U8g2lib.h>
#include <assert.h>
#include <string>
#include <STM32FreeRTOS.h>

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

uint8_t readCol(uint8_t Col){
  digitalWrite(REN_PIN,HIGH);
  switch(Col){
    case 0:
      return digitalRead(C0_PIN);
      break;
    case 1:
      return digitalRead(C1_PIN);
      break;
    case 2:
      return digitalRead(C2_PIN);
      break;
    case 3:
      return digitalRead(C3_PIN);
      break;
    default:
      return 1;
  }
  digitalWrite(REN_PIN,LOW);
}
uint8_t readCols(){
  digitalWrite(REN_PIN,HIGH);
  uint8_t c0 = digitalRead(C0_PIN), c1 = digitalRead(C1_PIN), c2 = digitalRead(C2_PIN), c3 = digitalRead(C3_PIN);
  return c0 + (c1<<1) + (c2 << 2) + (c3 <<3);
  digitalWrite(REN_PIN,LOW);
}



//defining step size for diferrent notes, Starting at C and ends at B
const int32_t stepSizes[] = {50953930,54113197,57330935,60740010,64351798,68178356,72232452,76527617,81078186,85899345,91007185,96418753};

volatile int32_t currentStepSize;

// Mutex created to handle asynchronous accesses to keyArray
SemaphoreHandle_t keyArrayMutex;
volatile uint8_t *keyArray = new uint8_t[8];
volatile uint8_t knob3Rotation = 8;

void updateKeyArray(){
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  
  //assertions to be added 
  for ( int i = 0; i < 8 ; i++ ){
    setRow(i);
    delayMicroseconds(3);
    keyArray[i] = readCols();
  }

  xSemaphoreGive(keyArrayMutex);
}

void checkKeyPress(){
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  const char* notes[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  int32_t localStepSize = 0;

  uint8_t CtoEb = keyArray[0];
  uint8_t EtoAb = keyArray[1];
  uint8_t AbtoB = keyArray[2];

  uint16_t CtoB = CtoEb + (EtoAb<<4) + (AbtoB<<8);
  if (CtoB != 0x0FFF){
    for(int note = 0 ; note < 12; note++){
      uint16_t mask = 0x1 << note;
      if (!(mask & CtoB)){
        localStepSize = stepSizes[note];
        break;
      }else{
        continue;
      }
    }
  }else{
    localStepSize = 0;
  }
  xSemaphoreGive(keyArrayMutex);
  __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
}

void decode_knob3(bool isSetUp=false){
  static uint8_t prevA;
  static uint8_t prevB;
  static int8_t prevRes;
  uint8_t currA;
  uint8_t currB;
  int8_t res;


  if(isSetUp){
    setRow(3);
    delayMicroseconds(3);
    prevA = readCol(0);
    prevB = readCol(1);
    prevRes = 0;
  }else{
    setRow(3);
    delayMicroseconds(3);
    currA = readCol(0);
    currB = readCol(1);
    
    uint8_t combined = prevA + (prevB << 1) + (currA << 2) + (currB << 3);
    
    if( combined == 0b0010 || combined == 0b1011 || combined == 0b1101 || combined == 0b0100){
      res = 1;
    }else if(combined == 0b0001 || combined == 0b1000 || combined == 0b1110 || combined == 0b0111){
      res = -1;
    }else if(combined == 0b0000||combined == 0b0101||combined == 0b1010|| combined == 0b1111){
      res = 0;
    }else{
      if (prevRes > 0)  res = +2;
      else if (prevRes < 0) res = -2;
      else res = 0;
    }

  }
  prevA = currA;
  prevB = currB;
  prevRes = res;

  if(knob3Rotation + res >=16) __atomic_store_n(&knob3Rotation, 16, __ATOMIC_RELAXED);
  else if (knob3Rotation + res <= 0) __atomic_store_n(&knob3Rotation, 0, __ATOMIC_RELAXED);
  else __atomic_store_n(&knob3Rotation, knob3Rotation+res, __ATOMIC_RELAXED);
}

const char* checkNote(){
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  const char* notes[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  int32_t localStepSize = 0;

  uint8_t CtoEb = keyArray[0];
  uint8_t EtoAb = keyArray[1];
  uint8_t AbtoB = keyArray[2];

  uint16_t CtoB = CtoEb + (EtoAb<<4) + (AbtoB<<8);
  const char* res = "None";

  for(int note = 0 ; note < 12; note++){
    uint16_t mask = 0x1 << note;
    if (!(mask & CtoB)){
      res = notes[note];
      break;
    }else{
      continue;
    }
  }

  xSemaphoreGive(keyArrayMutex);
  return res;
}

void scanKeysTask(void * pvParameters) {
  // Setting up initiation interval to 50ms
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  // Get current time, used in next function call
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    //Blocks call until xFrequncy time has passed
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    updateKeyArray();
    checkKeyPress();
    decode_knob3();
    Serial.println(knob3Rotation);
  }
}

void displayUpdateTask(void * pvParameters){
  // Setting up initiation interval to 50ms
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  // Get current time, used in next function call
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1){
    //Blocks call until xFrequncy time has passed
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    // u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory

    const char* note = checkNote();
    u8g2.drawStr(2,30,note);
    u8g2.setCursor(32,30);
    u8g2.print(currentStepSize);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for ( int i = 0; i < 8;i++){
      u8g2.setCursor(2+(int)i*10,20);
      u8g2.print(keyArray[i],HEX);
    }
    xSemaphoreGive(keyArrayMutex);

    u8g2.drawStr(2,10,"Vol:");
    u8g2.setCursor(32,10);
    u8g2.print(knob3Rotation);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void sampleISR(){
  static int32_t phaseAcc=0;
  phaseAcc += currentStepSize;
  int32_t Vout = phaseAcc >> 24;
  Vout = Vout >> (8 - knob3Rotation/2);
  analogWrite(OUTR_PIN, Vout+128);
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

  //Initialise Knob position;
  decode_knob3(true);
  //Initialise UART
  Serial.begin(9600);

  //Creates the Clock instance
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  Serial.println("Clock Created");

  //Starts the Clock
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  Serial.println("Clock Starts");

  //set up RTOS
  TaskHandle_t scanKeysHandle = NULL; 
  xTaskCreate(
    scanKeysTask, /* Function that implements the task */ 
    "scanKeys", /* Text name for the task */
    64, /* Stack size in words, not bytes */ 
    NULL, /* Parameter passed into the task */
    2, /* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  ); 

  TaskHandle_t displayUpdateHandle = NULL; 
  xTaskCreate(
    displayUpdateTask, /* Function that implements the task */ 
    "displayUpdate", /* Text name for the task */
    256, /* Stack size in words, not bytes */ 
    NULL, /* Parameter passed into the task */
    1, /* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  ); 

  keyArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;
  }
}